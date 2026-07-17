// fec_decoder.h — RaptorQ generation decoder for the video link (RX side).
//
// SYNC-HARDENED (v2). The original window logic trusted a SINGLE packet's
// generation id: any accepted packet whose id read as "future" instantly
// re-baselined the window and evicted everything. One corrupted-but-accepted
// header (e.g. a flipped high bit in gen_id during an overload event) jumped
// the baseline up to +32767 generations forward, after which every REAL
// generation looked "old" and was silently dropped — the link stayed dead at
// the FEC layer with perfect RF until the RX process was restarted. A TX
// restart (ids back to 0) wedged the RX the same way from the other side.
//
// Two mechanisms fix this, both RX-only and wire-compatible (TX untouched):
//
//  1) JUMP QUARANTINE — a forward jump larger than LARGE_JUMP_GENS does not
//     move the baseline by itself. The id becomes a candidate; only after
//     JUMP_CONFIRM_PACKETS packets land within +/-WINDOW_GENERATIONS of the
//     candidate is the jump believed (real traffic clusters; corrupted ids
//     don't). Unconfirmed outliers are dropped WITHOUT creating decoder
//     state, so garbage cannot pollute the generation map either.
//
//  2) RE-ACQUIRE WATCHDOG — if REACQUIRE_AFTER_OLD consecutive packets are
//     dropped as "older than the window", the decoder concludes its baseline
//     is desynced (a bad jump slipped through, or the TX restarted), drops
//     all in-flight state, and re-seeds from the next packet. At ~230 pkt/s
//     that is ~1.3 s of self-healing instead of a manual RX restart.
//
// Diagnostics (sync_acquired / sync_baseline / gens_in_flight / resync_count
// / jump_accepts) are exposed so a TUI line can make resyncs visible.

#pragma once
#include "fec_common.h"
#include "link_stats.h"
#include <RaptorQ/RaptorQ_v1_hdr.hpp>
#include <cstring>
#include <map>
#include <memory>
#include <vector>

namespace fec {

class FecDecoder {
public:
    static constexpr unsigned WINDOW_GENERATIONS   = 4;
    //! Forward jumps up to this many generations are accepted immediately
    //! (normal flow advances by ~1; even a multi-second dropout only skips
    //! tens). Anything larger must be corroborated first.
    static constexpr unsigned LARGE_JUMP_GENS      = 64;
    //! Packets that must agree (within +/-WINDOW of the candidate) before a
    //! large jump re-baselines the window.
    static constexpr unsigned JUMP_CONFIRM_PACKETS = 3;
    //! Consecutive "older than window" drops before the decoder assumes its
    //! baseline is wrong and re-acquires from scratch. ~1.3 s at 230 pkt/s.
    static constexpr unsigned REACQUIRE_AFTER_OLD  = 300;

    FecDecoder() : m_have_baseline(false), m_baseline_gen(0) {}

    void set_stats(stats::LinkStats* s) { m_stats = s; }

    std::vector<std::vector<unsigned char>>
    process_phy_packet(const std::vector<unsigned char>& phy) {
        std::vector<std::vector<unsigned char>> out;
        if (phy.size() != PHY_PACKET_BYTES) return out;

        WireHeader h;
        std::memcpy(&h, phy.data(), HEADER_BYTES);
        const uint16_t gen_id = ntoh16(h.generation_id_be);
        const uint32_t esi    = ntoh32(h.esi_be) & 0x00FFFFFF;
        const uint16_t K      = ntoh16(h.k_be);

        if (K != SOURCE_SYMBOLS_PER_GEN) return out;
        if (esi >= (1u << 24)) return out;

        if (!m_have_baseline) {
            m_baseline_gen  = gen_id;
            m_have_baseline = true;
        }

        // ---- sync guard -------------------------------------------------
        // age > 0  <=>  gen_id is behind the window baseline (wraparound-
        // aware). These are dropped, but COUNTED: a long unbroken run of
        // "old" packets means the baseline itself is wrong (bad jump got
        // believed, or the TX restarted its ids) -> re-acquire.
        const int16_t age = static_cast<int16_t>(m_baseline_gen - gen_id);
        if (age > 0) {
            if (++m_consecutive_old >= REACQUIRE_AFTER_OLD)
                resync();   // next packet re-seeds the baseline
            return out;
        }

        // Forward distance from the baseline. In this branch it is in
        // [0, 32767], except the exact half-range edge where both age and
        // advance read -32768 — treat that as a suspicious jump too.
        const int16_t advance = static_cast<int16_t>(gen_id - m_baseline_gen);
        if (advance < 0 ||
            advance > static_cast<int16_t>(LARGE_JUMP_GENS)) {
            // ---- jump quarantine ----------------------------------------
            const int16_t cand_dist =
                static_cast<int16_t>(gen_id - m_jump_candidate);
            const int w = static_cast<int>(WINDOW_GENERATIONS);
            if (m_jump_votes > 0 && cand_dist >= -w && cand_dist <= w) {
                ++m_jump_votes;
            } else {
                m_jump_candidate = gen_id;   // new candidate region
                m_jump_votes     = 1;
            }
            if (m_jump_votes < JUMP_CONFIRM_PACKETS)
                return out;   // lone outlier: dropped, NOT ingested

            // Corroborated: believe the jump, move the window there.
            set_baseline_evict(static_cast<uint16_t>(
                gen_id - (WINDOW_GENERATIONS - 1)));
            ++m_jump_accepts;
            m_jump_votes = 0;
            // fall through and ingest this packet
        }
        m_consecutive_old = 0;

        // ---- ingest (unchanged decode path) -----------------------------
        GenState& g = m_gens[gen_id];
        if (!g.decoder) g.init();
        if (g.completed) return out;

        std::vector<uint8_t> sym(SYMBOL_SIZE);
        std::memcpy(sym.data(), phy.data() + HEADER_BYTES, SYMBOL_SIZE);
        auto it  = sym.begin();
        auto end = sym.end();
        g.decoder->add_symbol(it, end, esi);
        ++g.symbols_seen;

        if (g.symbols_seen >= SOURCE_SYMBOLS_PER_GEN)
            attempt_decode(g, out);

        advance_window(gen_id);
        return out;
    }

    // ---- sync diagnostics (for TUI / tests) -----------------------------
    bool     sync_acquired()  const { return m_have_baseline; }
    uint16_t sync_baseline()  const { return m_baseline_gen; }
    size_t   gens_in_flight() const { return m_gens.size(); }
    uint64_t resync_count()   const { return m_resyncs; }      // watchdog fires
    uint64_t jump_accepts()   const { return m_jump_accepts; } // confirmed jumps

private:
    struct GenState {
        using Decoder = RaptorQ__v1::Decoder<
            std::vector<uint8_t>::iterator,
            std::vector<uint8_t>::iterator>;
        std::unique_ptr<Decoder> decoder;
        unsigned symbols_seen = 0;
        bool     completed    = false;

        void init() {
            decoder.reset(new Decoder(
                RaptorQ__v1::Block_Size::Block_32,
                SYMBOL_SIZE,
                RaptorQ__v1::Dec_Report::COMPLETE));
        }
    };

    void attempt_decode(GenState& g,
                        std::vector<std::vector<unsigned char>>& out) {
        // Tell the decoder no more symbols are coming for this attempt,
        // then wait for the computation to finish. Without these two calls
        // decode_bytes returns nothing.
        g.decoder->end_of_input(RaptorQ__v1::Fill_With_Zeros::NO);
        auto wait = g.decoder->wait_sync();
        if (wait.error != RaptorQ__v1::Error::NONE)
            return;   // not decodable yet (not enough symbols)

        std::vector<uint8_t> recovered(
            SOURCE_SYMBOLS_PER_GEN * SYMBOL_SIZE, 0);
        auto it  = recovered.begin();
        auto end = recovered.end();
        auto res = g.decoder->decode_bytes(it, end, 0, 0);
        if (res.written != SOURCE_SYMBOLS_PER_GEN * SYMBOL_SIZE)
            return;
        g.completed = true;

        if (m_stats) {
            // We can't easily know source vs repair from the decoder alone;
            // approximate: source_used = min(symbols_seen, K), rest = repair.
            unsigned src = std::min<unsigned>(g.symbols_seen,
                                              SOURCE_SYMBOLS_PER_GEN);
            unsigned rep = (g.symbols_seen > SOURCE_SYMBOLS_PER_GEN)
                         ? (g.symbols_seen - SOURCE_SYMBOLS_PER_GEN) : 0;
            m_stats->note_fec_gen_decoded(src, rep);
        }

        for (size_t i = 0; i < SOURCE_SYMBOLS_PER_GEN; ++i) {
            const uint8_t* p = &recovered[i * SYMBOL_SIZE];
            uint16_t len_be;
            std::memcpy(&len_be, p, 2);
            const uint16_t len = ntoh16(len_be);
            if (len == 0 || len > MAX_RTP_PAYLOAD) continue;
            out.emplace_back(p + 2, p + 2 + len);
        }
    }

    //! Move the baseline and evict everything behind it (shared by the
    //! normal advance and the confirmed-jump path).
    void set_baseline_evict(uint16_t new_baseline) {
        for (auto it = m_gens.begin(); it != m_gens.end();) {
            const int16_t a =
                static_cast<int16_t>(new_baseline - it->first);
            if (a > 0) {
                if (!it->second.completed && m_stats)
                    m_stats->note_fec_gen_dropped();
                it = m_gens.erase(it);
            } else ++it;
        }
        m_baseline_gen = new_baseline;
    }

    void advance_window(uint16_t arrived_gen) {
        const int16_t advance =
            static_cast<int16_t>(arrived_gen - m_baseline_gen);
        if (advance > 0
            && static_cast<unsigned>(advance) >= WINDOW_GENERATIONS) {
            set_baseline_evict(static_cast<uint16_t>(
                arrived_gen - (WINDOW_GENERATIONS - 1)));
        }
    }

    //! Full re-acquire: drop all in-flight state and re-seed the baseline
    //! from the next packet. Fired by the watchdog when the window is
    //! provably wrong (a sustained run of "old" drops).
    void resync() {
        for (auto& kv : m_gens)
            if (!kv.second.completed && m_stats)
                m_stats->note_fec_gen_dropped();
        m_gens.clear();
        m_have_baseline   = false;
        m_jump_votes      = 0;
        m_consecutive_old = 0;
        ++m_resyncs;
    }

    bool     m_have_baseline;
    uint16_t m_baseline_gen;
    std::map<uint16_t, GenState> m_gens;
    stats::LinkStats* m_stats = nullptr;

    // sync-hardening state
    uint16_t m_jump_candidate  = 0;
    unsigned m_jump_votes      = 0;
    unsigned m_consecutive_old = 0;
    uint64_t m_resyncs         = 0;
    uint64_t m_jump_accepts    = 0;
};

}  // namespace fec

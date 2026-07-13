/*!
 * \file phy_spreader.h
 * \brief TX-side block interleaver for FEC PHY packets (burst-loss spreading), v2.
 *
 * PROBLEM: the FEC emits each generation's packets as a contiguous run on the
 * air, so one fade/shadowing burst wipes a contiguous chunk of ONE generation.
 * If the burst exceeds R packets, that generation is unrecoverable — the
 * "generations lost in clumps while moving" pattern.
 *
 * FIX: buffer `depth` generations and emit them COLUMN-MAJOR, so consecutive
 * on-air packets belong to different generations. A burst of L consecutive
 * lost packets then costs any single generation only ~ceil(L/depth) packets;
 * as long as that is <= R, the repair symbols recover everything.
 *     burst tolerance ~ depth * R packets
 *     added latency   ~ depth * generation-cadence (min(fill time, FEC flush))
 *
 * v2 CHANGES (both matter — v1 was silently wrong with the real TX):
 *
 *  1) ROWS ARE KEYED BY GENERATION ID, NOT PACKET COUNT. The FEC flush timer
 *     closes generations PARTIALLY (fewer than SYMBOLS_PER_GEN packets), so a
 *     row that waits for a fixed width never completes and generations bleed
 *     into each other. v2 parses the generation id from the FEC wire header
 *     (fec_common.h WireHeader: uint16_t generation_id_be at offset 0,
 *     big-endian) and starts a new row whenever the id changes. Correct for
 *     full and partial generations alike; no `width` parameter.
 *
 *  2) FLUSH ONLY ON TRUE IDLE. flush_partial() must be called when the video
 *     stream has genuinely paused (no data for a few hundred ms) — NEVER on
 *     the periodic FEC flush. Flushing the spreader every FEC-flush period
 *     collapses the effective depth to ~1 and you pay latency for nothing.
 *     During active video the block fills and emits by itself.
 *
 * RX NEEDS NO CHANGES: packets are self-identifying (generation id + esi in
 * the header), so the decoder reassembles by id regardless of arrival order.
 *
 * Usage in video_tx.cpp (see integration notes):
 *     static PhySpreader g_spreader(2);              // depth
 *     ... g_spreader.push(g_enc.next_phy_packet());  // instead of send
 *     ... while (g_spreader.pop(p)) tx.send_frame(p, PHY_RATE);
 *     ... on IDLE only: g_spreader.flush_partial(); then drain pop().
 */

#ifndef PHY_SPREADER_H
#define PHY_SPREADER_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
#include <utility>

class PhySpreader {
public:
    //! depth: number of generations to interleave across.
    //! Burst tolerance ~ depth * R packets; latency ~ depth * generation-cadence.
    explicit PhySpreader(int depth)
        : m_depth(depth < 1 ? 1 : depth) {}

    //! Feed one FEC PHY packet in the encoder's natural emit order. When
    //! `depth` COMPLETED generations are buffered, they are interleaved into
    //! the output queue automatically. Drain with pop().
    void push(std::vector<unsigned char>&& pkt) {
        uint16_t gen = parse_gen_id(pkt);

        if (m_rows.empty() || gen != m_rows.back().gen_id) {
            // The previous row (if any) is now COMPLETE — a new generation
            // has started. Emit once `depth` completed rows are buffered.
            if (!m_rows.empty() &&
                static_cast<int>(m_rows.size()) >= m_depth) {
                emit_interleaved();
            }
            m_rows.push_back(Row{gen, {}});
        }
        m_rows.back().pkts.push_back(std::move(pkt));
    }

    //! Pop the next ready-to-send packet (interleaved order). Returns false
    //! when nothing is ready (block still accumulating).
    bool pop(std::vector<unsigned char>& out) {
        if (m_outq.empty()) return false;
        out = std::move(m_outq.front());
        m_outq.pop_front();
        return true;
    }

    //! Emit everything currently buffered, interleaved over however many
    //! generations are present. CALL ONLY ON TRUE IDLE (stream paused) or at
    //! shutdown — never on the periodic FEC flush, or the effective depth
    //! collapses and the latency buys nothing.
    void flush_partial() {
        emit_interleaved();
    }

    //! Diagnostics: generations currently buffered (not yet emitted).
    size_t buffered_generations() const { return m_rows.size(); }
    //! Diagnostics: packets waiting in the output queue.
    size_t pending_out() const { return m_outq.size(); }

private:
    struct Row {
        uint16_t gen_id;
        std::vector<std::vector<unsigned char>> pkts;
    };

    //! WireHeader (fec_common.h): uint16_t generation_id_be at offset 0.
    static uint16_t parse_gen_id(const std::vector<unsigned char>& pkt) {
        if (pkt.size() < 2) return 0;
        return static_cast<uint16_t>((pkt[0] << 8) | pkt[1]);   // big-endian
    }

    //! Column-major readout over all buffered rows (ragged rows handled:
    //! shorter/partial generations simply drop out of later columns).
    void emit_interleaved() {
        size_t max_len = 0;
        for (auto& r : m_rows) max_len = std::max(max_len, r.pkts.size());
        for (size_t c = 0; c < max_len; ++c)
            for (auto& r : m_rows)
                if (c < r.pkts.size())
                    m_outq.push_back(std::move(r.pkts[c]));
        m_rows.clear();
    }

    int m_depth;
    std::vector<Row> m_rows;                          // buffered generations
    std::deque<std::vector<unsigned char>> m_outq;    // interleaved, ready to send
};

#endif // PHY_SPREADER_H

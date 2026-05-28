// fec_decoder.h — RX-side FEC, corrected for the real libRaptorQ API.
//
// Real libRaptorQ API (verified):
//   Decoder(Block_Size, symbol_size, Dec_Report)   // Dec_Report, not Decoder_Type
//   decoder.add_symbol(iterator&, end, esi)         // returns Error
//   decoder.decode_bytes(iterator&, end, from, skip) // returns Decoder_written
//   Dec_Report::PARTIAL_FROM_BEGINNING / PARTIAL_ANY / COMPLETE
//
// API exposed to your program:
//   FecDecoder dec;
//   auto recovered = dec.process_phy_packet(vector<unsigned char>);
//   for (auto& rtp : recovered) { ... }

#pragma once
#include "fec_common.h"
#include <RaptorQ/RaptorQ_v1_hdr.hpp>
#include <cstring>
#include <map>
#include <memory>
#include <vector>

namespace fec {

class FecDecoder {
public:
    static constexpr unsigned WINDOW_GENERATIONS = 4;

    FecDecoder() : m_have_baseline(false), m_baseline_gen(0) {}

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

        if (!m_have_baseline) { m_baseline_gen = gen_id; m_have_baseline = true; }
        const int16_t age = static_cast<int16_t>(m_baseline_gen - gen_id);
        if (age > 0 && age < 32768) return out;

        GenState& g = m_gens[gen_id];
        if (!g.decoder) g.init();
        if (g.completed) return out;

        // Copy symbol bytes into a working buffer libRaptorQ can consume.
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

        for (size_t i = 0; i < SOURCE_SYMBOLS_PER_GEN; ++i) {
            const uint8_t* p = &recovered[i * SYMBOL_SIZE];
            uint16_t len_be;
            std::memcpy(&len_be, p, 2);
            const uint16_t len = ntoh16(len_be);
            if (len == 0 || len > MAX_RTP_PAYLOAD) continue;
            out.emplace_back(p + 2, p + 2 + len);
        }
    }

    void advance_window(uint16_t arrived_gen) {
        const int16_t advance =
            static_cast<int16_t>(arrived_gen - m_baseline_gen);
        if (advance > 0
            && static_cast<unsigned>(advance) >= WINDOW_GENERATIONS) {
            const uint16_t new_baseline = static_cast<uint16_t>(
                arrived_gen - (WINDOW_GENERATIONS - 1));
            for (auto it = m_gens.begin(); it != m_gens.end();) {
                const int16_t a =
                    static_cast<int16_t>(new_baseline - it->first);
                if (a > 0) it = m_gens.erase(it);
                else ++it;
            }
            m_baseline_gen = new_baseline;
        }
    }

    bool     m_have_baseline;
    uint16_t m_baseline_gen;
    std::map<uint16_t, GenState> m_gens;
};

}  // namespace fec
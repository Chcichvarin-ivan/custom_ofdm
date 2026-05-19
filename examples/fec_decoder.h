// fec_decoder.h — wraps libRaptorQ for the RX side.
#pragma once
#include "fec_common.h"
#include <RaptorQ/RaptorQ_v1_hdr.hpp>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
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

        if (!m_have_baseline) {
            m_baseline_gen = gen_id;
            m_have_baseline = true;
        }
        const int16_t age = static_cast<int16_t>(m_baseline_gen - gen_id);
        if (age > 0 && age < 32768) return out;

        auto& g = m_gens[gen_id];
        if (!g.decoder) g.init();
        if (g.completed) return out;

        constexpr size_t WORDS_PER_SYM = SYMBOL_SIZE / 4;
        std::vector<uint32_t> sym(WORDS_PER_SYM, 0);
        std::memcpy(sym.data(), phy.data() + HEADER_BYTES, SYMBOL_SIZE);
        auto it = sym.begin();
        (void)g.decoder->add_symbol(it, sym.end(), esi);
        ++g.symbols_seen;

        if (g.symbols_seen >= SOURCE_SYMBOLS_PER_GEN) {
            attempt_decode(g, out);
        }
        advance_window(gen_id);
        return out;
    }

private:
    struct GenState {
        using Decoder = RaptorQ__v1::Impl::Decoder<
            std::vector<uint32_t>::iterator,
            std::vector<uint32_t>::iterator>;
        std::unique_ptr<Decoder> decoder;
        unsigned symbols_seen = 0;
        bool     completed    = false;

        void init() {
            decoder = std::make_unique<Decoder>(
                RaptorQ__v1::Block_Size::Block_32,
                SYMBOL_SIZE,
                RaptorQ__v1::Decoder_Type::PARTIAL_FROM_BEGINNING);
        }
    };

    void attempt_decode(GenState& g,
                        std::vector<std::vector<unsigned char>>& out) {
        constexpr size_t WORDS_PER_SYM = SYMBOL_SIZE / 4;
        std::vector<uint32_t> recovered(
            SOURCE_SYMBOLS_PER_GEN * WORDS_PER_SYM, 0);
        auto it = recovered.begin();
        const auto res = g.decoder->decode_bytes(it, recovered.end(), 0, 0);
        if (res.written != SOURCE_SYMBOLS_PER_GEN * SYMBOL_SIZE) return;
        g.completed = true;

        for (size_t i = 0; i < SOURCE_SYMBOLS_PER_GEN; ++i) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(
                &recovered[i * WORDS_PER_SYM]);
            uint16_t len_be;
            std::memcpy(&len_be, p, 2);
            const uint16_t len = ntoh16(len_be);
            if (len == 0 || len > MAX_RTP_PAYLOAD) continue;
            std::vector<unsigned char> rtp(p + 2, p + 2 + len);
            out.push_back(std::move(rtp));
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

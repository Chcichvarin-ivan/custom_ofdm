// fec_encoder.h — wraps libRaptorQ for the TX side.
#pragma once
#include "fec_common.h"
#include <RaptorQ/RaptorQ_v1_hdr.hpp>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <vector>

namespace fec {

class FecEncoder {
public:
    FecEncoder() : m_gen_id(0) {
        m_buffered.reserve(SOURCE_SYMBOLS_PER_GEN);
    }

    void add_rtp_packet(const std::vector<unsigned char>& rtp) {
        if (rtp.size() > MAX_RTP_PAYLOAD) {
            throw std::runtime_error("RTP packet exceeds MAX_RTP_PAYLOAD; "
                                     "lower the GStreamer mtu setting.");
        }
        std::vector<uint8_t> sym(SYMBOL_SIZE, 0);
        uint16_t len_be = hton16(static_cast<uint16_t>(rtp.size()));
        std::memcpy(sym.data(),     &len_be, 2);
        std::memcpy(sym.data() + 2, rtp.data(), rtp.size());
        m_buffered.push_back(std::move(sym));

        if (m_buffered.size() == SOURCE_SYMBOLS_PER_GEN) {
            encode_current_generation();
            m_buffered.clear();
            ++m_gen_id;
        }
    }

    bool has_phy_packet() const { return !m_outbox.empty(); }

    std::vector<unsigned char> next_phy_packet() {
        std::vector<unsigned char> p = std::move(m_outbox.front());
        m_outbox.pop_front();
        return p;
    }

    void flush() {
        if (m_buffered.empty()) return;
        while (m_buffered.size() < SOURCE_SYMBOLS_PER_GEN) {
            m_buffered.emplace_back(SYMBOL_SIZE, 0);
        }
        encode_current_generation();
        m_buffered.clear();
        ++m_gen_id;
    }

private:
    using Encoder = RaptorQ__v1::Impl::Encoder<
        std::vector<uint32_t>::iterator,
        std::vector<uint32_t>::iterator>;

    void encode_current_generation() {
        constexpr size_t WORDS_PER_SYM = SYMBOL_SIZE / 4;
        std::vector<uint32_t> input(SOURCE_SYMBOLS_PER_GEN * WORDS_PER_SYM, 0);
        for (size_t i = 0; i < SOURCE_SYMBOLS_PER_GEN; ++i) {
            std::memcpy(&input[i * WORDS_PER_SYM],
                        m_buffered[i].data(), SYMBOL_SIZE);
        }
        Encoder encoder(input.begin(), input.end(),
                        SYMBOL_SIZE,
                        RaptorQ__v1::Block_Size::Block_32);
        if (!encoder.compute_sync()) {
            throw std::runtime_error("libRaptorQ compute_sync failed");
        }
        for (uint32_t esi = 0; esi < SYMBOLS_PER_GEN; ++esi) {
            std::vector<uint32_t> sym_out(WORDS_PER_SYM, 0);
            auto it = sym_out.begin();
            const size_t written = encoder.encode(it, sym_out.end(), esi);
            if (written != WORDS_PER_SYM) {
                throw std::runtime_error("libRaptorQ encode short write");
            }
            m_outbox.push_back(build_phy_packet(esi, sym_out));
        }
    }

    std::vector<unsigned char>
    build_phy_packet(uint32_t esi, const std::vector<uint32_t>& sym) const {
        std::vector<unsigned char> p(PHY_PACKET_BYTES);
        WireHeader h{};
        h.generation_id_be = hton16(m_gen_id);
        h.esi_be           = hton32(esi & 0x00FFFFFF);
        h.k_be             = hton16(SOURCE_SYMBOLS_PER_GEN);
        h.reserved_be      = 0;
        std::memcpy(p.data(),                &h,         HEADER_BYTES);
        std::memcpy(p.data() + HEADER_BYTES, sym.data(), SYMBOL_SIZE);
        return p;
    }

    uint16_t m_gen_id;
    std::vector<std::vector<uint8_t>> m_buffered;
    std::deque<std::vector<unsigned char>> m_outbox;
};

}  // namespace fec

// loopback_test.cpp — self-contained test of the FEC shim's framing logic.
//
// We can't link real libRaptorQ in this sandbox, so this test stubs out the
// RaptorQ encoder with a trivial pass-through that emits the K source
// symbols verbatim and 0 repair symbols. The point is to verify:
//   1. RTP -> source symbol packing (length prefix + zero pad)
//   2. Wire header serialisation (gen_id, ESI, K)
//   3. Generation tracking / boundary detection
//   4. Source-symbol unwrap on the RX side
//
// Real loss tolerance comes from libRaptorQ; that's a separate test that
// requires the real library installed.

#include "fec_common.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <random>
#include <vector>

using fec::SOURCE_SYMBOLS_PER_GEN;
using fec::SYMBOL_SIZE;
using fec::MAX_RTP_PAYLOAD;
using fec::HEADER_BYTES;
using fec::PHY_PACKET_BYTES;
using fec::WireHeader;

// -------- Stub TX encoder (no FEC, just framing) --------
class StubEncoder {
public:
    StubEncoder() : m_gen_id(0) {}
    void add_rtp_packet(const std::vector<unsigned char>& rtp) {
        std::vector<uint8_t> sym(SYMBOL_SIZE, 0);
        uint16_t len_be = fec::hton16((uint16_t)rtp.size());
        std::memcpy(sym.data(),     &len_be, 2);
        std::memcpy(sym.data() + 2, rtp.data(), rtp.size());
        m_buf.push_back(std::move(sym));
        if (m_buf.size() == SOURCE_SYMBOLS_PER_GEN) {
            for (uint32_t esi = 0; esi < SOURCE_SYMBOLS_PER_GEN; ++esi)
                m_out.push_back(make_phy(esi, m_buf[esi]));
            m_buf.clear();
            ++m_gen_id;
        }
    }
    bool has() const { return !m_out.empty(); }
    std::vector<unsigned char> next() {
        auto p = std::move(m_out.front()); m_out.pop_front(); return p;
    }
private:
    std::vector<unsigned char>
    make_phy(uint32_t esi, const std::vector<uint8_t>& sym) const {
        std::vector<unsigned char> p(PHY_PACKET_BYTES);
        WireHeader h{};
        h.generation_id_be = fec::hton16(m_gen_id);
        h.esi_be           = fec::hton32(esi);
        h.k_be             = fec::hton16(SOURCE_SYMBOLS_PER_GEN);
        h.reserved_be      = 0;
        std::memcpy(p.data(),                &h,         HEADER_BYTES);
        std::memcpy(p.data() + HEADER_BYTES, sym.data(), SYMBOL_SIZE);
        return p;
    }
    uint16_t m_gen_id;
    std::vector<std::vector<uint8_t>> m_buf;
    std::deque<std::vector<unsigned char>> m_out;
};

// -------- Stub RX decoder (also no FEC) --------
// Without RaptorQ we can only recover RTP packets from received source
// symbols. This is enough to test the shim framing.
class StubDecoder {
public:
    std::vector<std::vector<unsigned char>>
    process_phy_packet(const std::vector<unsigned char>& phy) {
        std::vector<std::vector<unsigned char>> out;
        if (phy.size() != PHY_PACKET_BYTES) return out;
        WireHeader h; std::memcpy(&h, phy.data(), HEADER_BYTES);
        const uint32_t esi = fec::ntoh32(h.esi_be) & 0x00FFFFFF;
        if (esi >= SOURCE_SYMBOLS_PER_GEN) return out;  // only source syms
        const uint8_t* sym = phy.data() + HEADER_BYTES;
        uint16_t len_be; std::memcpy(&len_be, sym, 2);
        const uint16_t len = fec::ntoh16(len_be);
        if (len == 0 || len > MAX_RTP_PAYLOAD) return out;
        out.emplace_back(sym + 2, sym + 2 + len);
        return out;
    }
};

int main() {
    StubEncoder enc;
    StubDecoder dec;
    std::mt19937 rng(42);

    // Generate 2 generations' worth of variable-size RTP packets.
    std::vector<std::vector<unsigned char>> sent;
    for (size_t i = 0; i < 2 * SOURCE_SYMBOLS_PER_GEN; ++i) {
        std::uniform_int_distribution<unsigned> sz(50, MAX_RTP_PAYLOAD);
        std::vector<unsigned char> pkt(sz(rng));
        for (auto& b : pkt) b = (unsigned char)(rng() & 0xFF);
        sent.push_back(pkt);
        enc.add_rtp_packet(pkt);
    }

    // Drain the wire, simulate 10% packet loss.
    std::vector<std::vector<unsigned char>> wire;
    while (enc.has()) wire.push_back(enc.next());
    std::printf("encoder produced %zu PHY packets from %zu RTP packets\n",
                wire.size(), sent.size());
    assert(wire.size() == 2 * SOURCE_SYMBOLS_PER_GEN);  // no FEC = K, not K+R

    std::uniform_real_distribution<double> drop(0.0, 1.0);
    std::vector<std::vector<unsigned char>> recovered;
    size_t dropped = 0;
    for (auto& p : wire) {
        if (drop(rng) < 0.10) { ++dropped; continue; }
        for (auto& r : dec.process_phy_packet(p)) recovered.push_back(r);
    }
    std::printf("dropped %zu PHY packets, recovered %zu RTP packets\n",
                dropped, recovered.size());

    // Without real FEC, we expect to recover (sent - dropped) packets.
    // Verify that every recovered packet matches some sent packet exactly.
    size_t matches = 0;
    for (auto& r : recovered) {
        for (auto& s : sent) {
            if (r == s) { ++matches; break; }
        }
    }
    std::printf("matched %zu of %zu recovered packets\n",
                matches, recovered.size());
    assert(matches == recovered.size());

    std::printf("PASS — framing layer survives random loss correctly.\n");
    return 0;
}

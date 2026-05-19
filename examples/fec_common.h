// fec_common.h — shared definitions for the FEC shim layer.
//
// The wire format on top of the fun_ofdm PHY is:
//
//   [0..1]  uint16_t generation_id    (big-endian)
//   [2..5]  uint32_t esi              (big-endian, 24-bit ESI in low bits)
//   [6..7]  uint16_t K                (big-endian, source symbols in this gen)
//   [8..9]  uint16_t reserved         (zero)
//   [10..]  uint8_t  symbol[T]        (fixed size T = SYMBOL_SIZE bytes)

#pragma once
#include <cstdint>

namespace fec {

constexpr uint16_t SOURCE_SYMBOLS_PER_GEN = 32;   // K
constexpr uint16_t REPAIR_SYMBOLS_PER_GEN = 8;    // R, gives 25% overhead
constexpr uint16_t SYMBOLS_PER_GEN        = SOURCE_SYMBOLS_PER_GEN
                                          + REPAIR_SYMBOLS_PER_GEN;
constexpr uint16_t SYMBOL_SIZE            = 1200;
constexpr uint16_t MAX_RTP_PAYLOAD        = SYMBOL_SIZE - 2;
constexpr unsigned HEADER_BYTES           = 10;
constexpr unsigned PHY_PACKET_BYTES       = HEADER_BYTES + SYMBOL_SIZE;

#pragma pack(push, 1)
struct WireHeader {
    uint16_t generation_id_be;
    uint32_t esi_be;
    uint16_t k_be;
    uint16_t reserved_be;
};
#pragma pack(pop)
static_assert(sizeof(WireHeader) == HEADER_BYTES, "header packing");

inline uint16_t hton16(uint16_t v) {
    return (uint16_t)((v << 8) | (v >> 8));
}
inline uint16_t ntoh16(uint16_t v) { return hton16(v); }
inline uint32_t hton32(uint32_t v) {
    return ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8)
         | ((v & 0x00FF0000u) >> 8)  | ((v & 0xFF000000u) >> 24);
}
inline uint32_t ntoh32(uint32_t v) { return hton32(v); }

}  // namespace fec

// fec_common.h — shared definitions for the FEC shim.
#pragma once
#include <cstdint>

namespace fec {

    constexpr uint16_t SOURCE_SYMBOLS_PER_GEN = 32;   // K
    constexpr uint16_t REPAIR_SYMBOLS_PER_GEN = 16;    // R (25% overhead)
    constexpr uint16_t SYMBOLS_PER_GEN        = SOURCE_SYMBOLS_PER_GEN
                                              + REPAIR_SYMBOLS_PER_GEN;
    constexpr uint16_t SYMBOL_SIZE            = 1920;  // multiple of 4
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

    inline uint16_t hton16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }
    inline uint16_t ntoh16(uint16_t v) { return hton16(v); }
    inline uint32_t hton32(uint32_t v) {
        return ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8)
             | ((v & 0x00FF0000u) >> 8)  | ((v & 0xFF000000u) >> 24);
    }
    inline uint32_t ntoh32(uint32_t v) { return hton32(v); }

}  // namespace fec
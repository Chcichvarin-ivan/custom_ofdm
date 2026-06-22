/*!
 * \file video_packet.h
 * \brief Shared packet geometry + header layout for the H.265 video link.
 *
 * BOTH video_tx.cpp and video_rx.cpp include this so the wire format can never
 * drift between them. (TX/RX format mismatches were a recurring source of
 * "healthy PHY, no video" bugs — a single shared header prevents that.)
 *
 * Header is 20 bytes, same size as the original JPEG link. The only change
 * from the JPEG format is that a previously-reserved byte now carries a flags
 * field whose bit 0 marks H.265 keyframes. Old position layout is preserved.
 *
 *   offset  size  field
 *   ------  ----  -----------------------------------------------------
 *    0       4    magic        (0xDEADBEEF, network order)
 *    4       4    frame_id     (network order, increments per video frame)
 *    8       2    chunk_index  (network order, 0..total_chunks-1)
 *   10       2    total_chunks (network order, packets in this frame)
 *   12       2    payload_size (network order, bytes of payload in THIS pkt)
 *   14       1    flags        (bit0 = keyframe; other bits reserved 0)
 *   15       5    reserved     (zero)
 *   20            payload begins
 */

#ifndef VIDEO_PACKET_H
#define VIDEO_PACKET_H

#include <cstdint>
#include <cstddef>

namespace vid {

static const std::size_t PACKET_SIZE  = 1900;
static const std::size_t HEADER_SIZE  = 20;
static const std::size_t PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE; // 1880
static const uint32_t    MAGIC        = 0xDEADBEEFu;

// flags byte (offset 14)
static const uint8_t FLAG_KEYFRAME = 0x01;

// Field offsets, so TX and RX read/write the exact same positions.
static const std::size_t OFF_MAGIC       = 0;
static const std::size_t OFF_FRAME_ID    = 4;
static const std::size_t OFF_CHUNK_IDX   = 8;
static const std::size_t OFF_TOTAL       = 10;
static const std::size_t OFF_PAYLOAD_SZ  = 12;
static const std::size_t OFF_FLAGS       = 14;

} // namespace vid

#endif // VIDEO_PACKET_H

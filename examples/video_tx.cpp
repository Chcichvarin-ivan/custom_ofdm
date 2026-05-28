/*!
 * \file video_tx.cpp
 * \brief Webcam video transmitter over fun_ofdm, with optional RaptorQ FEC.
 *
 * Captures webcam frames, JPEG-encodes them, fragments into 1900-byte
 * application packets, optionally wraps each packet in RaptorQ FEC, and
 * transmits via the fun_ofdm transmitter.
 *
 * FEC is compile-time optional. Build with -DWITH_FEC to enable it.
 *
 *   Without FEC: each 1900-byte packet -> tx.send_frame() directly.
 *   With FEC:    packets are buffered into generations of 32, encoded into
 *                40 FEC packets (32 source + 8 repair), each sent as one
 *                PHY frame. The receiver recovers all 32 from any 32 of 40.
 *
 * Packet layout (1900 bytes, network byte order header):
 *    0   4  magic        = 0xDEADBEEF
 *    4   4  frame_id     (uint32)
 *    8   2  chunk_index  (uint16)
 *   10   2  total_chunks (uint16)
 *   12   2  payload_size (uint16)
 *   14   6  reserved
 *   20 1880 payload
 */

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>
#include <arpa/inet.h>

#include <opencv2/opencv.hpp>

#include "transmitter.h"
#include "realtime.h"

#ifdef WITH_FEC
#  include "fec_common.h"
#  include "fec_encoder.h"
#endif

using namespace fun;

// ---------------------- Radio parameters ------------------------------------
static const double FREQ        = 3.4e9;    // center freq [Hz] — matches RX
static const double SAMPLE_RATE = 20e6;     // sample rate [Hz]
static const double TX_GAIN     = 60.0;     // B200 mini scale
static const double TX_AMP      = 0.5;      // digital backoff
static const Rate   PHY_RATE    = RATE_1_2_QPSK;

// ---------------------- Packet geometry -------------------------------------
static const std::size_t PACKET_SIZE  = 1900;
static const std::size_t HEADER_SIZE  = 20;
static const std::size_t PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE;  // 1880
static const uint32_t    MAGIC        = 0xDEADBEEFu;

// ---------------------- Camera / encoding -----------------------------------
static const int CAM_INDEX  = 0;
static const int FRAME_W    = 640;
static const int FRAME_H    = 480;
static const int JPEG_Q     = 50;
static const int TARGET_FPS = 30;

#ifdef WITH_FEC
// Sanity check: FEC symbol must be large enough for our packet.
static_assert(fec::SYMBOL_SIZE >= PACKET_SIZE,
              "fec::SYMBOL_SIZE must be >= PACKET_SIZE (set it to 1920)");
static fec::FecEncoder g_enc;
#endif

// ----------------------------------------------------------------------------
static void build_packet(std::vector<unsigned char>& pkt,
                         uint32_t frame_id,
                         uint16_t chunk_index,
                         uint16_t total_chunks,
                         const unsigned char* payload,
                         uint16_t payload_size)
{
    pkt.assign(PACKET_SIZE, 0);
    uint32_t magic_n      = htonl(MAGIC);
    uint32_t frame_id_n   = htonl(frame_id);
    uint16_t chunk_idx_n  = htons(chunk_index);
    uint16_t total_n      = htons(total_chunks);
    uint16_t payload_sz_n = htons(payload_size);
    std::memcpy(&pkt[0],  &magic_n,      4);
    std::memcpy(&pkt[4],  &frame_id_n,   4);
    std::memcpy(&pkt[8],  &chunk_idx_n,  2);
    std::memcpy(&pkt[10], &total_n,      2);
    std::memcpy(&pkt[12], &payload_sz_n, 2);
    if (payload_size > 0)
        std::memcpy(&pkt[HEADER_SIZE], payload, payload_size);
}

// Sends one application packet, through FEC if enabled, else directly.
static void transmit_packet(transmitter& tx,
                            const std::vector<unsigned char>& packet)
{
#ifdef WITH_FEC
    g_enc.add_rtp_packet(packet);
    while (g_enc.has_phy_packet()) {
        std::vector<unsigned char> phy = g_enc.next_phy_packet();
        tx.send_frame(phy, PHY_RATE);
    }
#else
    tx.send_frame(packet, PHY_RATE);
#endif
}

int main(int /*argc*/, char** /*argv*/)
{
    std::cout << "fun_ofdm video transmitter"
#ifdef WITH_FEC
              << " [FEC ENABLED]"
#else
              << " [no FEC]"
#endif
              << "\n";

    set_realtime_priority(1.0f);

    cv::VideoCapture cap(CAM_INDEX);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: cannot open webcam index " << CAM_INDEX << "\n";
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  FRAME_W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_H);
    cap.set(cv::CAP_PROP_FPS,          TARGET_FPS);

    transmitter tx(FREQ, SAMPLE_RATE, TX_GAIN, TX_AMP,
                   "type=b200,serial=3123B0D");

    std::vector<int> jpeg_params = { cv::IMWRITE_JPEG_QUALITY, JPEG_Q };
    std::vector<unsigned char> packet;
    std::vector<unsigned char> jpeg_buf;

    uint32_t frame_id = 0;
    const auto frame_period = std::chrono::milliseconds(1000 / TARGET_FPS);

    while (true) {
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "WARN: dropped empty frame\n";
            continue;
        }

        jpeg_buf.clear();
        if (!cv::imencode(".jpg", frame, jpeg_buf, jpeg_params)) {
            std::cerr << "WARN: JPEG encode failed\n";
            continue;
        }

        std::size_t nbytes = jpeg_buf.size();
        std::size_t total_chunks =
            (nbytes + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE;
        if (total_chunks == 0) total_chunks = 1;
        if (total_chunks > 0xFFFFu) {
            std::cerr << "WARN: frame too large (" << nbytes << " B)\n";
            continue;
        }

        std::cout << "Frame " << frame_id << "  " << nbytes
                  << " B  " << total_chunks << " pkts\n";

        for (std::size_t i = 0; i < total_chunks; ++i) {
            std::size_t offset    = i * PAYLOAD_SIZE;
            std::size_t remaining = nbytes - offset;
            uint16_t this_payload =
                static_cast<uint16_t>(std::min(remaining, PAYLOAD_SIZE));

            build_packet(packet, frame_id,
                         static_cast<uint16_t>(i),
                         static_cast<uint16_t>(total_chunks),
                         jpeg_buf.data() + offset, this_payload);

            if (packet.size() != PACKET_SIZE) {
                std::cerr << "BUG: packet size " << packet.size() << "\n";
                return 2;
            }
            transmit_packet(tx, packet);
        }

        ++frame_id;

#ifdef WITH_FEC
        // Flush any partial generation at end of frame so the receiver
        // doesn't wait indefinitely for a generation to fill.
        g_enc.flush();
        while (g_enc.has_phy_packet())
            tx.send_frame(g_enc.next_phy_packet(), PHY_RATE);
#endif

        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < frame_period)
            std::this_thread::sleep_for(frame_period - elapsed);
    }
    return 0;
}
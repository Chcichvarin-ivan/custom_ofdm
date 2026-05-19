/*!
 * \file video_tx.cpp
 * \brief Webcam video transmitter over fun_ofdm.
 *
 * Captures frames from a webcam, JPEG-encodes each frame, fragments the
 * encoded bytes into STRICTLY 1900-byte packets, and transmits them using
 * the fun_ofdm transmitter class.
 *
 * Packet layout (1900 bytes total, network byte order for header fields):
 *
 *   offset  size  field
 *   ------  ----  -----
 *        0     4  magic        = 0xDEADBEEF
 *        4     4  frame_id     (uint32) monotonically increasing
 *        8     2  chunk_index  (uint16) 0-based index of this chunk
 *       10     2  total_chunks (uint16) total chunks in this frame
 *       12     2  payload_size (uint16) actual payload bytes in this chunk
 *       14     6  reserved (zeroed)
 *       20  1880  payload (zero-padded on the last chunk)
 *
 * Build example (after `sudo make install` / `sudo ldconfig` of fun_ofdm):
 *
 *   g++ -std=c++11 -O2 video_tx.cpp -o video_tx \
 *       -lfun_ofdm -luhd -lfftw3 -lboost_system -lpthread \
 *       `pkg-config --cflags --libs opencv4`
 *
 * Run (needs root for the USRP):
 *
 *   sudo ./video_tx
 */

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>
#include <arpa/inet.h>          // htonl, htons

#include <opencv2/opencv.hpp>

#include "transmitter.h"        // fun_ofdm
#include "realtime.h"           // set_realtime_priority()
using namespace fun;

// ---------------------- Radio parameters ------------------------------------
static const double FREQ        = 3.4e9;              // center freq [Hz]
static const double SAMPLE_RATE = 20e6;                 // 16 MHz
static const double TX_GAIN     = 85.0;
static const double TX_AMP      = 0.7;
static const Rate   PHY_RATE    =  RATE_1_2_QPSK;//RATE_1_2_BPSK;//RATE_3_4_QAM16;//RATE_2_3_BPSK;//      // 36 Mbps, good for video

// ---------------------- Packet geometry -------------------------------------
static const std::size_t PACKET_SIZE  = 1900;          // strict
static const std::size_t HEADER_SIZE  = 20;
static const std::size_t PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE;  // 1880
static const uint32_t    MAGIC        = 0xDEADBEEFu;

// ---------------------- Camera / encoding -----------------------------------
static const int         CAM_INDEX  = 0;
static const int         FRAME_W    = 640;             // small frames keep
static const int         FRAME_H    = 480;             // JPEGs small enough
static const int         JPEG_Q     = 50;              // quality 0..100
static const int         TARGET_FPS = 30;

// ----------------------------------------------------------------------------
// Writes one 1900-byte packet into `pkt` from the given chunk info.
static void build_packet(std::vector<unsigned char>& pkt,
                         uint32_t frame_id,
                         uint16_t chunk_index,
                         uint16_t total_chunks,
                         const unsigned char* payload,
                         uint16_t payload_size)
{
    pkt.assign(PACKET_SIZE, 0);  // zero-pad the whole packet

    uint32_t magic_n       = htonl(MAGIC);
    uint32_t frame_id_n    = htonl(frame_id);
    uint16_t chunk_idx_n   = htons(chunk_index);
    uint16_t total_n       = htons(total_chunks);
    uint16_t payload_sz_n  = htons(payload_size);

    std::memcpy(&pkt[0],  &magic_n,      4);
    std::memcpy(&pkt[4],  &frame_id_n,   4);
    std::memcpy(&pkt[8],  &chunk_idx_n,  2);
    std::memcpy(&pkt[10], &total_n,      2);
    std::memcpy(&pkt[12], &payload_sz_n, 2);
    // bytes 14..19 reserved, already zero

    if (payload_size > 0)
        std::memcpy(&pkt[HEADER_SIZE], payload, payload_size);
    // remaining bytes [HEADER_SIZE + payload_size .. PACKET_SIZE) stay zero
}

int main(int /*argc*/, char** /*argv*/)
{
    std::cout << "fun_ofdm video transmitter -- strict 1900-byte packets\n";
    // Boost this thread so JPEG-encode + frame-build + send_burst aren't
    // preempted mid-burst (USRP underruns cause dropped frames at the RX).
    set_realtime_priority(1.0f);
    // ------------------------------ camera ----------------------------------
    cv::VideoCapture cap(CAM_INDEX);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: cannot open webcam index " << CAM_INDEX << "\n";
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  FRAME_W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_H);
    cap.set(cv::CAP_PROP_FPS,          TARGET_FPS);

    // ------------------------------ radio -----------------------------------
    transmitter tx(FREQ, SAMPLE_RATE, TX_GAIN, TX_AMP, "type=b200,serial=3123B0D");

    std::vector<int> jpeg_params = { cv::IMWRITE_JPEG_QUALITY, JPEG_Q };
    std::vector<unsigned char> packet;     // reusable 1900-byte buffer
    std::vector<unsigned char> jpeg_buf;   // encoded frame

    uint32_t frame_id = 0;
    const auto frame_period = std::chrono::milliseconds(1000 / TARGET_FPS);

    while (true) {
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "WARN: dropped empty frame\n";
            continue;
        }

        // Encode to JPEG
        jpeg_buf.clear();
        if (!cv::imencode(".jpg", frame, jpeg_buf, jpeg_params)) {
            std::cerr << "WARN: JPEG encode failed\n";
            continue;
        }

        // Fragment. `total_chunks` must fit in uint16_t; with 1880-byte payload
        // that supports frames up to ~123 MB -- plenty for JPEG.
        std::size_t nbytes = jpeg_buf.size();
        std::size_t total_chunks =
            (nbytes + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE;
        if (total_chunks == 0) total_chunks = 1;    // empty-frame edge case
        if (total_chunks > 0xFFFFu) {
            std::cerr << "WARN: frame too large (" << nbytes
                      << " B), skipping\n";
            continue;
        }

        std::cout << "Frame " << frame_id
                  << "  " << nbytes << " B  "
                  << total_chunks << " pkts" << std::endl;

        for (std::size_t i = 0; i < total_chunks; ++i) {
            std::size_t offset = i * PAYLOAD_SIZE;
            std::size_t remaining = nbytes - offset;
            uint16_t this_payload =
                static_cast<uint16_t>(std::min(remaining, PAYLOAD_SIZE));

            build_packet(packet,
                         frame_id,
                         static_cast<uint16_t>(i),
                         static_cast<uint16_t>(total_chunks),
                         jpeg_buf.data() + offset,
                         this_payload);

            // Safety: enforce the strict size contract.
            if (packet.size() != PACKET_SIZE) {
                std::cerr << "BUG: packet size " << packet.size()
                          << " != " << PACKET_SIZE << "\n";
                return 2;
            }
            tx.send_frame(packet, PHY_RATE);
        }

        ++frame_id;

        // Simple rate pacing; skip if we're already behind.
        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < frame_period)
            std::this_thread::sleep_for(frame_period - elapsed);
    }

    return 0;
}

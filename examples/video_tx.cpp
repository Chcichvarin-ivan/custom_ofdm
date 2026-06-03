/*!
 * \file video_tx.cpp
 * \brief Webcam video TX over fun_ofdm, with optional FEC and TUI.
 *
 *   -DWITH_FEC enables RaptorQ FEC (generation 32 src + 8 repair).
 *   -DWITH_TUI enables the terminal-based link-health display.
 *
 * Build both together, individually, or neither. The non-FEC, non-TUI build
 * is identical in behavior to the original video_tx.
 */

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <arpa/inet.h>

#include <opencv2/opencv.hpp>

#include "transmitter.h"
#include "realtime.h"

#ifdef WITH_FEC
#  include "fec_common.h"
#  include "fec_encoder.h"
#endif

#ifdef WITH_TUI
#  include "link_stats.h"
#  include "link_tui.h"
#endif

using namespace fun;

// ---------------------- Radio parameters ------------------------------------
static const double FREQ        = 3.4e9;
static const double SAMPLE_RATE = 20e6;
static const double TX_GAIN     = 60.0;
static const double TX_AMP      = 0.5;
static const Rate   PHY_RATE    = RATE_1_2_QPSK;

// ---------------------- Packet geometry -------------------------------------
static const std::size_t PACKET_SIZE  = 1900;
static const std::size_t HEADER_SIZE  = 20;
static const std::size_t PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE;
static const uint32_t    MAGIC        = 0xDEADBEEFu;

// ---------------------- Camera / encoding -----------------------------------
static const int CAM_INDEX  = 0;
static const int FRAME_W    = 640;
static const int FRAME_H    = 480;
static const int JPEG_Q     = 50;
static const int TARGET_FPS = 30;

#ifdef WITH_FEC
static_assert(fec::SYMBOL_SIZE >= PACKET_SIZE,
              "fec::SYMBOL_SIZE must be >= PACKET_SIZE (set to 1920)");
static fec::FecEncoder g_enc;
#endif

#ifdef WITH_TUI
static stats::LinkStats g_stats;
static std::atomic<bool> g_stop{false};
static stats::LinkTui*   g_tui_ptr = nullptr;
static void on_signal(int) {
    g_stop.store(true);
    if (g_tui_ptr) g_tui_ptr->stop();
}
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

static void transmit_packet(transmitter& tx,
                            const std::vector<unsigned char>& packet)
{
#ifdef WITH_FEC
    g_enc.add_rtp_packet(packet);
    while (g_enc.has_phy_packet()) {
        std::vector<unsigned char> phy = g_enc.next_phy_packet();
        tx.send_frame(phy, PHY_RATE);
#  ifdef WITH_TUI
        g_stats.note_phy_packet_tx();
#  endif
    }
#else
    tx.send_frame(packet, PHY_RATE);
#  ifdef WITH_TUI
    g_stats.note_phy_packet_tx();
#  endif
#endif
}

int main(int /*argc*/, char** /*argv*/)
{
    std::cout << "fun_ofdm video TX"
#ifdef WITH_FEC
              << " [FEC]"
#endif
#ifdef WITH_TUI
              << " [TUI]"
#endif
              << "\n";

    set_realtime_priority(1.0f);

#ifdef WITH_TUI
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);
#  ifdef WITH_FEC
    g_enc.set_stats(&g_stats);
#  endif
    stats::LinkTui tui(g_stats, stats::TuiMode::TX);
    g_tui_ptr = &tui;
    std::thread tui_thread([&tui]() { tui.run(); });
#endif

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

#ifdef WITH_TUI
    while (!g_stop.load()) {
#else
    while (true) {
#endif
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "WARN: empty frame\n";
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
        if (total_chunks > 0xFFFFu) continue;

#ifdef WITH_TUI
        g_stats.note_video_frame_encoded(nbytes, total_chunks);
#else
        std::cout << "Frame " << frame_id << "  " << nbytes
                  << " B  " << total_chunks << " pkts\n";
#endif

        for (std::size_t i = 0; i < total_chunks; ++i) {
            std::size_t offset    = i * PAYLOAD_SIZE;
            std::size_t remaining = nbytes - offset;
            uint16_t this_payload =
                static_cast<uint16_t>(std::min(remaining, PAYLOAD_SIZE));
            build_packet(packet, frame_id,
                         static_cast<uint16_t>(i),
                         static_cast<uint16_t>(total_chunks),
                         jpeg_buf.data() + offset, this_payload);
            transmit_packet(tx, packet);
        }
        ++frame_id;

#ifdef WITH_FEC
        // Do NOT flush every frame — that pads every generation and triples
        // airtime. Only flush if too much time passed since last send, so a
        // stalled stream still drains. Otherwise let generations fill across
        // frames at their natural 32-packet boundary.
        static auto last_flush = std::chrono::steady_clock::now();
        auto now_f = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                now_f - last_flush).count() > 200) {
            g_enc.flush();
            last_flush = now_f;
                }
        while (g_enc.has_phy_packet()) {
            tx.send_frame(g_enc.next_phy_packet(), PHY_RATE);
#  ifdef WITH_TUI
            g_stats.note_phy_packet_tx();
#  endif
        }
#endif

        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < frame_period)
            std::this_thread::sleep_for(frame_period - elapsed);
    }

#ifdef WITH_TUI
    tui.stop();
    tui_thread.join();
#endif
    return 0;
}
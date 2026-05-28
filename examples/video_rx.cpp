/*!
 * \file video_rx.cpp
 * \brief Webcam video receiver over fun_ofdm, with optional RaptorQ FEC.
 *
 * Receives PHY frames via the fun_ofdm receiver callback. If FEC is enabled,
 * each PHY frame is fed to the RaptorQ decoder, which emits recovered
 * 1900-byte application packets once a generation is decodable. Those
 * packets then go through the same JPEG reassembly path as the non-FEC build.
 *
 * Build with -DWITH_FEC to enable FEC (must match the TX build).
 */

#include <iostream>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <atomic>
#include <arpa/inet.h>

#include <opencv2/opencv.hpp>

#include "receiver.h"
#include "realtime.h"

#ifdef WITH_FEC
#  include "fec_common.h"
#  include "fec_decoder.h"
#endif

using namespace fun;

// ---------------------- Radio parameters (must match TX) --------------------
static const double FREQ        = 3.4e9;    // matches TX
static const double SAMPLE_RATE = 20e6;
static const double RX_GAIN     = 20.0;     // with LNA; sweep 15-30

// ---------------------- Packet geometry (must match TX) --------------------
static const std::size_t PACKET_SIZE  = 1900;
static const std::size_t HEADER_SIZE  = 20;
static const std::size_t PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE;  // 1880
static const uint32_t    MAGIC        = 0xDEADBEEFu;

#ifdef WITH_FEC
static_assert(fec::SYMBOL_SIZE >= PACKET_SIZE,
              "fec::SYMBOL_SIZE must be >= PACKET_SIZE (set it to 1920)");
static fec::FecDecoder g_dec;
#endif

// ---------------------------------------------------------------------------
struct PartialFrame {
    uint16_t total_chunks = 0;
    std::vector<std::vector<unsigned char>> chunks;
    std::vector<bool> received;
    std::size_t chunks_received = 0;
    std::chrono::steady_clock::time_point first_seen;
};

static std::mutex                                g_queue_mu;
static std::condition_variable                   g_queue_cv;
static std::queue<std::vector<unsigned char>>    g_jpeg_queue;
static std::atomic<bool>                         g_stop{false};

static std::map<uint32_t, PartialFrame> g_partials;
static uint32_t g_last_displayed_frame_id = 0;
static bool     g_have_displayed = false;

// ---------------------------------------------------------------------------
static void flush_frame(uint32_t frame_id, PartialFrame& pf)
{
    std::vector<unsigned char> jpeg;
    jpeg.reserve(pf.total_chunks * PAYLOAD_SIZE);
    for (uint16_t i = 0; i < pf.total_chunks; ++i)
        jpeg.insert(jpeg.end(), pf.chunks[i].begin(), pf.chunks[i].end());

    {
        std::lock_guard<std::mutex> lk(g_queue_mu);
        while (g_jpeg_queue.size() > 4) g_jpeg_queue.pop();
        g_jpeg_queue.push(std::move(jpeg));
    }
    g_queue_cv.notify_one();
    g_last_displayed_frame_id = frame_id;
    g_have_displayed = true;
}

// ---------------------------------------------------------------------------
// Process ONE recovered application packet (1900 bytes) into the reassembly
// table. This is the same logic for FEC and non-FEC paths.
static void process_app_packet(const std::vector<unsigned char>& pkt)
{
    if (pkt.size() != PACKET_SIZE) return;

    uint32_t magic_n, frame_id_n;
    uint16_t chunk_idx_n, total_n, payload_sz_n;
    std::memcpy(&magic_n,      &pkt[0],  4);
    std::memcpy(&frame_id_n,   &pkt[4],  4);
    std::memcpy(&chunk_idx_n,  &pkt[8],  2);
    std::memcpy(&total_n,      &pkt[10], 2);
    std::memcpy(&payload_sz_n, &pkt[12], 2);

    if (ntohl(magic_n) != MAGIC) return;

    uint32_t frame_id     = ntohl(frame_id_n);
    uint16_t chunk_index  = ntohs(chunk_idx_n);
    uint16_t total_chunks = ntohs(total_n);
    uint16_t payload_size = ntohs(payload_sz_n);

    if (total_chunks == 0 || chunk_index >= total_chunks) return;
    if (payload_size > PAYLOAD_SIZE) return;
    if (g_have_displayed && frame_id <= g_last_displayed_frame_id) return;

    PartialFrame& pf = g_partials[frame_id];
    if (pf.total_chunks == 0) {
        pf.total_chunks = total_chunks;
        pf.chunks.resize(total_chunks);
        pf.received.assign(total_chunks, false);
        pf.first_seen = std::chrono::steady_clock::now();
    }
    if (pf.total_chunks != total_chunks) {
        pf.total_chunks = total_chunks;
        pf.chunks.assign(total_chunks, {});
        pf.received.assign(total_chunks, false);
        pf.chunks_received = 0;
        pf.first_seen = std::chrono::steady_clock::now();
    }
    if (!pf.received[chunk_index]) {
        pf.chunks[chunk_index].assign(
            pkt.begin() + HEADER_SIZE,
            pkt.begin() + HEADER_SIZE + payload_size);
        pf.received[chunk_index] = true;
        ++pf.chunks_received;
    }
    if (pf.chunks_received == pf.total_chunks) {
        flush_frame(frame_id, pf);
        g_partials.erase(frame_id);
    }
}

// ---------------------------------------------------------------------------
static void rx_callback(std::vector<std::vector<unsigned char>> packets)
{
    for (auto& pkt : packets) {
#ifdef WITH_FEC
        // Each PHY frame is a FEC packet. Feed it to the decoder; when a
        // generation completes, we get back the original application packets.
        std::vector<std::vector<unsigned char>> recovered =
            g_dec.process_phy_packet(pkt);
        for (auto& app : recovered)
            process_app_packet(app);
#else
        process_app_packet(pkt);
#endif
    }

    // Garbage-collect stale partials.
    auto now = std::chrono::steady_clock::now();
    for (auto it = g_partials.begin(); it != g_partials.end(); ) {
        bool too_old = (now - it->second.first_seen) > std::chrono::seconds(1);
        bool superseded = g_have_displayed &&
                          (it->first + 30 < g_last_displayed_frame_id);
        if (too_old || superseded) it = g_partials.erase(it);
        else ++it;
    }
}

// ---------------------------------------------------------------------------
int main(int /*argc*/, char** /*argv*/)
{
    std::cout << "fun_ofdm video receiver"
#ifdef WITH_FEC
              << " [FEC ENABLED]"
#else
              << " [no FEC]"
#endif
              << "\n";

    set_realtime_priority(1.0f);

    receiver rx(&rx_callback, FREQ, SAMPLE_RATE, RX_GAIN,
                "type=b200,serial=314C000,num_recv_frames=700,"
                "num_send_frames=700,recv_frame_size=11000,"
                "send_frame_size=11000");

    cv::namedWindow("fun_ofdm video", cv::WINDOW_AUTOSIZE);
    std::cout << "Press 'q' in the video window to quit.\n";

    while (!g_stop) {
        std::vector<unsigned char> jpeg;
        {
            std::unique_lock<std::mutex> lk(g_queue_mu);
            g_queue_cv.wait_for(lk, std::chrono::milliseconds(100),
                                [] { return !g_jpeg_queue.empty() ||
                                            g_stop.load(); });
            if (!g_jpeg_queue.empty()) {
                jpeg = std::move(g_jpeg_queue.front());
                g_jpeg_queue.pop();
            }
        }

        if (!jpeg.empty()) {
            cv::Mat frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);
            if (!frame.empty())
                cv::imshow("fun_ofdm video", frame);
            else
                std::cerr << "WARN: JPEG decode failed ("
                          << jpeg.size() << " B)\n";
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) g_stop = true;
    }

    rx.pause();
    cv::destroyAllWindows();
    return 0;
}
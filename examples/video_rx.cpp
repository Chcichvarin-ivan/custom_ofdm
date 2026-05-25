/*!
 * \file video_rx.cpp
 * \brief Webcam video receiver over fun_ofdm.
 *
 * Registers a callback with the fun_ofdm receiver. Each received packet is
 * expected to be STRICTLY 1900 bytes and follow the layout documented in
 * video_tx.cpp. This program reassembles JPEG frames across packets and
 * displays them with OpenCV.
 *
 * A frame is flushed and displayed as soon as all its chunks are present,
 * OR when we start seeing chunks from a newer frame (partial-frame cleanup).
 *
 * Build example:
 *
 *   g++ -std=c++11 -O2 video_rx.cpp -o video_rx \
 *       -lfun_ofdm -luhd -lfftw3 -lboost_system -lpthread \
 *       `pkg-config --cflags --libs opencv4`
 *
 * Run (needs root for the USRP):
 *
 *   sudo ./video_rx
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
#include <arpa/inet.h>          // ntohl, ntohs

#include <opencv2/opencv.hpp>

#include "receiver.h"           // fun_ofdm
#include "realtime.h"           // set_realtime_priority()
using namespace fun;

// ---------------------- Radio parameters (must match TX) --------------------
const double FREQ        = 3.4e9;
static const double SAMPLE_RATE = 20e6;
static const double RX_GAIN     = 25.0;

// ---------------------- Packet geometry (must match TX) --------------------
static const std::size_t PACKET_SIZE  = 1900;
static const std::size_t HEADER_SIZE  = 20;
static const std::size_t PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE;  // 1880
static const uint32_t    MAGIC        = 0xDEADBEEFu;

// ---------------------------------------------------------------------------
// Partial frame reassembly state for a single frame_id.
struct PartialFrame {
    uint16_t total_chunks = 0;
    std::vector<std::vector<unsigned char>> chunks;   // one buffer per chunk
    std::vector<bool> received;
    std::size_t chunks_received = 0;
    std::chrono::steady_clock::time_point first_seen;
};

// ---------------------------------------------------------------------------
// Thread-safe handoff of completed JPEG buffers from the receiver callback
// thread to the display thread (OpenCV GUI must be driven from main thread).
static std::mutex                                g_queue_mu;
static std::condition_variable                   g_queue_cv;
static std::queue<std::vector<unsigned char>>    g_jpeg_queue;
static std::atomic<bool>                         g_stop{false};

// Reassembly table, keyed by frame_id. Accessed only from the RX callback
// thread, so no locking needed here.
static std::map<uint32_t, PartialFrame> g_partials;
static uint32_t g_last_displayed_frame_id = 0;
static bool     g_have_displayed = false;

// ---------------------------------------------------------------------------
// Flush a completed frame: concatenate chunks and push JPEG bytes to the
// display queue.
static void flush_frame(uint32_t frame_id, PartialFrame& pf)
{
    std::vector<unsigned char> jpeg;
    jpeg.reserve(pf.total_chunks * PAYLOAD_SIZE);
    for (uint16_t i = 0; i < pf.total_chunks; ++i)
        jpeg.insert(jpeg.end(), pf.chunks[i].begin(), pf.chunks[i].end());

    {
        std::lock_guard<std::mutex> lk(g_queue_mu);
        // Drop old frames if the UI is falling behind -- video should be live.
        while (g_jpeg_queue.size() > 4) g_jpeg_queue.pop();
        g_jpeg_queue.push(std::move(jpeg));
    }
    g_queue_cv.notify_one();

    g_last_displayed_frame_id = frame_id;
    g_have_displayed = true;
}

// ---------------------------------------------------------------------------
// fun_ofdm receiver callback: invoked (on a worker thread) with a batch of
// successfully decoded packets whenever any are available.
static void rx_callback(std::vector<std::vector<unsigned char>> packets)
{
    for (auto& pkt : packets) {
        if (pkt.size() != PACKET_SIZE) {
            // Not one of ours (or corrupted length) -- ignore.
            continue;
        }

        // Parse header.
        uint32_t magic_n, frame_id_n;
        uint16_t chunk_idx_n, total_n, payload_sz_n;
        std::memcpy(&magic_n,      &pkt[0],  4);
        std::memcpy(&frame_id_n,   &pkt[4],  4);
        std::memcpy(&chunk_idx_n,  &pkt[8],  2);
        std::memcpy(&total_n,      &pkt[10], 2);
        std::memcpy(&payload_sz_n, &pkt[12], 2);

        if (ntohl(magic_n) != MAGIC)
            continue;   // not one of our packets

        uint32_t frame_id     = ntohl(frame_id_n);
        uint16_t chunk_index  = ntohs(chunk_idx_n);
        uint16_t total_chunks = ntohs(total_n);
        uint16_t payload_size = ntohs(payload_sz_n);

        if (total_chunks == 0 || chunk_index >= total_chunks)
            continue;   // malformed
        if (payload_size > PAYLOAD_SIZE)
            continue;   // malformed

        // Drop chunks for frames we've already displayed (late arrivals).
        if (g_have_displayed && frame_id <= g_last_displayed_frame_id)
            continue;

        PartialFrame& pf = g_partials[frame_id];
        if (pf.total_chunks == 0) {
            pf.total_chunks = total_chunks;
            pf.chunks.resize(total_chunks);
            pf.received.assign(total_chunks, false);
            pf.first_seen = std::chrono::steady_clock::now();
        }
        if (pf.total_chunks != total_chunks) {
            // Conflicting metadata -- reset this frame's state.
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

    // Garbage-collect stale partial frames.
    auto now = std::chrono::steady_clock::now();
    for (auto it = g_partials.begin(); it != g_partials.end(); ) {
        // Drop partials older than 1 second, or any partial whose id is
        // older than (newest - 30) to bound memory.
        bool too_old = (now - it->second.first_seen) > std::chrono::seconds(1);
        bool superseded = g_have_displayed &&
                          (it->first + 30 < g_last_displayed_frame_id);
        if (too_old || superseded)
            it = g_partials.erase(it);
        else
            ++it;
    }
}

// ---------------------------------------------------------------------------
int main(int /*argc*/, char** /*argv*/)
{
    std::cout << "fun_ofdm video receiver -- strict 1900-byte packets\n";
    // The RX worker thread is spawned inside the receiver ctor and its
    // priority is managed by UHD/fun_ofdm itself. We boost the main thread
    // so the JPEG-decode + imshow path keeps up and so that when packets
    // are handed off to us via the callback they get processed promptly.
    set_realtime_priority(1.0f);

    receiver rx(&rx_callback, FREQ, SAMPLE_RATE, RX_GAIN, "type=b200,serial=314C000,num_recv_frames=700,num_send_frames=700,recv_frame_size=11000,send_frame_size=11000");

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
        if (key == 'q' || key == 27)   // q or ESC
            g_stop = true;
    }

    rx.pause();
    cv::destroyAllWindows();
    return 0;
}

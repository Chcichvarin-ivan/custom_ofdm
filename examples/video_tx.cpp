/*!
 * \file video_tx.cpp
 * \brief H.265 video TX over fun_ofdm — runs on Radxa (hardware encode) OR
 *        x86 (software encode), auto-detecting which encoder is available.
 *
 *   -DWITH_FEC enables RaptorQ FEC (must match the RX build). STRONGLY
 *              recommended for H.265 — see note below.
 *   -DWITH_TUI enables the terminal link-health display.
 *
 * Captures from the camera, encodes to H.265, packetizes each compressed frame
 * over fun_ofdm. Encoder is chosen at runtime:
 *   - mpph265enc (RK3588 hardware VPU) if present — the Radxa.
 *   - x265enc (software, ultrafast) otherwise — x86 or any board without the
 *     Rockchip encoder. Software encode uses real CPU; fine for 720p on a
 *     decent x86 box, but it's not free like the VPU.
 * A keyframe flag is set in the packet header so the RX can resync after loss.
 *
 * WHY FEC MATTERS HERE: H.265 delta frames reference earlier frames, so a
 * single lost packet corrupts video until the next keyframe. Testing showed
 * that at ~1.5% packet loss WITHOUT FEC, ~80% of frames are lost. Build with
 * -DWITH_FEC so the FEC recovers lost packets before they break the stream.
 *
 * 720p @ 4 Mbit/s, keyframe every 15 frames (~0.5 s).
 *
 * RK3588 fd-LEAK NOTE: the hardware pipeline forces a colour-convert copy
 * (NV12->I420->NV12) before mpph265enc. This is REQUIRED, not cosmetic — see
 * the long comment at the pipeline string. Removing it (or making it
 * passthrough) brings back the "dst has not fd" RGA crash after ~30 s.
 */

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <csignal>
#include <arpa/inet.h>
#include <sys/resource.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include "transmitter.h"
#include "realtime.h"
#include "video_packet.h"

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
static const double TX_GAIN     = 75.0;
static const double TX_AMP      = 0.7;
static const Rate   PHY_RATE    = RATE_1_2_QPSK;

// ---------------------- Video / encoder -------------------------------------
// 720p @ 4 Mbit/s leaves headroom under a ~5 Mbit/s link. Keyframe every 15
// frames so loss recovers within ~0.5 s. Adjust bps / gop here.
//   gop on mpph265enc: property name may be "gop" on your build. Verify with
//   `gst-inspect-1.0 mpph265enc | grep -iE "gop|key"`. If it's a different
//   name, change KEY_PROP below.
static const char* CAM_DEVICE = "/dev/video11";
static const int   FRAME_W    = 1280;
static const int   FRAME_H    = 720;
static const int   TARGET_FPS = 15;
static const int   BPS        = 2200000;   // 1.2 Mbit/s
static const int   GOP        = 5;        // keyframe interval (frames)

#ifdef WITH_FEC
static_assert(fec::SYMBOL_SIZE >= vid::PACKET_SIZE,
              "fec::SYMBOL_SIZE must be >= PACKET_SIZE (set to 1920)");
static fec::FecEncoder g_enc;
#endif
#ifdef WITH_TUI
static stats::LinkStats g_stats;
static stats::LinkTui*  g_tui_ptr = nullptr;
#endif
static std::atomic<bool> g_stop{false};

static void on_signal(int) {
    g_stop.store(true);
#ifdef WITH_TUI
    if (g_tui_ptr) g_tui_ptr->stop();
#endif
}

// ----------------------------------------------------------------------------
static void build_packet(std::vector<unsigned char>& pkt,
                         uint32_t frame_id, uint16_t chunk_index,
                         uint16_t total_chunks, uint8_t flags,
                         const unsigned char* payload, uint16_t payload_size)
{
    pkt.assign(vid::PACKET_SIZE, 0);
    uint32_t magic_n   = htonl(vid::MAGIC);
    uint32_t fid_n     = htonl(frame_id);
    uint16_t chunk_n   = htons(chunk_index);
    uint16_t total_n   = htons(total_chunks);
    uint16_t paysz_n   = htons(payload_size);
    std::memcpy(&pkt[vid::OFF_MAGIC],      &magic_n, 4);
    std::memcpy(&pkt[vid::OFF_FRAME_ID],   &fid_n,   4);
    std::memcpy(&pkt[vid::OFF_CHUNK_IDX],  &chunk_n, 2);
    std::memcpy(&pkt[vid::OFF_TOTAL],      &total_n, 2);
    std::memcpy(&pkt[vid::OFF_PAYLOAD_SZ], &paysz_n, 2);
    pkt[vid::OFF_FLAGS] = flags;
    if (payload_size > 0)
        std::memcpy(&pkt[vid::HEADER_SIZE], payload, payload_size);
}

static void transmit_packet(transmitter& tx,
                            const std::vector<unsigned char>& packet)
{
#ifdef WITH_FEC
    g_enc.add_rtp_packet(packet);
    while (g_enc.has_phy_packet()) {
        tx.send_frame(g_enc.next_phy_packet(), PHY_RATE);
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

// Packetize one compressed H.265 frame and send it.
static void send_h265_frame(transmitter& tx, const unsigned char* data,
                            std::size_t nbytes, bool keyframe,
                            uint32_t frame_id)
{
    std::size_t total_chunks =
        (nbytes + vid::PAYLOAD_SIZE - 1) / vid::PAYLOAD_SIZE;
    if (total_chunks == 0) total_chunks = 1;
    if (total_chunks > 0xFFFFu) return;  // frame too large to index

    uint8_t flags = keyframe ? vid::FLAG_KEYFRAME : 0;

#ifdef WITH_TUI
    g_stats.note_video_frame_encoded(nbytes, total_chunks);
#endif

    std::vector<unsigned char> packet;
    for (std::size_t i = 0; i < total_chunks; ++i) {
        std::size_t offset    = i * vid::PAYLOAD_SIZE;
        std::size_t remaining = nbytes - offset;
        uint16_t this_payload =
            static_cast<uint16_t>(std::min(remaining, vid::PAYLOAD_SIZE));
        build_packet(packet, frame_id, static_cast<uint16_t>(i),
                     static_cast<uint16_t>(total_chunks), flags,
                     data + offset, this_payload);
        transmit_packet(tx, packet);
    }
}

// ---------------------- Producer/consumer frame queue -----------------------
// The capture+encode thread (producer) pulls compressed frames from the encoder
// at the full camera rate and pushes them here. The transmit thread (consumer)
// pops them and does the slow FEC + OFDM send. Decoupling the two keeps the
// appsink drained at the camera rate regardless of how slow the radio is, so
// the encoder's OUTPUT pool never backs up and FPS stays smooth under FEC
// bursts.
//
//   NOTE ON THE ~30 s "dst has not fd" CRASH: that was NOT caused by this
//   backpressure. It was a per-frame dmabuf fd leak in the encoder's *input*
//   zero-copy import path (it imports the HDMI-RX camera's DMA buffers and
//   leaks one fd each frame until the 1024 fd limit is hit). The real fix is
//   the forced colour-convert copy in the hardware pipeline string below; this
//   threading is kept only for FPS smoothing, not as the crash fix.
//
// When the consumer falls behind, the queue caps at its depth and drops the
// OLDEST non-keyframe frame — NEVER a keyframe, so the RX can always resync at
// the next keyframe. A dropped delta frame looks like packet loss to the RX,
// which its keyframe-resync handles.
struct EncodedFrame {
    std::vector<unsigned char> data;
    bool      keyframe = false;
    uint32_t  frame_id = 0;
};

class FrameQueue {
public:
    explicit FrameQueue(std::size_t max_depth) : m_max(max_depth) {}

    // Producer side. Never blocks; applies the keyframe-aware drop policy when
    // full. Returns the number of frames dropped to make room (usually 0).
    int push(EncodedFrame&& f) {
        int dropped = 0;
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            if (m_q.size() >= m_max) {
                // Drop the oldest NON-keyframe to make room: walk from the
                // front (oldest) and erase the first delta frame found.
                bool removed = false;
                for (auto it = m_q.begin(); it != m_q.end(); ++it) {
                    if (!it->keyframe) {
                        m_q.erase(it);
                        removed = true;
                        ++dropped;
                        break;
                    }
                }
                // Pathological case: every queued frame is a keyframe. Drop the
                // oldest to bound latency/memory.
                if (!removed) {
                    m_q.pop_front();
                    ++dropped;
                }
            }
            m_q.push_back(std::move(f));
        }
        m_cv.notify_one();
        return dropped;
    }

    // Consumer side. Blocks until a frame is available or the queue is stopped
    // and drained. Returns false only when stopped and empty.
    bool pop(EncodedFrame& out) {
        std::unique_lock<std::mutex> lk(m_mtx);
        m_cv.wait(lk, [this] { return !m_q.empty() || m_stop; });
        if (m_q.empty()) return false;
        out = std::move(m_q.front());
        m_q.pop_front();
        return true;
    }

    void stop() {
        { std::lock_guard<std::mutex> lk(m_mtx); m_stop = true; }
        m_cv.notify_all();
    }

private:
    std::deque<EncodedFrame> m_q;
    std::mutex               m_mtx;
    std::condition_variable  m_cv;
    std::size_t              m_max;
    bool                     m_stop = false;
};

int main(int /*argc*/, char** /*argv*/)
{
    gst_init(nullptr, nullptr);

    // ---- fd-exhaustion backstop (defence-in-depth) ----
    // The forced colour-convert copy in the hardware pipeline (below) is the
    // REAL fix for the encoder's per-frame dmabuf fd leak, so the fd count
    // should stay flat. This additionally raises the soft open-file limit to
    // the hard maximum: if any residual slow leak ever exists, it turns a
    // mid-flight encoder crash into many hours of headroom instead of ~30 s.
    // Harmless when there's no leak, and needs no privileges (soft is only
    // raised up to the existing hard limit). Remove if you don't want it.
    {
        struct rlimit rl;
        if (getrlimit(RLIMIT_NOFILE, &rl) == 0) {
            rl.rlim_cur = rl.rlim_max;
            if (setrlimit(RLIMIT_NOFILE, &rl) == 0)
                std::cout << "fd soft limit raised to " << rl.rlim_cur << "\n";
        }
    }

    std::cout << "fun_ofdm H.265 video TX"
#ifdef WITH_FEC
              << " [FEC]"
#endif
#ifdef WITH_TUI
              << " [TUI]"
#endif
              << "  " << FRAME_W << "x" << FRAME_H << " @ " << (BPS/1000000.0)
              << " Mbit/s, keyframe every " << GOP << "\n";

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

    // ---- Build the encode pipeline, auto-detecting the encoder ----
    // On the Radxa, mpph265enc (hardware VPU) is available — use it. On x86 it
    // isn't, so fall back to x265enc (software, ultrafast preset for real-time).
    // The rest of the pipeline (capture, parse, appsink) is identical; only the
    // encoder element and its property names differ.
    char desc[1024];
    bool have_hw =
        (gst_element_factory_find("mpph265enc") != nullptr);

    if (have_hw) {
        // Hardware path (Radxa RK3588 VPU).
        //   bps = bitrate (bits/s), rc-mode vbr, gop = keyframe interval.
        //   (If 'gop' is the wrong property name on your build, change it here;
        //    check `gst-inspect-1.0 mpph265enc | grep -iE "gop|key"`.)
        //
        // *** THE fd-LEAK FIX — DO NOT REMOVE ***
        //   videoconvert ! I420 ! videoconvert ! NV12 ! queue
        //
        //   /dev/video11 is the HDMI-RX input: a MULTIPLANAR V4L2 device whose
        //   buffers are DMA-backed. When mpph265enc zero-copy-imports those
        //   buffers it leaks ONE dmabuf fd per frame; after ~1024 frames
        //   (~32 s @ 30 fps) the process fd table fills and RGA dies with
        //   "dst has not fd and address for render". Proven by watching
        //   /proc/PID/fd: it climbs +30/s to 1023 then crashes. Feeding
        //   videotestsrc (plain CPU buffers) NEVER leaked, because the encoder
        //   then copies input into its own internal VPU pool instead of
        //   importing.
        //
        //   The two converts force exactly that copy: a genuine
        //   NV12->I420->NV12 colour conversion produces a fresh system-memory
        //   buffer with no dmabuf to import, so the encoder falls back to its
        //   leak-free internal pool. Verified: fd count flat at ~70 for
        //   minutes. The converts are cheap chroma re-orders at 720p (a few %
        //   CPU on the RK3588); the trailing queue gives the encoder its own
        //   thread.
        //
        //   DO NOT "optimise" this to a single same-format videoconvert — that
        //   negotiates to PASSTHROUGH (no copy), forwards the camera's dmabuf,
        //   and the leak returns. The format must actually change.
        std::snprintf(desc, sizeof(desc),
            "v4l2src device=%s ! "
            "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            "videoconvert ! video/x-raw,format=NV12 ! "
            "queue ! "
            "mpph265enc bps=%d rc-mode=cbr gop=%d ! "
            "h265parse config-interval=1 ! "
            "video/x-h265,stream-format=byte-stream,alignment=au ! "
            "appsink name=sink emit-signals=false sync=false "
            "max-buffers=4 drop=true",
            CAM_DEVICE, FRAME_W, FRAME_H, TARGET_FPS, BPS, GOP);
        std::cout << "Encoder: mpph265enc (HARDWARE / RK3588 VPU)\n";
    } else {
        // Software path (x86 or any board without the Rockchip encoder).
        //   x265enc: bitrate is in KBIT/s (not bits), key-int-max = keyframe
        //   interval in frames, speed-preset=ultrafast keeps it real-time.
        //   x265enc wants I420 input, so the caps request I420 (videoconvert
        //   bridges whatever the camera gives).
        std::snprintf(desc, sizeof(desc),
            "v4l2src device=%s ! "
            "video/x-raw,width=%d,height=%d,framerate=%d/1 ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            "x265enc bitrate=%d key-int-max=%d speed-preset=ultrafast "
            "tune=zerolatency ! "
            "h265parse config-interval=1 ! "
            "video/x-h265,stream-format=byte-stream,alignment=au ! "
            "appsink name=sink emit-signals=false sync=false "
            "max-buffers=4 drop=true",
            CAM_DEVICE, FRAME_W, FRAME_H, TARGET_FPS, (BPS / 1000), GOP);
        std::cout << "Encoder: x265enc (SOFTWARE, ultrafast). "
                     "mpph265enc not found — this is expected on x86.\n";
    }

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(desc, &err);
    if (!pipeline) {
        std::cerr << "Encode pipeline failed: "
                  << (err ? err->message : "?") << "\n";
        if (have_hw)
            std::cerr << "If it's the 'gop' property, run "
                         "`gst-inspect-1.0 mpph265enc` and use the right name.\n";
        else
            std::cerr << "Is gstreamer1.0-plugins-bad (x265enc) installed?\n";
        if (err) g_error_free(err);
        return 1;
    }
    GstAppSink* sink =
        GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), "sink"));
    if (!sink) { std::cerr << "no appsink\n"; return 1; }

    transmitter tx(FREQ, SAMPLE_RATE, TX_GAIN, TX_AMP,
                   "type=b200,serial=3123B0D");

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) ==
        GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Failed to start encode pipeline (camera/format/encoder)\n";
        return 1;
    }

    // ---- Producer/consumer split ----
    // Producer thread: pull encoded frames from the appsink as fast as they
    // arrive and queue them. It does NOTHING slow, so the encoder's output pool
    // is always drained and FPS stays smooth even when the radio is slow. (The
    // ~30 s "dst has not fd" crash was the encoder's INPUT dmabuf import leak,
    // not output backpressure — that is fixed by the forced colour-convert in
    // the pipeline string above. This split is for FPS smoothing.) The consumer
    // (this main thread) does the slow FEC + OFDM transmit. The bounded queue
    // absorbs FEC bursts; when the consumer can't keep up, the queue drops the
    // oldest delta frame (never a keyframe), which cooperates with the RX's
    // keyframe-resync.
    //
    // Depth 8 ≈ 0.27 s at 30 fps: enough to ride out FEC bursts without piling
    // on latency. Raise for burstier links, lower if latency matters more.
    FrameQueue queue(8);

    std::thread producer([&]() {
        uint32_t pid = 0;
        while (!g_stop.load()) {
            GstSample* sample =
                gst_app_sink_try_pull_sample(sink, 100 * GST_MSECOND);
            if (!sample) {
                if (gst_app_sink_is_eos(sink)) break;
                continue;
            }
            GstBuffer* buf = gst_sample_get_buffer(sample);
            EncodedFrame f;
            f.keyframe =
                !GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT);
            f.frame_id = pid++;     // every captured frame gets an id; gaps from
                                    // dropped frames signal loss to the RX.
            GstMapInfo map;
            if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
                f.data.assign(map.data, map.data + map.size);
                gst_buffer_unmap(buf, &map);
            }
            gst_sample_unref(sample);   // release the DMA buffer immediately

            if (!f.data.empty())
                queue.push(std::move(f));   // drop policy handled inside
        }
        queue.stop();   // unblock the consumer so it drains and exits
    });

    // Consumer (main thread): pop queued frames and do the slow FEC + transmit.
    EncodedFrame f;
    while (queue.pop(f)) {
        send_h265_frame(tx, f.data.data(), f.data.size(),
                        f.keyframe, f.frame_id);

#ifdef WITH_FEC
        // Periodic flush so a stalled stream still drains (don't flush every
        // frame — that pads generations and wastes airtime).
        static auto last_flush = std::chrono::steady_clock::now();
        auto now_f = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                now_f - last_flush).count() > 400) {
            g_enc.flush();
            last_flush = now_f;
            while (g_enc.has_phy_packet()) {
                tx.send_frame(g_enc.next_phy_packet(), PHY_RATE);
#  ifdef WITH_TUI
                g_stats.note_phy_packet_tx();
#  endif
            }
        }
#endif
    }

    producer.join();

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
#ifdef WITH_TUI
    tui.stop();
    tui_thread.join();
#endif
    return 0;
}

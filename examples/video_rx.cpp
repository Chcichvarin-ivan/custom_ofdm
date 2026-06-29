/*!
 * \file video_rx.cpp
 * \brief H.265 video RX over fun_ofdm (x86, software HEVC decode).
 *
 *   -DWITH_FEC enables RaptorQ FEC (must match the TX build).
 *   -DWITH_TUI enables the terminal link-health display.
 *
 * Receives packetized H.265 over fun_ofdm, reassembles each compressed frame,
 * and decodes it to video with GStreamer's software HEVC decoder
 * (appsrc ! h265parse ! avdec_h265 ! ... ! appsink), displaying with OpenCV.
 *
 * LOSS HANDLING (important for a lossy radio link):
 *   H.265 delta frames reference earlier frames. If a frame arrives incomplete
 *   (a packet was lost), feeding it — or any following delta frame — to the
 *   decoder produces garbage or errors. So after ANY incomplete frame, the RX
 *   enters a "need keyframe" state and DROPS frames until the next keyframe,
 *   which is a clean restart point. Keyframes arrive every ~15 frames (0.5 s)
 *   from the TX, so recovery is quick.
 */

#include <iostream>
#include <vector>
#include <map>
#include <cstring>
#include <cstdint>
#include <atomic>
#include <csignal>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <arpa/inet.h>

#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include "receiver.h"
#include "realtime.h"
#include "video_packet.h"

#ifdef WITH_FEC
#  include "fec_common.h"
#  include "fec_decoder.h"
#endif

// WITH_STATS is true when ANY consumer of LinkStats is enabled (TUI, the RSSI
// diagnostic, the on-video overlay, or the AGC). The g_stats counters and
// object are guarded by it; link_stats.h is included once here.
#if defined(WITH_TUI) || defined(WITH_DIAG) || defined(WITH_OVERLAY) || defined(WITH_AGC)
#  define WITH_STATS
#  include "link_stats.h"
#endif

#ifdef WITH_TUI
#  include "link_tui.h"
#endif
#ifdef WITH_DIAG
#  include "link_diagnostics.h"
#endif
#ifdef WITH_AGC
#  include "link_agc.h"
#endif
#ifdef WITH_OVERLAY
#  include "link_overlay.h"
#endif

using namespace fun;

// ---------------------- Radio parameters (match TX) -------------------------
static const double FREQ        = 3.3e9;
static const double SAMPLE_RATE = 20e6;
static const double RX_GAIN     = 15.0;

#ifdef WITH_FEC
static fec::FecDecoder g_dec;
#endif
#ifdef WITH_STATS
static stats::LinkStats g_stats;
#endif
#ifdef WITH_TUI
static stats::LinkTui*  g_tui_ptr = nullptr;
#endif
#ifdef WITH_DIAG
static stats::LinkDiagnostics* g_diag_ptr = nullptr;
#endif
#ifdef WITH_AGC
static stats::LinkAgc* g_agc_ptr = nullptr;
#endif
static std::atomic<bool> g_stop{false};

static void on_signal(int) {
    g_stop.store(true);
#ifdef WITH_TUI
    if (g_tui_ptr) g_tui_ptr->stop();
#endif
#ifdef WITH_DIAG
    if (g_diag_ptr) g_diag_ptr->stop();
#endif
#ifdef WITH_AGC
    if (g_agc_ptr) g_agc_ptr->stop();
#endif
}

// RAII guard: stops then joins a thread on any exit path, so a running thread
// is never destroyed (which would abort) and join never hangs.
struct ThreadJoiner {
    std::thread& t;
    std::function<void()> stopper;
    ThreadJoiner(std::thread& th, std::function<void()> stop)
        : t(th), stopper(std::move(stop)) {}
    ~ThreadJoiner() { if (stopper) stopper(); if (t.joinable()) t.join(); }
};

// ===========================================================================
//  H.265 decode pipeline (GStreamer, software decode)
//  We push reassembled compressed frames into appsrc; decoded raw frames come
//  out of appsink, which we convert to cv::Mat and display.
// ===========================================================================
struct H265Decoder {
    GstElement* pipeline = nullptr;
    GstElement* src      = nullptr;   // appsrc: we push H.265 bytes in
    GstAppSink* sink     = nullptr;   // appsink: decoded frames come out
    std::atomic<bool> ok{false};

    bool start() {
        // BGR out so OpenCV can use it directly. avdec_h265 = libav software
        // HEVC decoder. videoconvert handles the decoder's native format
        // (typically I420) -> BGR.
        const char* desc =
            "appsrc name=src is-live=true do-timestamp=true format=time "
            "  caps=video/x-h265,stream-format=byte-stream,alignment=au ! "
            "h265parse ! avdec_h265 ! videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=false sync=false max-buffers=2 drop=true";

        GError* err = nullptr;
        pipeline = gst_parse_launch(desc, &err);
        if (!pipeline) {
            std::cerr << "RX decode pipeline failed: "
                      << (err ? err->message : "?") << "\n";
            if (err) g_error_free(err);
            return false;
        }
        src  = gst_bin_get_by_name(GST_BIN(pipeline), "src");
        sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), "sink"));
        if (!src || !sink) {
            std::cerr << "RX: could not get appsrc/appsink\n";
            return false;
        }
        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) ==
            GST_STATE_CHANGE_FAILURE) {
            std::cerr << "RX: failed to start decode pipeline\n";
            return false;
        }
        ok.store(true);
        return true;
    }

    // Push one reassembled compressed H.265 frame into the decoder.
    void push(const std::vector<unsigned char>& frame) {
        if (!ok.load() || frame.empty()) return;
        GstBuffer* buf = gst_buffer_new_allocate(nullptr, frame.size(), nullptr);
        gst_buffer_fill(buf, 0, frame.data(), frame.size());
        GstFlowReturn r =
            gst_app_src_push_buffer(GST_APP_SRC(src), buf); // takes ownership
        if (r != GST_FLOW_OK) {
            // Non-fatal; decoder may be catching up.
        }
    }

    // Pull a decoded frame if one is ready (non-blocking). Returns empty Mat
    // if nothing available.
    cv::Mat try_pull() {
        if (!ok.load()) return cv::Mat();
        GstSample* sample =
            gst_app_sink_try_pull_sample(sink, 0); // 0 = non-blocking
        if (!sample) return cv::Mat();

        cv::Mat out;
        GstCaps* caps = gst_sample_get_caps(sample);
        GstBuffer* buf = gst_sample_get_buffer(sample);
        GstStructure* s = caps ? gst_caps_get_structure(caps, 0) : nullptr;
        int w = 0, h = 0;
        if (s) { gst_structure_get_int(s, "width", &w);
                 gst_structure_get_int(s, "height", &h); }
        GstMapInfo map;
        if (w > 0 && h > 0 && gst_buffer_map(buf, &map, GST_MAP_READ)) {
            // BGR, 3 bytes/pixel. Copy out (the buffer is freed on unref).
            cv::Mat tmp(h, w, CV_8UC3, (void*)map.data);
            out = tmp.clone();
            gst_buffer_unmap(buf, &map);
        }
        gst_sample_unref(sample);
        return out;
    }

    void stop() {
        if (src) gst_app_src_end_of_stream(GST_APP_SRC(src));
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
        ok.store(false);
    }
};

static H265Decoder g_decoder;

// ===========================================================================
//  Frame reassembly (same idea as the JPEG RX, plus keyframe tracking and
//  loss-aware resync).
// ===========================================================================
struct PartialFrame {
    uint16_t total_chunks = 0;
    uint16_t got = 0;
    bool     is_keyframe = false;
    std::vector<std::vector<unsigned char>> chunks; // indexed by chunk_index
    std::vector<bool> have;
};

static std::map<uint32_t, PartialFrame> g_partials;

// Resync state: after an incomplete frame, drop everything until next keyframe.
static bool     g_need_keyframe = true;   // start by waiting for a keyframe
static uint32_t g_last_pushed_frame = 0;
static bool     g_have_pushed = false;

// Reassemble a completed frame and push it to the decoder (with loss logic).
static void deliver_frame(uint32_t frame_id, PartialFrame& pf)
{
    // Concatenate chunks in order into one compressed H.265 access unit.
    std::vector<unsigned char> frame;
    frame.reserve(pf.total_chunks * vid::PAYLOAD_SIZE);
    for (uint16_t i = 0; i < pf.total_chunks; ++i)
        frame.insert(frame.end(), pf.chunks[i].begin(), pf.chunks[i].end());

    // ---- loss-aware gating ----
    if (g_need_keyframe) {
        if (!pf.is_keyframe) {
            // Still waiting for a clean restart point; drop this delta frame.
#ifdef WITH_STATS
            g_stats.note_video_frame_dropped();
#endif
            return;
        }
        // A keyframe — we can resync here.
        g_need_keyframe = false;
    }

    g_decoder.push(frame);
    g_last_pushed_frame = frame_id;
    g_have_pushed = true;
#ifdef WITH_STATS
    g_stats.note_video_frame_displayed(frame.size());
#endif
}

// Called when a frame is detected as incomplete (we moved past it without all
// its packets). Triggers resync.
static void note_incomplete_frame()
{
    g_need_keyframe = true;
#ifdef WITH_STATS
    g_stats.note_video_frame_dropped();
#endif
}

static void process_app_packet(const std::vector<unsigned char>& pkt)
{
    if (pkt.size() != vid::PACKET_SIZE) {
#ifdef WITH_STATS
        g_stats.note_phy_packet_rejected();
#endif
        return;
    }
    uint32_t magic_n;
    std::memcpy(&magic_n, &pkt[vid::OFF_MAGIC], 4);
    if (ntohl(magic_n) != vid::MAGIC) {
#ifdef WITH_STATS
        g_stats.note_phy_packet_rejected();
#endif
        return;
    }

    uint32_t frame_id_n; uint16_t chunk_n, total_n, paysz_n;
    std::memcpy(&frame_id_n, &pkt[vid::OFF_FRAME_ID],   4);
    std::memcpy(&chunk_n,    &pkt[vid::OFF_CHUNK_IDX],  2);
    std::memcpy(&total_n,    &pkt[vid::OFF_TOTAL],      2);
    std::memcpy(&paysz_n,    &pkt[vid::OFF_PAYLOAD_SZ], 2);
    uint8_t flags = pkt[vid::OFF_FLAGS];

    uint32_t frame_id    = ntohl(frame_id_n);
    uint16_t chunk_index = ntohs(chunk_n);
    uint16_t total_chunks= ntohs(total_n);
    uint16_t payload_sz  = ntohs(paysz_n);
    bool     keyframe    = (flags & vid::FLAG_KEYFRAME) != 0;

    if (total_chunks == 0 || chunk_index >= total_chunks) return;
    if (payload_sz > vid::PAYLOAD_SIZE) return;

#ifdef WITH_STATS
    g_stats.note_app_packet_rx();
#endif

    PartialFrame& pf = g_partials[frame_id];
    if (pf.total_chunks == 0) {
        pf.total_chunks = total_chunks;
        pf.is_keyframe  = keyframe;
        pf.chunks.assign(total_chunks, std::vector<unsigned char>());
        pf.have.assign(total_chunks, false);
    }
    if (!pf.have[chunk_index]) {
        pf.chunks[chunk_index].assign(pkt.begin() + vid::HEADER_SIZE,
                                      pkt.begin() + vid::HEADER_SIZE + payload_sz);
        pf.have[chunk_index] = true;
        pf.got++;
    }

    // Complete?
    if (pf.got == pf.total_chunks) {
        deliver_frame(frame_id, pf);
        g_partials.erase(frame_id);
    }

    // Garbage-collect older partials: if we've completed/seen a newer frame,
    // any still-incomplete older frame lost packets — mark resync + drop it.
    for (auto it = g_partials.begin(); it != g_partials.end(); ) {
        // "older" by a margin (frame_id wraps are far away in practice)
        if ((int64_t)frame_id - (int64_t)it->first > 4) {
            note_incomplete_frame();      // an older frame never completed
            it = g_partials.erase(it);
        } else {
            ++it;
        }
    }
}

// fun_ofdm receive callback.
static void rx_callback(std::vector<std::vector<unsigned char>> packets)
{
    for (auto& pkt : packets) {
#ifdef WITH_STATS
        g_stats.note_phy_packet_rx();
#endif
#ifdef WITH_FEC
        std::vector<std::vector<unsigned char>> apps = g_dec.process_phy_packet(pkt);
        for (auto& app : apps) process_app_packet(app);
#else
        process_app_packet(pkt);
#endif
    }
}

int main(int /*argc*/, char** /*argv*/)
{
    gst_init(nullptr, nullptr);

    std::cout << "fun_ofdm H.265 video RX"
#ifdef WITH_FEC
              << " [FEC]"
#endif
#ifdef WITH_TUI
              << " [TUI]"
#endif
#ifdef WITH_DIAG
              << " [DIAG]"
#endif
#ifdef WITH_AGC
              << " [AGC]"
#endif
#ifdef WITH_OVERLAY
              << " [OVERLAY]"
#endif
              << "\n";

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);
    set_realtime_priority(1.0f);

    if (!g_decoder.start()) {
        std::cerr << "Could not start H.265 decoder. Is gstreamer-libav "
                     "(avdec_h265) installed?\n";
        return 1;
    }

    // Feed FEC decode/drop counts into the stats (TUI error-rate + diagnostic).
#if defined(WITH_FEC) && defined(WITH_STATS)
    g_dec.set_stats(&g_stats);
#endif

    // Construct the receiver (and USRP) FIRST, before any worker threads, so a
    // device-init failure can't leave a running thread to be destroyed (abort).
    receiver rx(&rx_callback, FREQ, SAMPLE_RATE, RX_GAIN,
                "type=b200,serial=314C000,num_recv_frames=700,"
                "num_send_frames=700,recv_frame_size=11000,"
                "send_frame_size=11000");

    // ---- worker threads, each guarded by a ThreadJoiner ----
#ifdef WITH_TUI
    stats::LinkTui tui(g_stats, stats::TuiMode::RX);
    g_tui_ptr = &tui;
    std::thread tui_thread([&tui]() { tui.run(); });
    ThreadJoiner tui_joiner(tui_thread, [&tui]() { tui.stop(); });
#endif

#ifdef WITH_DIAG
    stats::LinkDiagnostics diag(rx.get_multi_usrp(), g_stats, /*chan=*/0);
    g_diag_ptr = &diag;
#  if defined(WITH_TUI) || defined(WITH_OVERLAY)
    diag.set_console_output(false);   // display shows it instead
#  endif
    std::thread diag_thread([&diag]() { diag.run(); });
    ThreadJoiner diag_joiner(diag_thread, [&diag]() { diag.stop(); });
#endif

#ifdef WITH_AGC
    // Slow software AGC: keeps RSSI in a target window by nudging RX gain ~1Hz.
    // Owns the gain (don't also enable UHD AGC). Parameters match the design.
    stats::LinkAgc agc(rx.get_multi_usrp(), g_stats, /*chan=*/0);
    agc.set_window(-60.0, -35.0);
    agc.set_gain_band(10.0, 60.0);
    agc.set_step_db(1.0);
    agc.set_period_ms(1000);
    g_agc_ptr = &agc;
#  if defined(WITH_TUI) || defined(WITH_OVERLAY)
    agc.set_console_output(false);
#  endif
    std::thread agc_thread([&agc]() { agc.run(); });
    ThreadJoiner agc_joiner(agc_thread, [&agc]() { agc.stop(); });
#endif

#ifdef WITH_OVERLAY
    stats::LinkOverlay overlay(g_stats, stats::TuiMode::RX);
#endif

    cv::namedWindow("fun_ofdm H.265 RX", cv::WINDOW_AUTOSIZE);
#ifndef WITH_TUI
    std::cout << "Press 'q' in the video window to quit.\n";
#endif

    // Main loop: pull decoded frames and display. The receive happens on
    // fun_ofdm's callback thread; decoded frames surface here.
    while (!g_stop.load()) {
        cv::Mat frame = g_decoder.try_pull();
        if (!frame.empty()) {
#ifdef WITH_OVERLAY
            overlay.render(frame);   // draw RSSI/verdict/gain HUD on the video
#endif
            cv::imshow("fun_ofdm H.265 RX", frame);
        }
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) g_stop = true;
        if (frame.empty())
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    rx.pause();
    g_decoder.stop();
    cv::destroyAllWindows();

    // Signal worker threads to stop; ThreadJoiners join them at scope exit.
#ifdef WITH_DIAG
    diag.stop();
#endif
#ifdef WITH_AGC
    agc.stop();
#endif
#ifdef WITH_TUI
    tui.stop();
#endif
    return 0;
}

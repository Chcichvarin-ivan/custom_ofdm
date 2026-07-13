// link_stats.h — thread-safe counters for the video link.
//
// Updated from any thread (atomics for counters), sampled by the TUI thread
// for rate calculation. No external dependencies.
//
// RATE READOUT (RX): two bitrates are exposed so the dashboard is meaningful
// even though frames reach the display in bursty clumps:
//   * link_kbps_rx  — bytes of app packets ARRIVING (after FEC). Steady; proves
//                     the radio link is carrying data regardless of display.
//   * app_kbps_rx   — bytes of frames actually DECODED/displayed. Dips when a
//                     frame is dropped, so it reflects what the viewer sees.
// All per-second rates are EMA-smoothed (see alpha) so FEC-generation clumping
// and VBR scene changes don't make the numbers jump.

#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>

namespace stats {

enum class TuiMode { TX, RX };

enum class LinkVerdict { Unknown, Healthy, Saturation, Weak, NoSignal, MidErrors };

class LinkStats {
public:
    LinkStats() : m_start(std::chrono::steady_clock::now()) {}

    // ---- TX-side increments ----
    void note_video_frame_encoded(size_t jpeg_bytes, unsigned chunks) {
        m_video_frames_encoded.fetch_add(1, std::memory_order_relaxed);
        m_jpeg_bytes.fetch_add(jpeg_bytes, std::memory_order_relaxed);
        m_app_packets_tx.fetch_add(chunks, std::memory_order_relaxed);
    }
    void note_fec_gen_encoded(unsigned phy_packets) {
        m_fec_gens_encoded.fetch_add(1, std::memory_order_relaxed);
        m_phy_packets_tx.fetch_add(phy_packets, std::memory_order_relaxed);
    }
    void note_phy_packet_tx() {
        m_phy_packets_tx.fetch_add(1, std::memory_order_relaxed);
    }

    // ---- RX-side increments ----
    void note_phy_packet_rx() {
        m_phy_packets_rx.fetch_add(1, std::memory_order_relaxed);
    }
    void note_phy_packet_rejected() {
        m_phy_packets_rejected.fetch_add(1, std::memory_order_relaxed);
    }
    void note_fec_gen_decoded(unsigned source_used, unsigned repair_used) {
        m_fec_gens_decoded.fetch_add(1, std::memory_order_relaxed);
        m_fec_source_used.fetch_add(source_used, std::memory_order_relaxed);
        m_fec_repair_used.fetch_add(repair_used, std::memory_order_relaxed);
    }
    void note_fec_gen_dropped() {
        m_fec_gens_dropped.fetch_add(1, std::memory_order_relaxed);
    }
    // payload_bytes = the actual video bytes carried by this app packet. Feeds
    // the steady "received link rate" (link_kbps_rx).
    void note_app_packet_rx(size_t payload_bytes) {
        m_app_packets_rx.fetch_add(1, std::memory_order_relaxed);
        m_app_bytes_rx.fetch_add(payload_bytes, std::memory_order_relaxed);
    }
    void note_video_frame_displayed(size_t jpeg_bytes) {
        m_video_frames_displayed.fetch_add(1, std::memory_order_relaxed);
        m_jpeg_bytes_rx.fetch_add(jpeg_bytes, std::memory_order_relaxed);
    }
    void note_video_frame_dropped() {
        m_video_frames_dropped.fetch_add(1, std::memory_order_relaxed);
    }

    // ---- RF-layer metrics, written by LinkDiagnostics / AGC ----
    void update_rssi_dbm(double dbm, bool valid) {
        m_rssi_mdbm.store(static_cast<int64_t>(dbm * 1000.0),
                          std::memory_order_relaxed);
        m_have_rssi.store(valid, std::memory_order_relaxed);
    }
    void update_verdict(LinkVerdict v) {
        m_verdict.store(static_cast<int>(v), std::memory_order_relaxed);
    }
    void update_rx_gain_db(double db) {
        m_rx_gain_mdb.store(static_cast<int64_t>(db * 1000.0),
                            std::memory_order_relaxed);
        m_have_gain.store(true, std::memory_order_relaxed);
    }

    // ---- Snapshot for the TUI ----
    struct Snapshot {
        uint64_t video_frames_encoded = 0;
        uint64_t video_frames_displayed = 0;
        uint64_t video_frames_dropped = 0;
        uint64_t app_packets_tx = 0;
        uint64_t app_packets_rx = 0;
        uint64_t app_bytes_rx = 0;
        uint64_t phy_packets_tx = 0;
        uint64_t phy_packets_rx = 0;
        uint64_t phy_packets_rejected = 0;
        uint64_t fec_gens_encoded = 0;
        uint64_t fec_gens_decoded = 0;
        uint64_t fec_gens_dropped = 0;
        uint64_t fec_source_used = 0;
        uint64_t fec_repair_used = 0;
        uint64_t jpeg_bytes = 0;
        uint64_t jpeg_bytes_rx = 0;
        // Derived per-second rates (EMA-smoothed)
        double video_fps_tx = 0.0;
        double video_fps_rx = 0.0;
        double phy_pps_tx = 0.0;
        double phy_pps_rx = 0.0;
        double app_kbps_tx = 0.0;      // TX encoder bitrate
        double app_kbps_rx = 0.0;      // RX DECODED/displayed video rate
        double link_kbps_rx = 0.0;     // RX RECEIVED link rate (steady)
        double fec_repair_pct = 0.0;
        double rejection_pct = 0.0;
        double uptime_s = 0.0;
        bool        have_rssi = false;
        double      rssi_dbm  = 0.0;
        LinkVerdict verdict   = LinkVerdict::Unknown;
        bool        have_gain = false;
        double      rx_gain_db = 0.0;
    };

    Snapshot sample() {
        std::lock_guard<std::mutex> lock(m_mu);
        Snapshot s;
        s.video_frames_encoded   = m_video_frames_encoded.load();
        s.video_frames_displayed = m_video_frames_displayed.load();
        s.video_frames_dropped   = m_video_frames_dropped.load();
        s.app_packets_tx         = m_app_packets_tx.load();
        s.app_packets_rx         = m_app_packets_rx.load();
        s.app_bytes_rx           = m_app_bytes_rx.load();
        s.phy_packets_tx         = m_phy_packets_tx.load();
        s.phy_packets_rx         = m_phy_packets_rx.load();
        s.phy_packets_rejected   = m_phy_packets_rejected.load();
        s.fec_gens_encoded       = m_fec_gens_encoded.load();
        s.fec_gens_decoded       = m_fec_gens_decoded.load();
        s.fec_gens_dropped       = m_fec_gens_dropped.load();
        s.fec_source_used        = m_fec_source_used.load();
        s.fec_repair_used        = m_fec_repair_used.load();
        s.jpeg_bytes             = m_jpeg_bytes.load();
        s.jpeg_bytes_rx          = m_jpeg_bytes_rx.load();

        const auto now = std::chrono::steady_clock::now();
        s.uptime_s = std::chrono::duration<double>(now - m_start).count();

        const double dt = std::chrono::duration<double>(now - m_last_t).count();
        if (m_have_last && dt > 0.001) {
            // Instantaneous rates over this interval.
            double v_fps_tx  = (s.video_frames_encoded   - m_last_vft) / dt;
            double v_fps_rx  = (s.video_frames_displayed - m_last_vfr) / dt;
            double p_pps_tx  = (s.phy_packets_tx - m_last_phy_tx) / dt;
            double p_pps_rx  = (s.phy_packets_rx - m_last_phy_rx) / dt;
            double a_kbps_tx = ((s.jpeg_bytes    - m_last_jb)   * 8.0/1000.0)/dt;
            double a_kbps_rx = ((s.jpeg_bytes_rx - m_last_jbrx) * 8.0/1000.0)/dt;
            double l_kbps_rx = ((s.app_bytes_rx  - m_last_appb) * 8.0/1000.0)/dt;

            // EMA smoothing. alpha = weight of the newest sample: lower is
            // calmer/laggier, higher is twitchier. 0.1 ~ a few-second window,
            // steady enough to read while still tracking real changes.
            const double alpha = 0.1;
            auto ema = [alpha](double prev, double cur, bool have) {
                return have ? (prev * (1.0 - alpha) + cur * alpha) : cur;
            };

            // A truly quiet interval (no PHY at all) means the source paused;
            // hold the last smoothed values rather than decaying toward 0.
            const bool quiet_tick =
                (s.phy_packets_rx == m_last_phy_rx) &&
                (s.phy_packets_tx == m_last_phy_tx);
            if (quiet_tick && m_have_rates) {
                s.video_fps_tx = m_last_v_fps_tx;
                s.video_fps_rx = m_last_v_fps_rx;
                s.phy_pps_tx   = m_last_p_pps_tx;
                s.phy_pps_rx   = m_last_p_pps_rx;
                s.app_kbps_tx  = m_last_a_kbps_tx;
                s.app_kbps_rx  = m_last_a_kbps_rx;
                s.link_kbps_rx = m_last_link_kbps;
            } else {
                s.video_fps_tx = ema(m_last_v_fps_tx,  v_fps_tx,  m_have_rates);
                s.video_fps_rx = ema(m_last_v_fps_rx,  v_fps_rx,  m_have_rates);
                s.phy_pps_tx   = ema(m_last_p_pps_tx,  p_pps_tx,  m_have_rates);
                s.phy_pps_rx   = ema(m_last_p_pps_rx,  p_pps_rx,  m_have_rates);
                s.app_kbps_tx  = ema(m_last_a_kbps_tx, a_kbps_tx, m_have_rates);
                s.app_kbps_rx  = ema(m_last_a_kbps_rx, a_kbps_rx, m_have_rates);
                s.link_kbps_rx = ema(m_last_link_kbps, l_kbps_rx, m_have_rates);
                m_last_v_fps_tx  = s.video_fps_tx;  m_last_v_fps_rx  = s.video_fps_rx;
                m_last_p_pps_tx  = s.phy_pps_tx;    m_last_p_pps_rx  = s.phy_pps_rx;
                m_last_a_kbps_tx = s.app_kbps_tx;   m_last_a_kbps_rx = s.app_kbps_rx;
                m_last_link_kbps = s.link_kbps_rx;
                m_have_rates = true;
            }

            const uint64_t total = (s.fec_source_used + s.fec_repair_used)
                                 - (m_last_src + m_last_rep);
            const uint64_t reps  = s.fec_repair_used - m_last_rep;
            s.fec_repair_pct = (total > 0) ? (100.0 * reps / total) : 0.0;
            const uint64_t seen = (s.phy_packets_rx + s.phy_packets_rejected)
                                - (m_last_phy_rx + m_last_phy_rej);
            const uint64_t rej  = s.phy_packets_rejected - m_last_phy_rej;
            s.rejection_pct = (seen > 0) ? (100.0 * rej / seen) : 0.0;
        }
        m_last_vft     = s.video_frames_encoded;
        m_last_vfr     = s.video_frames_displayed;
        m_last_phy_tx  = s.phy_packets_tx;
        m_last_phy_rx  = s.phy_packets_rx;
        m_last_phy_rej = s.phy_packets_rejected;
        m_last_jb      = s.jpeg_bytes;
        m_last_jbrx    = s.jpeg_bytes_rx;
        m_last_appb    = s.app_bytes_rx;
        m_last_src     = s.fec_source_used;
        m_last_rep     = s.fec_repair_used;
        m_last_t       = now;
        m_have_last    = true;

        s.have_rssi = m_have_rssi.load(std::memory_order_relaxed);
        s.rssi_dbm  = m_rssi_mdbm.load(std::memory_order_relaxed) / 1000.0;
        s.verdict   = static_cast<LinkVerdict>(
                          m_verdict.load(std::memory_order_relaxed));
        s.have_gain  = m_have_gain.load(std::memory_order_relaxed);
        s.rx_gain_db = m_rx_gain_mdb.load(std::memory_order_relaxed) / 1000.0;
        return s;
    }

private:
    std::atomic<uint64_t> m_video_frames_encoded{0};
    std::atomic<uint64_t> m_video_frames_displayed{0};
    std::atomic<uint64_t> m_video_frames_dropped{0};
    std::atomic<uint64_t> m_app_packets_tx{0};
    std::atomic<uint64_t> m_app_packets_rx{0};
    std::atomic<uint64_t> m_app_bytes_rx{0};
    std::atomic<uint64_t> m_phy_packets_tx{0};
    std::atomic<uint64_t> m_phy_packets_rx{0};
    std::atomic<uint64_t> m_phy_packets_rejected{0};
    std::atomic<uint64_t> m_fec_gens_encoded{0};
    std::atomic<uint64_t> m_fec_gens_decoded{0};
    std::atomic<uint64_t> m_fec_gens_dropped{0};
    std::atomic<uint64_t> m_fec_source_used{0};
    std::atomic<uint64_t> m_fec_repair_used{0};
    std::atomic<uint64_t> m_jpeg_bytes{0};
    std::atomic<uint64_t> m_jpeg_bytes_rx{0};
    std::atomic<int64_t> m_rssi_mdbm{0};
    std::atomic<bool>    m_have_rssi{false};
    std::atomic<int>     m_verdict{0};
    std::atomic<int64_t> m_rx_gain_mdb{0};
    std::atomic<bool>    m_have_gain{false};

    std::mutex m_mu;
    std::chrono::steady_clock::time_point m_start;
    std::chrono::steady_clock::time_point m_last_t;
    bool m_have_last = false;
    uint64_t m_last_vft = 0, m_last_vfr = 0;
    uint64_t m_last_phy_tx = 0, m_last_phy_rx = 0, m_last_phy_rej = 0;
    uint64_t m_last_jb = 0, m_last_jbrx = 0, m_last_appb = 0;
    uint64_t m_last_src = 0, m_last_rep = 0;
    bool m_have_rates = false;
    double m_last_v_fps_tx = 0, m_last_v_fps_rx = 0;
    double m_last_p_pps_tx = 0, m_last_p_pps_rx = 0;
    double m_last_a_kbps_tx = 0, m_last_a_kbps_rx = 0;
    double m_last_link_kbps = 0;
};

}  // namespace stats

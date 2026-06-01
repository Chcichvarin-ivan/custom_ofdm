// link_tui.h — terminal display for the video link.
//
// Uses ANSI escape codes — no ncurses dependency. Renders once per second
// from its own thread. Selects layout based on Mode: TX shows outgoing
// stats, RX shows incoming + recovery stats.

#pragma once
#include "link_stats.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace stats {



class LinkTui {
public:
    LinkTui(LinkStats& s, TuiMode mode) : m_stats(s), m_mode(mode) {}
    void stop() { m_running.store(false); }

    void run() {
        std::cout << "\x1b[?25l\x1b[2J" << std::flush;   // hide cursor, clear
        while (m_running.load()) {
            auto s = m_stats.sample();
            if (m_mode == TuiMode::TX) render_tx(s);
            else                       render_rx(s);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "\x1b[?25h\x1b[0m\n" << std::flush;  // restore cursor
    }

private:
    static constexpr const char* RESET  = "\x1b[0m";
    static constexpr const char* BOLD   = "\x1b[1m";
    static constexpr const char* DIM    = "\x1b[2m";
    static constexpr const char* RED    = "\x1b[31m";
    static constexpr const char* GREEN  = "\x1b[32m";
    static constexpr const char* YELLOW = "\x1b[33m";
    static constexpr const char* CYAN   = "\x1b[36m";
    static constexpr const char* GRAY   = "\x1b[90m";

    static std::string cup(int r, int c) {
        std::ostringstream o; o << "\x1b[" << r << ";" << c << "H";
        return o.str();
    }
    static constexpr const char* EOL = "\x1b[K";

    static std::string bar(double pct, int w) {
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        int filled = static_cast<int>(pct * w / 100.0);
        std::string s;
        for (int i = 0; i < w; ++i)
            s += (i < filled) ? "\xe2\x96\x88" : "\xe2\x96\x91";
        return s;
    }

    static const char* color(double v, double good, double warn,
                              bool higher_better = true) {
        if (higher_better) {
            if (v >= good) return GREEN;
            if (v >= warn) return YELLOW;
            return RED;
        }
        if (v <= good) return GREEN;
        if (v <= warn) return YELLOW;
        return RED;
    }

    static std::string fmt_duration(double s) {
        int t = static_cast<int>(s);
        int h = t / 3600, m = (t / 60) % 60, sec = t % 60;
        std::ostringstream o;
        if (h > 0) o << h << "h ";
        o << std::setw(2) << std::setfill('0') << m << ":"
          << std::setw(2) << std::setfill('0') << sec;
        return o.str();
    }

    static std::string fmt_kbps(double k) {
        std::ostringstream o;
        if (k < 1) o << std::fixed << std::setprecision(0) << (k * 1000) << " bps";
        else if (k < 1000) o << std::fixed << std::setprecision(1) << k << " kbps";
        else o << std::fixed << std::setprecision(2) << (k / 1000.0) << " Mbps";
        return o.str();
    }

    void render_tx(const LinkStats::Snapshot& s) {
        std::ostringstream o;
        o << cup(1,1) << BOLD << CYAN << "  fun_ofdm Video TX" << RESET
          << DIM << "   uptime: " << fmt_duration(s.uptime_s) << RESET
          << EOL << "\n";
        o << cup(2,1) << GRAY << "  " << std::string(60,'-') << RESET
          << EOL << "\n";

        int r = 4;
        o << cup(r++,1) << BOLD << "Video capture" << RESET << EOL;
        o << cup(r++,1) << "  Frames    : " << s.video_frames_encoded
          << " (" << std::fixed << std::setprecision(1)
          << s.video_fps_tx << " fps)" << EOL;
        o << cup(r++,1) << "  Bitrate   : " << fmt_kbps(s.app_kbps_tx) << EOL;
        o << cup(r++,1) << "  Packets   : " << s.app_packets_tx
          << GRAY << " app pkts sent" << RESET << EOL;

        r++;
        o << cup(r++,1) << BOLD << "PHY layer" << RESET << EOL;
        o << cup(r++,1) << "  PHY pkts  : " << s.phy_packets_tx
          << " (" << std::fixed << std::setprecision(0)
          << s.phy_pps_tx << "/s)" << EOL;

        r++;
        o << cup(r++,1) << BOLD << "FEC layer" << RESET << EOL;
        if (s.fec_gens_encoded > 0) {
            o << cup(r++,1) << "  Gens enc  : " << s.fec_gens_encoded << EOL;
            o << cup(r++,1) << GRAY
              << "  ratio 32 src + 8 repair per gen" << RESET << EOL;
        } else {
            o << cup(r++,1) << DIM << "  (not compiled with WITH_FEC)"
              << RESET << EOL;
        }

        r++;
        o << cup(r++,1) << DIM << "  Press Ctrl-C to quit" << RESET << EOL;
        std::cout << o.str() << std::flush;
    }

    void render_rx(const LinkStats::Snapshot& s) {
        std::ostringstream o;
        o << cup(1,1) << BOLD << CYAN << "  fun_ofdm Video RX" << RESET
          << DIM << "   uptime: " << fmt_duration(s.uptime_s) << RESET
          << EOL << "\n";
        o << cup(2,1) << GRAY << "  " << std::string(60,'-') << RESET
          << EOL << "\n";

        int r = 4;
        o << cup(r++,1) << BOLD << "PHY layer" << RESET << EOL;
        o << cup(r++,1) << "  PHY pkts  : " << s.phy_packets_rx
          << " (" << std::fixed << std::setprecision(0)
          << s.phy_pps_rx << "/s)" << EOL;
        if (s.phy_packets_rejected > 0) {
            o << cup(r++,1) << "  Rejected  : "
              << YELLOW << s.phy_packets_rejected << RESET
              << " (wrong size/magic)" << EOL;
        } else {
            o << cup(r++,1) << GRAY << "  Rejected  : none" << RESET << EOL;
        }

        r++;
        o << cup(r++,1) << BOLD << "FEC layer" << RESET << EOL;
        if (s.fec_gens_decoded > 0 || s.fec_gens_dropped > 0) {
            o << cup(r++,1) << "  Decoded   : " << GREEN << s.fec_gens_decoded
              << RESET << GRAY << " gens recovered" << RESET << EOL;
            o << cup(r++,1) << "  Dropped   : "
              << (s.fec_gens_dropped > 0 ? RED : GRAY)
              << s.fec_gens_dropped << RESET
              << GRAY << " gens lost (too many losses)" << RESET << EOL;

            const char* rc = color(s.fec_repair_pct, 5.0, 15.0, false);
            o << cup(r++,1) << "  Repair use: " << rc
              << bar(s.fec_repair_pct, 24) << RESET
              << " " << rc << std::fixed << std::setprecision(1)
              << s.fec_repair_pct << "%" << RESET << EOL;
            o << cup(r++,1) << GRAY
              << "    0%=clean   ~20%=at limit   >25%=unrecoverable"
              << RESET << EOL;
        } else {
            o << cup(r++,1) << DIM << "  (not compiled with WITH_FEC)"
              << RESET << EOL;
        }

        r++;
        o << cup(r++,1) << BOLD << "Application layer" << RESET << EOL;
        o << cup(r++,1) << "  App pkts  : " << s.app_packets_rx << EOL;
        o << cup(r++,1) << "  Frames    : " << s.video_frames_displayed
          << " (" << std::fixed << std::setprecision(1)
          << s.video_fps_rx << " fps)" << EOL;
        if (s.video_frames_dropped > 0) {
            o << cup(r++,1) << "  Dropped   : " << RED
              << s.video_frames_dropped << RESET << " incomplete frames"
              << EOL;
        }
        o << cup(r++,1) << "  Bitrate   : " << fmt_kbps(s.app_kbps_rx) << EOL;

        r++;
        o << cup(r++,1) << DIM << "  Press 'q' in video window to quit"
          << RESET << EOL;
        std::cout << o.str() << std::flush;
    }

    LinkStats& m_stats;
    TuiMode    m_mode;
    std::atomic<bool> m_running{true};
};

}  // namespace stats

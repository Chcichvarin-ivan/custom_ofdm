// link_overlay.h — render LinkStats as a HUD overlay on a cv::Mat.
//
// This file is only included when WITH_OVERLAY is defined, which implies
// OpenCV is available. The overlay draws stats in the top-left corner of
// the video frame with a semi-transparent dark background for legibility.
//
// Threshold-based color coding matches the terminal TUI:
//   green   = healthy
//   yellow  = marginal
//   red     = failing
//
// Call render(stats, frame) on each video frame before imshow.

#pragma once
#include "link_stats.h"

#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sstream>
#include <string>

namespace stats {

class LinkOverlay {
public:
    LinkOverlay(LinkStats& s, TuiMode mode) : m_stats(s), m_mode(mode) {}

    // Draw the current stats onto `frame`. Returns the modified frame.
    void render(cv::Mat& frame) {
        auto snap = m_stats.sample();

        // Color palette in BGR (OpenCV native)
        const cv::Scalar WHITE  (255, 255, 255);
        const cv::Scalar GRAY   (180, 180, 180);
        const cv::Scalar GREEN  (80,  220, 80);
        const cv::Scalar YELLOW (40,  220, 220);
        const cv::Scalar RED    (60,  60,  255);
        const cv::Scalar CYAN   (220, 220, 80);

        // Build the lines we're going to draw.
        struct Line { std::string text; cv::Scalar color; };
        std::vector<Line> lines;

        // Header
        std::ostringstream hdr;
        hdr << (m_mode == TuiMode::TX ? "TX  " : "RX  ");
        hdr << fmt_duration(snap.uptime_s);
        lines.push_back({hdr.str(), CYAN});

        if (m_mode == TuiMode::TX) {
            // ---- TX stats ----
            std::ostringstream l1;
            l1 << "FPS    " << std::fixed << std::setprecision(1)
               << snap.video_fps_tx;
            lines.push_back({l1.str(),
                             color_threshold(snap.video_fps_tx, 25, 15, true)});

            std::ostringstream l2;
            l2 << "Rate   " << fmt_kbps(snap.app_kbps_tx);
            lines.push_back({l2.str(), WHITE});

            std::ostringstream l3;
            l3 << "PHY    " << static_cast<int>(snap.phy_pps_tx) << "/s";
            lines.push_back({l3.str(), WHITE});

            if (snap.fec_gens_encoded > 0) {
                std::ostringstream l4;
                l4 << "FEC G  " << snap.fec_gens_encoded;
                lines.push_back({l4.str(), GRAY});
            }
        } else {
            // ---- RX stats ----
            // RF-layer line first (RSSI + verdict) — most important for
            // diagnosing saturation vs weak signal at a glance.
            {
                cv::Scalar vcolor = WHITE;
                std::string vtext;
                switch (snap.verdict) {
                    case stats::LinkVerdict::Healthy:
                        vtext = "OK";       vcolor = GREEN;  break;
                    case stats::LinkVerdict::Saturation:
                        vtext = "SAT!";     vcolor = RED;    break;
                    case stats::LinkVerdict::Weak:
                        vtext = "WEAK";     vcolor = YELLOW; break;
                    case stats::LinkVerdict::NoSignal:
                        vtext = "NO SIG";   vcolor = RED;    break;
                    case stats::LinkVerdict::MidErrors:
                        vtext = "ERR";      vcolor = YELLOW; break;
                    default:
                        vtext = "--";       vcolor = GRAY;   break;
                }
                std::ostringstream lr;
                lr << "RF     ";
                if (snap.have_rssi) {
                    lr << std::fixed << std::setprecision(0)
                       << snap.rssi_dbm << "dBm ";
                } else {
                    lr << "--- ";
                }
                lr << vtext;
                lines.push_back({lr.str(), vcolor});
            }

            std::ostringstream l1;
            l1 << "PHY    " << static_cast<int>(snap.phy_pps_rx) << "/s";
            lines.push_back({l1.str(),
                             color_threshold(snap.phy_pps_rx, 50, 10, true)});

            if (snap.phy_packets_rejected > 0) {
                std::ostringstream l2;
                l2 << "Rej    " << snap.phy_packets_rejected;
                lines.push_back({l2.str(), YELLOW});
            }

            if (snap.fec_gens_decoded > 0 || snap.fec_gens_dropped > 0) {
                std::ostringstream l3;
                l3 << "FEC ok " << snap.fec_gens_decoded;
                lines.push_back({l3.str(), GREEN});

                if (snap.fec_gens_dropped > 0) {
                    std::ostringstream l4;
                    l4 << "FEC  X " << snap.fec_gens_dropped;
                    lines.push_back({l4.str(), RED});
                }

                std::ostringstream l5;
                l5 << "Repair " << std::fixed << std::setprecision(1)
                   << snap.fec_repair_pct << "%";
                lines.push_back({l5.str(),
                                 color_threshold(snap.fec_repair_pct,
                                                 5.0, 15.0, false)});
            }

            std::ostringstream l6;
            l6 << "FPS    " << std::fixed << std::setprecision(1)
               << snap.video_fps_rx;
            lines.push_back({l6.str(),
                             color_threshold(snap.video_fps_rx, 25, 10, true)});

            std::ostringstream l7;
            l7 << "Link   " << fmt_kbps(snap.link_kbps_rx);
            lines.push_back({l7.str(), WHITE});

            std::ostringstream l8;
            l8 << "Video  " << fmt_kbps(snap.app_kbps_rx);
            lines.push_back({l8.str(), WHITE});
        }

        // Layout constants
        const int       font          = cv::FONT_HERSHEY_DUPLEX;
        const double    font_scale    = 0.5;
        const int       thickness     = 1;
        const int       line_height   = 18;
        const int       padding       = 8;
        const int       text_x        = padding + 4;
        const int       text_y_start  = padding + 14;

        // Compute panel dimensions
        int panel_w = 0;
        for (auto& line : lines) {
            int bl = 0;
            cv::Size sz = cv::getTextSize(line.text, font, font_scale,
                                          thickness, &bl);
            if (sz.width > panel_w) panel_w = sz.width;
        }
        panel_w += padding * 2 + 4;
        int panel_h = static_cast<int>(lines.size()) * line_height + padding * 2;

        // Clamp panel to frame bounds
        panel_w = std::min(panel_w, frame.cols - 8);
        panel_h = std::min(panel_h, frame.rows - 8);

        // Draw semi-transparent dark background
        cv::Rect panel(4, 4, panel_w, panel_h);
        if (panel.x + panel.width <= frame.cols
            && panel.y + panel.height <= frame.rows) {
            cv::Mat overlay = frame(panel).clone();
            cv::rectangle(overlay, cv::Rect(0, 0, panel_w, panel_h),
                          cv::Scalar(0, 0, 0), cv::FILLED);
            cv::addWeighted(overlay, 0.55, frame(panel), 0.45, 0, frame(panel));
            // Border
            cv::rectangle(frame, panel, cv::Scalar(80, 80, 80), 1);
        }

        // Draw text lines
        for (size_t i = 0; i < lines.size(); ++i) {
            int y = 4 + text_y_start + static_cast<int>(i) * line_height;
            cv::putText(frame, lines[i].text,
                        cv::Point(4 + text_x, y),
                        font, font_scale, lines[i].color,
                        thickness, cv::LINE_AA);
        }
    }

private:
    static cv::Scalar color_threshold(double v, double good, double warn,
                                       bool higher_better) {
        const cv::Scalar GREEN  (80,  220, 80);
        const cv::Scalar YELLOW (40,  220, 220);
        const cv::Scalar RED    (60,  60,  255);
        if (higher_better) {
            if (v >= good) return GREEN;
            if (v >= warn) return YELLOW;
            return RED;
        }
        if (v <= good) return GREEN;
        if (v <= warn) return YELLOW;
        return RED;
    }

    static std::string fmt_kbps(double k) {
        std::ostringstream o;
        if (k < 1)        o << std::fixed << std::setprecision(0)
                            << (k * 1000) << " bps";
        else if (k < 1000) o << std::fixed << std::setprecision(0)
                             << k << " kbps";
        else               o << std::fixed << std::setprecision(2)
                             << (k / 1000.0) << " Mbps";
        return o.str();
    }

    static std::string fmt_duration(double s) {
        int t = static_cast<int>(s);
        int h = t / 3600, m = (t / 60) % 60, sec = t % 60;
        std::ostringstream o;
        if (h > 0) o << h << "h";
        o << std::setw(2) << std::setfill('0') << m << ":"
          << std::setw(2) << std::setfill('0') << sec;
        return o.str();
    }

    LinkStats& m_stats;
    TuiMode    m_mode;
};

}  // namespace stats

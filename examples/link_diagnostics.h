// link_diagnostics.h — RSSI + error-rate diagnostic thread for the RX.
//
// Runs in its own thread, once per second:
//   - reads RSSI from the B200 (if the sensor exists and works)
//   - reads the current error rate from LinkStats
//   - correlates them to flag SATURATION vs WEAK-SIGNAL vs HEALTHY
//
// Why this exists: a saturated receiver and a weak-signal receiver both show
// up as "lots of CRC errors". The ONLY way to tell them apart is to look at
// the signal LEVEL at the same time:
//     high level + errors  -> SATURATION  (lower RX gain / remove LNA)
//     low level  + errors  -> WEAK SIGNAL (raise gain / closer / better aim)
//     any level  + no errs -> HEALTHY
//
// B200 RSSI caveat: on some UHD versions / with AGC on, the B200 rssi sensor
// returns a fixed, meaningless value. This class enumerates the available
// sensors at startup so you can SEE what your unit exposes, and warns if RSSI
// looks stuck. If RSSI is unreliable on your unit, the error-rate trend alone
// is still useful, and you can cross-check against the manual gain you set.

#pragma once
#include "link_stats.h"

#include <uhd/usrp/multi_usrp.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace stats {

class LinkDiagnostics {
public:
    // usrp:  the multi_usrp handle (so we can read sensors)
    // stats: the shared LinkStats the RX is already updating
    // chan:  RX channel index (0 for B200 mini)
    LinkDiagnostics(uhd::usrp::multi_usrp::sptr usrp,
                    LinkStats& stats,
                    size_t chan = 0)
        : m_usrp(usrp), m_stats(stats), m_chan(chan) {}

    void stop() { m_running.store(false); }

    // Control whether the per-second diagnostic line is printed to stderr.
    // Turn this OFF when a TUI or overlay is showing RSSI/verdict instead, so
    // the console isn't spammed (and doesn't scramble the TUI's display).
    // Default ON, so standalone use (no TUI) still logs to the console.
    void set_console_output(bool on) { m_console.store(on); }

    void run() {
        probe_sensors();   // one-time: list what's available

        // Track previous error counters so we can compute per-interval rates.
        auto prev = m_stats.sample();
        double rssi_min =  1e9, rssi_max = -1e9;
        int    rssi_stuck_count = 0;
        double last_rssi = 0;

        while (m_running.load()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            auto cur = m_stats.sample();

            // ---- error rate over this 1s interval ----
            const uint64_t d_rx  = cur.phy_packets_rx  - prev.phy_packets_rx;
            const uint64_t d_rej = cur.phy_packets_rejected
                                 - prev.phy_packets_rejected;
            const uint64_t d_drop = cur.fec_gens_dropped - prev.fec_gens_dropped;
            // "error indicator": rejected packets + dropped FEC generations
            // (rejections = bad length/magic = corrupt demod; drops = FEC
            // couldn't recover = heavy loss). Either way, trouble.
            const uint64_t errors = d_rej + d_drop;
            const uint64_t total  = d_rx + d_rej;
            const double err_pct  = (total > 0)
                ? (100.0 * errors / total) : 0.0;
            prev = cur;

            // ---- RSSI (if available) ----
            double rssi_db = 0;
            bool   have_rssi = read_rssi(rssi_db);
            if (have_rssi) {
                if (rssi_db < rssi_min) rssi_min = rssi_db;
                if (rssi_db > rssi_max) rssi_max = rssi_db;
                if (std::abs(rssi_db - last_rssi) < 0.01) rssi_stuck_count++;
                else rssi_stuck_count = 0;
                last_rssi = rssi_db;
            }

            // ---- verdict ----
            LinkVerdict v = classify(have_rssi, rssi_db, err_pct, d_rx);

            // ---- surface to LinkStats so the TUI/overlay can show it ----
            // (always — this is how the visual displays get RSSI/verdict)
            m_stats.update_rssi_dbm(rssi_db, have_rssi);
            m_stats.update_verdict(v);

            // ---- optionally print one line to stderr ----
            // Skipped when a TUI/overlay is showing the data (console off), so
            // we don't spam the terminal or scramble the TUI's cursor output.
            if (m_console.load()) {
                std::ostringstream o;
                o << "[DIAG] ";
                if (have_rssi) {
                    o << "RSSI " << std::fixed << std::setprecision(1)
                      << std::setw(7) << rssi_db << " dBm  ";
                } else {
                    o << "RSSI   n/a      ";
                }
                o << "rx/s " << std::setw(4) << d_rx << "  "
                  << "err " << std::fixed << std::setprecision(1)
                  << std::setw(5) << err_pct << "%  "
                  << "-> " << verdict_text(v, have_rssi);
                if (have_rssi && rssi_stuck_count >= 5) {
                    o << "  (warning: RSSI not changing -- sensor may be"
                         " unreliable on this unit; trust err% + your gain)";
                }
                std::cerr << o.str() << std::endl;
            }
        }
    }

private:
    // Enumerate the RX sensors this unit exposes (and optionally print them).
    void probe_sensors() {
        const bool say = m_console.load();
        if (say)
            std::cerr << "[DIAG] Probing RX sensors on channel "
                      << m_chan << "...\n";
        try {
            std::vector<std::string> names =
                m_usrp->get_rx_sensor_names(m_chan);
            if (say) {
                std::ostringstream o;
                o << "[DIAG] Available RX sensors:";
                for (auto& n : names) o << " " << n;
                std::cerr << o.str() << std::endl;
            }
            // Detect rssi (this must happen regardless of console output).
            m_have_rssi_sensor = false;
            for (auto& n : names)
                if (n == "rssi") m_have_rssi_sensor = true;
            if (!m_have_rssi_sensor && say)
                std::cerr << "[DIAG] No 'rssi' sensor on this unit/UHD. "
                             "Will report error-rate only.\n";
        } catch (const std::exception& e) {
            if (say)
                std::cerr << "[DIAG] Could not enumerate sensors: "
                          << e.what() << "\n";
            m_have_rssi_sensor = false;
        }

        if (say) {
            try {
                double g = m_usrp->get_rx_gain(m_chan);
                std::cerr << "[DIAG] Current RX gain: " << g << " dB\n";
            } catch (...) {}
        }
    }

    bool read_rssi(double& out_db) {
        if (!m_have_rssi_sensor) return false;
        try {
            uhd::sensor_value_t s = m_usrp->get_rx_sensor("rssi", m_chan);
            out_db = s.to_real();   // RSSI is a REALNUM sensor in dBm
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    // The core logic: combine signal level and error rate into a diagnosis.
    LinkVerdict classify(bool have_rssi, double rssi_db,
                         double err_pct, uint64_t rx_per_s) {
        if (rx_per_s == 0) return LinkVerdict::NoSignal;
        if (err_pct <= 5.0) return LinkVerdict::Healthy;
        // Errors are high. Use RSSI to decide which kind of trouble.
        if (have_rssi) {
            if (rssi_db > SAT_THRESHOLD_DBM)  return LinkVerdict::Saturation;
            if (rssi_db < WEAK_THRESHOLD_DBM) return LinkVerdict::Weak;
            return LinkVerdict::MidErrors;
        }
        return LinkVerdict::MidErrors;  // can't disambiguate without RSSI
    }

    // Map the verdict to the detailed line printed to stderr.
    static const char* verdict_text(LinkVerdict v, bool have_rssi) {
        switch (v) {
            case LinkVerdict::NoSignal:
                return "NO PACKETS (link down / not transmitting / wrong freq)";
            case LinkVerdict::Healthy:
                return "HEALTHY";
            case LinkVerdict::Saturation:
                return "SATURATION  (lower RX gain / remove LNA / back off)";
            case LinkVerdict::Weak:
                return "WEAK SIGNAL (raise gain / closer / aim antenna)";
            case LinkVerdict::MidErrors:
                return have_rssi
                    ? "ERRORS at mid-level (check freq match, clock, multipath)"
                    : "ERRORS (no RSSI -- if signal strong suspect saturation,"
                      " if weak suspect range)";
            default: return "...";
        }
    }

    // Tunable thresholds (dBm at the RX frontend). Adjust after observing
    // your unit's RSSI range across known-good and known-saturated cases.
    static constexpr double SAT_THRESHOLD_DBM  = -20.0;
    static constexpr double WEAK_THRESHOLD_DBM = -70.0;

    uhd::usrp::multi_usrp::sptr m_usrp;
    LinkStats& m_stats;
    size_t m_chan;
    bool m_have_rssi_sensor = false;
    std::atomic<bool> m_running{true};
    std::atomic<bool> m_console{true};   // print per-second line to stderr?
};

}  // namespace stats

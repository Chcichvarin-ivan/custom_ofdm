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
            const char* verdict = classify(have_rssi, rssi_db, err_pct, d_rx);

            // ---- print one line ----
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
              << "-> " << verdict;
            if (have_rssi && rssi_stuck_count >= 5) {
                o << "  (warning: RSSI not changing -- sensor may be"
                     " unreliable on this unit; trust err% + your gain)";
            }
            std::cerr << o.str() << std::endl;
        }
    }

private:
    // Enumerate and print the RX sensors this unit actually exposes.
    void probe_sensors() {
        std::cerr << "[DIAG] Probing RX sensors on channel "
                  << m_chan << "...\n";
        try {
            std::vector<std::string> names =
                m_usrp->get_rx_sensor_names(m_chan);
            std::ostringstream o;
            o << "[DIAG] Available RX sensors:";
            for (auto& n : names) o << " " << n;
            std::cerr << o.str() << std::endl;

            // Does rssi exist?
            m_have_rssi_sensor = false;
            for (auto& n : names)
                if (n == "rssi") m_have_rssi_sensor = true;
            if (!m_have_rssi_sensor)
                std::cerr << "[DIAG] No 'rssi' sensor on this unit/UHD. "
                             "Will report error-rate only.\n";
        } catch (const std::exception& e) {
            std::cerr << "[DIAG] Could not enumerate sensors: "
                      << e.what() << "\n";
            m_have_rssi_sensor = false;
        }

        // Also report the static gain we're running, for context.
        try {
            double g = m_usrp->get_rx_gain(m_chan);
            std::cerr << "[DIAG] Current RX gain: " << g << " dB\n";
        } catch (...) {}
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
    const char* classify(bool have_rssi, double rssi_db,
                          double err_pct, uint64_t rx_per_s) {
        // No packets arriving at all -> link is down, not just noisy.
        if (rx_per_s == 0)
            return "NO PACKETS (link down / not transmitting / wrong freq)";

        const bool errors_high = err_pct > 5.0;

        if (!errors_high)
            return "HEALTHY";

        // Errors are high. Use RSSI to decide which kind of trouble.
        if (have_rssi) {
            // These thresholds are starting points -- adjust to your unit.
            // On a B200 mini, ADC saturation tends to show up as RSSI above
            // roughly -20 dBm at the frontend. Below ~-70 dBm you're weak.
            if (rssi_db > SAT_THRESHOLD_DBM)
                return "SATURATION  (lower RX gain / remove LNA / back off)";
            if (rssi_db < WEAK_THRESHOLD_DBM)
                return "WEAK SIGNAL (raise gain / closer / aim antenna)";
            return "ERRORS at mid-level (check freq match, clock, multipath)";
        }
        // No RSSI: can't disambiguate automatically.
        return "ERRORS (no RSSI to classify -- if signal is strong, suspect"
               " saturation; if weak, suspect range)";
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
};

}  // namespace stats

/*!
 * \file link_agc.h
 * \brief Slow software AGC for the RX — keeps RSSI in a target window by
 *        nudging RX gain, on the timescale of distance changes (~1 Hz), NOT
 *        the OFDM envelope.
 *
 * WHY NOT UHD's AGC: the AD9361's hardware AGC reacts in microseconds and
 * chases the OFDM signal's ~12 dB peak-to-average envelope, which disrupts
 * frame sync and equalization — the link dies. This AGC instead adjusts gain
 * slowly (once per second, 1 dB at a time) to track how far away the drone is,
 * which never disturbs a frame.
 *
 * It is a sibling of LinkDiagnostics: same thread pattern, same multi_usrp
 * handle, reads RSSI the same way — but instead of only REPORTING the level it
 * ACTS on it by calling set_rx_gain. It owns the gain exclusively (UHD AGC must
 * be OFF and the radio in manual-gain mode, which it already is).
 *
 * Control law (level-targeting with a dead zone):
 *   - RSSI above the window (too hot)  -> lower gain (step down, faster)
 *   - RSSI below the window (too weak)  -> raise gain (step up, slower)
 *   - RSSI inside the window            -> do nothing (dead zone; prevents hunting)
 *
 * Safety rails:
 *   - Gain clamped to [min,max] (default 10..60 dB).
 *   - Anti-windup: stop stepping when already clamped.
 *   - Wide dead zone + 1 dB steps absorb the gain<->RSSI coupling (raising gain
 *     also raises reported RSSI) without oscillating.
 *   - Steps DOWN happen every tick; steps UP only every Nth tick (asymmetry:
 *     saturation hurts more than weak signal, so back off fast, ramp up gently).
 *   - If RSSI is unavailable, the AGC does nothing (never blind-adjusts).
 *
 * The current gain is written into LinkStats so the TUI/overlay can show it.
 */

#ifndef LINK_AGC_H
#define LINK_AGC_H

#include <uhd/usrp/multi_usrp.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <iomanip>

#include "link_stats.h"

namespace stats {

class LinkAgc {
public:
    // usrp : the multi_usrp handle (same one the diagnostic uses)
    // stats: to publish the current gain for display
    // chan : RX channel
    LinkAgc(uhd::usrp::multi_usrp::sptr usrp, LinkStats& stats, size_t chan = 0)
        : m_usrp(usrp), m_stats(stats), m_chan(chan) {}

    // --- tunables (set before run(); defaults match the chosen parameters) ---
    void set_window(double low_dbm, double high_dbm) {
        m_win_low = low_dbm; m_win_high = high_dbm;
    }
    void set_gain_band(double min_db, double max_db) {
        m_gain_min = min_db; m_gain_max = max_db;
    }
    void set_step_db(double step) { m_step = step; }
    void set_period_ms(int ms)    { m_period_ms = ms; }
    // Step up only every Nth tick (asymmetry). 1 = up as often as down.
    void set_up_divisor(int n)    { m_up_divisor = std::max(1, n); }
    void set_console_output(bool on) { m_console.store(on); }

    void stop() { m_running.store(false); }

    void run() {
        // Make sure we're in manual-gain mode and seed from the current gain.
        double gain = seed_gain();
        publish_gain(gain);

        if (!have_rssi_sensor()) {
            if (m_console.load())
                std::cerr << "[AGC] No 'rssi' sensor on this unit — AGC cannot "
                             "run safely without a level reading. Idle.\n";
            // Idle loop: do nothing but stay alive until stopped, so the rest
            // of the program is unaffected.
            while (m_running.load())
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            return;
        }

        if (m_console.load())
            std::cerr << "[AGC] running: target " << m_win_low << ".."
                      << m_win_high << " dBm, gain " << m_gain_min << ".."
                      << m_gain_max << " dB, " << m_step << " dB steps every "
                      << m_period_ms << " ms\n";

        int tick = 0;
        while (m_running.load()) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(m_period_ms));
            if (!m_running.load()) break;
            ++tick;

            double rssi;
            if (!read_rssi(rssi)) continue;   // no reading this tick — skip

            // ---- control law ----
            double new_gain = gain;
            const char* action = "hold";

            if (rssi > m_win_high) {
                // Too hot -> reduce gain. Do this EVERY tick (fast back-off).
                new_gain = gain - m_step;
                action = "DOWN";
            } else if (rssi < m_win_low) {
                // Too weak -> raise gain, but only every Nth tick (gentle ramp).
                if (tick % m_up_divisor == 0) {
                    new_gain = gain + m_step;
                    action = "up";
                } else {
                    action = "up(wait)";
                }
            } else {
                action = "hold";   // inside the dead zone — leave it alone
            }

            // ---- clamp + anti-windup ----
            new_gain = std::max(m_gain_min, std::min(m_gain_max, new_gain));
            bool changed = std::abs(new_gain - gain) > 1e-6;

            if (changed) {
                try {
                    m_usrp->set_rx_gain(new_gain, m_chan);
                    // Read back what the hardware actually set (it quantizes).
                    gain = m_usrp->get_rx_gain(m_chan);
                } catch (const std::exception& e) {
                    if (m_console.load())
                        std::cerr << "[AGC] set_rx_gain failed: "
                                  << e.what() << "\n";
                }
                publish_gain(gain);
            }

            if (m_console.load()) {
                std::cerr << "[AGC] RSSI " << std::fixed
                          << std::setprecision(1) << rssi << " dBm  gain "
                          << gain << " dB  [" << action << "]";
                if (gain <= m_gain_min + 1e-6) std::cerr << " (at min)";
                if (gain >= m_gain_max - 1e-6) std::cerr << " (at max)";
                std::cerr << "\n";
            }
        }
    }

private:
    double seed_gain() {
        try {
            // Ensure manual gain mode (AGC off) so we own the gain.
            // (UHD: setting a gain implicitly uses manual mode on the B200.)
            double g = m_usrp->get_rx_gain(m_chan);
            // Clamp the starting point into our band.
            g = std::max(m_gain_min, std::min(m_gain_max, g));
            m_usrp->set_rx_gain(g, m_chan);
            return m_usrp->get_rx_gain(m_chan);
        } catch (const std::exception&) {
            return m_gain_min;
        }
    }

    bool have_rssi_sensor() {
        try {
            auto names = m_usrp->get_rx_sensor_names(m_chan);
            for (auto& n : names) if (n == "rssi") return true;
        } catch (const std::exception&) {}
        return false;
    }

    bool read_rssi(double& out_db) {
        try {
            uhd::sensor_value_t s = m_usrp->get_rx_sensor("rssi", m_chan);
            out_db = s.to_real();
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    void publish_gain(double g) {
        m_stats.update_rx_gain_db(g);
    }

    uhd::usrp::multi_usrp::sptr m_usrp;
    LinkStats& m_stats;
    size_t     m_chan;

    // tunables (defaults = chosen parameters)
    double m_win_low    = -60.0;   // dBm
    double m_win_high   = -35.0;   // dBm
    double m_gain_min   = 10.0;    // dB
    double m_gain_max   = 60.0;    // dB
    double m_step       = 1.0;     // dB per adjustment
    int    m_period_ms  = 1000;    // ~1 Hz
    int    m_up_divisor = 1;       // up every tick by default (set >1 to ramp slower)

    std::atomic<bool> m_running{true};
    std::atomic<bool> m_console{true};
};

} // namespace stats

#endif // LINK_AGC_H

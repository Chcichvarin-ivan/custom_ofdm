/*!
 * \file link_agc.h
 * \brief Software AGC v2 for a MOVING link — fast attack, slow release.
 *
 * WHY v2: the v1 loop (1 dB step / 1000 ms) was designed for a stationary
 * link and cannot track motion. At 3.3 GHz the wavelength is ~9.1 cm, so a
 * moving antenna crosses multipath fades every ~4.5 cm — 15-20 fades/second
 * at walking speed — while the MEAN level changes at several dB/s from path
 * loss (1 m -> 2 m alone is 6 dB). A 1 dB/s loop falls behind immediately,
 * and when moving CLOSER the level rises into ADC clipping, which corrupts
 * every OFDM subcarrier at once (hard nonlinearity) — the link dies hard.
 *
 * THE v2 STRUCTURE (classic attack/release with clip override):
 *   - EMERGENCY ATTACK: if the clip fraction from RxPowerMonitor exceeds
 *     ~1%, gain drops 6 dB IMMEDIATELY — no dead zone, no waiting. Recovers
 *     from a fast approach within ~100-200 ms instead of tens of seconds.
 *   - PROPORTIONAL DOWN: mean level above the window -> step down by up to
 *     3 dB per tick, proportional to the excess (up to ~30 dB/s).
 *   - CONDITIONED SLOW UP (release): gain rises only after the level has
 *     been below the window for 3 CONSECUTIVE ticks (~300 ms sustained),
 *     then +1 dB/tick (~10 dB/s). A 50 ms multipath null can never
 *     accumulate 3 low ticks, so fades do not pump the gain — but walking
 *     away (a sustained trend) is tracked.
 *   - WIDE DEAD ZONE: the window is ~14 dB wide, so ordinary fading wiggle
 *     around the center causes NO gain changes at all. The AGC's job is to
 *     keep the MEAN centered; the window width + FEC absorb the fading.
 *   - SETTLE TICK: after any change, one tick is skipped so the loop never
 *     reacts to its own transition.
 *
 * WHAT THIS DOES NOT DO (by design): it does not chase per-symbol OFDM
 * envelope (the hardware AGC's fatal mistake) and it does not try to flatten
 * multipath fading — no gain loop can. Fading losses are the job of FEC
 * (R=16) and, if bursts exceed the FEC budget, the TX interleaver.
 *
 * Level source: RxPowerMonitor (sample power published by frame_detector),
 * in dBFS. No hardware RSSI sensor is used anywhere. Gain is still applied
 * via set_rx_gain on the shared usrp handle; the current gain is published
 * to LinkStats for the TUI/overlay readout.
 *
 * Cost note: a gain step mid-packet corrupts that packet (equalizer channel
 * estimate mismatch). Worst-case active slewing costs a few % packet loss,
 * absorbed by FEC; in the dead zone (steady state) there are no steps and
 * no cost.
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
#include "rx_power_monitor.h"   // software level source (mean dBFS + clip fraction)

namespace stats {

class LinkAgc {
public:
    LinkAgc(uhd::usrp::multi_usrp::sptr usrp, LinkStats& stats, size_t chan = 0)
        : m_usrp(usrp), m_stats(stats), m_chan(chan) {}

    // --- tunables (set before run(); defaults sized for the moving link) ---
    // Window on the MEAN level, in dBFS. Wide on purpose: fading wiggle
    // inside it causes no steps. With OFDM's ~11 dB peak-to-average, a mean
    // at the top (-14) puts peaks near -3 dBFS — where the clip path engages.
    void set_window(double low_dbfs, double high_dbfs) {
        m_win_low = low_dbfs; m_win_high = high_dbfs;
    }
    void set_gain_band(double min_db, double max_db) {
        m_gain_min = min_db; m_gain_max = max_db;
    }
    void set_period_ms(int ms)          { m_period_ms = ms; }
    void set_clip_threshold(double f)   { m_clip_thresh = f; }       // fraction
    void set_attack_step_db(double db)  { m_attack_step = db; }      // clip drop
    void set_max_down_db(double db)     { m_max_down = db; }         // per tick
    void set_up_step_db(double db)      { m_up_step = db; }          // per tick
    void set_up_hold_ticks(int n)       { m_up_hold = std::max(1, n); }
    void set_console_output(bool on)    { m_console.store(on); }

    void stop() { m_running.store(false); }

    void run() {
        double gain = seed_gain();
        publish_gain(gain);

        if (m_console.load())
            std::cerr << "[AGC v2] attack/release AGC: window " << m_win_low
                      << ".." << m_win_high << " dBFS, gain " << m_gain_min
                      << ".." << m_gain_max << " dB, tick " << m_period_ms
                      << " ms, clip>" << (m_clip_thresh * 100.0)
                      << "% => -" << m_attack_step << " dB\n";

        int  low_ticks = 0;       // consecutive below-window ticks (release gate)
        bool settle    = false;   // skip one decision after any gain change
        bool warned_no_data = false;

        while (m_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(m_period_ms));
            if (!m_running.load()) break;

            double level_dbfs;
            if (!read_level_dbfs(level_dbfs)) {
                if (m_console.load() && !warned_no_data) {
                    std::cerr << "[AGC v2] waiting for received samples...\n";
                    warned_no_data = true;
                }
                continue;
            }
            warned_no_data = false;
            double clip = fun::RxPowerMonitor::clip_fraction();

            if (settle) {           // one quiet tick after a change
                settle = false;
                if (m_console.load()) log_line(level_dbfs, clip, gain, "settle");
                continue;
            }

            double new_gain = gain;
            const char* action = "hold";

            if (clip > m_clip_thresh) {
                // EMERGENCY ATTACK: clipping corrupts everything — get out now.
                new_gain = gain - m_attack_step;
                action = "CLIP!";
                low_ticks = 0;
            } else if (level_dbfs > m_win_high) {
                // Proportional down: bigger excess -> bigger step (capped).
                double over = level_dbfs - m_win_high;
                new_gain = gain - std::min(m_max_down, over);
                action = "down";
                low_ticks = 0;
            } else if (level_dbfs < m_win_low) {
                // Release: only after SUSTAINED weakness, so multipath nulls
                // (tens of ms) never pump the gain up.
                ++low_ticks;
                if (low_ticks >= m_up_hold) {
                    new_gain = gain + m_up_step;
                    action = "up";
                } else {
                    action = "low(wait)";
                }
            } else {
                low_ticks = 0;      // inside the dead zone — do nothing
            }

            new_gain = std::max(m_gain_min, std::min(m_gain_max, new_gain));
            if (std::abs(new_gain - gain) > 1e-6) {
                try {
                    m_usrp->set_rx_gain(new_gain, m_chan);
                    gain = m_usrp->get_rx_gain(m_chan);   // read back (quantized)
                } catch (const std::exception& e) {
                    if (m_console.load())
                        std::cerr << "[AGC v2] set_rx_gain failed: "
                                  << e.what() << "\n";
                }
                publish_gain(gain);
                settle = true;      // don't react to our own transition
            }

            if (m_console.load()) {
                log_line(level_dbfs, clip, gain, action);
                if (gain <= m_gain_min + 1e-6) std::cerr << "        (at min)\n";
                if (gain >= m_gain_max - 1e-6) std::cerr << "        (at max)\n";
            }
        }
    }

private:
    double seed_gain() {
        try {
            double g = m_usrp->get_rx_gain(m_chan);
            g = std::max(m_gain_min, std::min(m_gain_max, g));
            m_usrp->set_rx_gain(g, m_chan);
            return m_usrp->get_rx_gain(m_chan);
        } catch (const std::exception&) {
            return m_gain_min;
        }
    }

    bool read_level_dbfs(double& out_dbfs) {
        double p = fun::RxPowerMonitor::level_linear();
        if (p <= 0.0) return false;             // no reading yet
        if (p < 1e-12) p = 1e-12;               // -120 dBFS guard
        out_dbfs = 10.0 * std::log10(p);        // fc64 full scale => 0 dBFS
        return true;
    }

    void log_line(double lvl, double clip, double gain, const char* action) {
        std::cerr << "[AGC v2] level " << std::fixed << std::setprecision(1)
                  << lvl << " dBFS  clip " << std::setprecision(1)
                  << (clip * 100.0) << "%  gain " << std::setprecision(1)
                  << gain << " dB  [" << action << "]\n";
    }

    void publish_gain(double g) { m_stats.update_rx_gain_db(g); }

    uhd::usrp::multi_usrp::sptr m_usrp;
    LinkStats& m_stats;
    size_t     m_chan;

    // tunables — defaults sized for the moving-drone case
    double m_win_low     = -28.0;   // dBFS (release below this, if sustained)
    double m_win_high    = -14.0;   // dBFS (proportional down above this)
    double m_gain_min    = 10.0;    // dB
    double m_gain_max    = 70.0;    // dB
    int    m_period_ms   = 100;     // 10 Hz tick
    double m_clip_thresh = 0.01;    // >1% near-clip samples => emergency
    double m_attack_step = 6.0;     // dB, immediate, on clip
    double m_max_down    = 3.0;     // dB per tick, proportional path
    double m_up_step     = 1.0;     // dB per tick, release path
    int    m_up_hold     = 3;       // consecutive low ticks before releasing up

    std::atomic<bool> m_running{true};
    std::atomic<bool> m_console{true};
};

} // namespace stats

#endif // LINK_AGC_H

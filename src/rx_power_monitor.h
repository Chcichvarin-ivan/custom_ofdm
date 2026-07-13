/*!
 * \file rx_power_monitor.h
 * \brief Shared receive-level statistics for the software AGC (v2).
 *
 * Published by frame_detector from the raw IQ samples it already processes;
 * read by the AGC thread. No hardware sensor involved.
 *
 * TWO statistics, because the AGC needs different speeds in each direction:
 *   - level_linear(): EMA of mean |sample|^2. The SLOW view — used for the
 *     "raise gain / lower gain a bit" decisions. dBFS = 10*log10(level).
 *     fun_ofdm streams fc64 normalized to +/-1.0, so full scale ~ 0 dBFS,
 *     -20 dBFS ~ 0.01 linear.
 *   - clip_fraction(): fast-EMA of the fraction of samples whose instantaneous
 *     power is near full scale (> CLIP_LIN, ~ -3 dBFS). The FAST view — used
 *     for the emergency "we are clipping, slam gain down NOW" path. Clipping
 *     is a hard nonlinearity that corrupts every OFDM subcarrier at once, so
 *     it must be reacted to much faster than ordinary level drift.
 *
 * Thread-safety: two process-wide atomics (function-local statics, so exactly
 * one instance even though header-only). frame_detector (RX thread) is the
 * sole writer; the AGC thread is the sole reader. Relaxed ordering is fine.
 */

#ifndef RX_POWER_MONITOR_H
#define RX_POWER_MONITOR_H

#include <atomic>

namespace fun
{
    class RxPowerMonitor
    {
    public:
        //! Instantaneous |s|^2 above this counts as "near clip" (~ -3 dBFS).
        static constexpr double CLIP_LIN = 0.5;

        //! Publish one batch's statistics (called by frame_detector).
        //!   batch_mean_power : mean |sample|^2 over the batch (linear)
        //!   batch_clip_frac  : fraction of the batch's samples above CLIP_LIN
        static void publish(double batch_mean_power, double batch_clip_frac)
        {
            // Mean level: slow EMA. Batches arrive at 100s of Hz, so
            // alpha=0.05 gives a time constant of a few tens of ms — enough
            // smoothing to ride over individual multipath fades.
            double prev = mean_atomic().load(std::memory_order_relaxed);
            double next = (prev <= 0.0)
                            ? batch_mean_power                        // seed
                            : prev * 0.95 + batch_mean_power * 0.05;  // EMA
            mean_atomic().store(next, std::memory_order_relaxed);

            // Clip fraction: FAST EMA (reacts within a few batches, decays
            // within a few batches) so the AGC's emergency path sees clipping
            // almost immediately and stands down almost immediately after.
            double cprev = clip_atomic().load(std::memory_order_relaxed);
            double cnext = cprev * 0.7 + batch_clip_frac * 0.3;
            clip_atomic().store(cnext, std::memory_order_relaxed);
        }

        //! Smoothed mean power, LINEAR (|sample|^2). 0.0 = no data yet.
        static double level_linear()
        {
            return mean_atomic().load(std::memory_order_relaxed);
        }

        //! Recent fraction of near-clip samples, 0.0 .. 1.0.
        static double clip_fraction()
        {
            return clip_atomic().load(std::memory_order_relaxed);
        }

    private:
        static std::atomic<double>& mean_atomic()
        {
            static std::atomic<double> v{0.0};
            return v;
        }
        static std::atomic<double>& clip_atomic()
        {
            static std::atomic<double> v{0.0};
            return v;
        }
    };

} // namespace fun

#endif // RX_POWER_MONITOR_H

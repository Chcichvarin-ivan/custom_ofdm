/*! \file timing_sync.h
 *  \brief Header file for the Timing Sync block.
 *
 * This block uses the two LTS symbols to align the received frame in time,
 * estimates the carrier frequency offset (CFO) from them, and applies the
 * correction to all subsequent samples.
 *
 * CFO FIX (three parts, see timing_sync.cpp for details):
 *  1) The upstream CFO estimation loop was dead code (it never executed), so
 *     no frequency correction was ever applied. The estimate is now computed
 *     correctly from the two LTS symbols (Schmidl & Cox style).
 *  2) The estimate is NOT learned from clipped preambles (clip fraction from
 *     RxPowerMonitor > 2%): a clipped LTS still passes the 0.9 detection
 *     threshold but its phase is garbage, and a persistent garbage estimate
 *     would corrupt every following frame.
 *  3) A watchdog resets the correction to neutral if no LTS has been seen
 *     for RESET_AFTER_SAMPLES while samples keep flowing, so a stale or bad
 *     estimate can never outlive the signal that produced it.
 */

#ifndef TIMING_SYNC_H
#define TIMING_SYNC_H

#define LTS_CORR_THRESHOLD 0.9
#define CARRYOVER_LENGTH 160
#define LTS_LENGTH 64

//! CFO watchdog: if no LTS is detected for this many samples, the persistent
//! frequency correction (m_phase_offset / m_phase_acc) resets to neutral.
//! 2,500,000 samples = 250 ms at 10 Msps (125 ms at 20 Msps).
#define RESET_AFTER_SAMPLES 2500000ULL

#include <complex>

#include "block.h"
#include "tagged_vector.h"

namespace fun
{
    /*!
     * \brief The timing_sync block.
     *
     * Inputs tagged_samples from the frame_detector block.
     * Outputs tagged_samples to the fft_symbols block.
     *
     * Uses the two LTS symbols to align the frame in time, performs the
     * initial frequency-offset estimation, and applies the correction.
     */
    class timing_sync : public fun::block<tagged_sample, tagged_sample>
    {
    public:
        timing_sync(); //!< Constructor for timing_sync block.
        virtual void work(); //!< Signal processing happens here.

    private:
        double m_phase_offset; //!< The phase rotation from sample to sample
        double m_phase_acc;    //!< The total phase rotation for the current sample
        std::vector<tagged_sample> m_carryover; //!< Samples carried over between calls

        //! Samples processed since the last LTS detection (CFO watchdog).
        unsigned long long m_samples_since_lts;
    };
}

#endif // TIMING_SYNC_H

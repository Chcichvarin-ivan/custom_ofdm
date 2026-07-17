/*!
 * \file cfo_selftest.cpp
 * \brief Self-test for the timing_sync CFO fix (estimate + clip gate + watchdog).
 *
 * Drives the timing_sync block directly (its buffers are public) with
 * synthetic signals built from the real LTS table in preamble.h:
 *
 *   [noise][ 32-sample CP | LTS | LTS ][ constant carrier ... ]
 *
 * everything rotated by e^{+j w n} to simulate a carrier frequency offset.
 * The STS_END tag is placed 12 samples into the CP (mimicking real detector
 * lag) so both LTS correlation peaks land inside the search window.
 *
 * Pipeline note: the block delays the stream by CARRYOVER_LENGTH (160)
 * samples — concatenated output index i corresponds to fed sample i-160.
 * All measurements below account for that shift.
 *
 * T1  known offset w=0.01 rad/sample: after the preamble, the corrected
 *     carrier must be rotation-free (residual < 1e-3 rad/sample vs w).
 * T2  clip gate: with RxPowerMonitor clip fraction published at 20%, the
 *     same preamble must NOT be learned — output still rotates at ~w.
 *     (Monitor is a process-wide static: T2 cleans it afterwards with
 *     clean publishes so later tests see clip ~ 0.)
 * T3  watchdog: learn a nonzero offset, then feed > RESET_AFTER_SAMPLES of
 *     untagged near-silence; the stale correction must reset — a subsequent
 *     unrotated carrier passes through with < 1e-4 rad/sample rotation.
 *
 * Build (from fun_ofdm/src, with rx_power_monitor.h v2 present):
 *   g++ -std=c++14 -I. cfo_selftest.cpp timing_sync.cpp -o cfo_selftest && ./cfo_selftest
 */

#include "timing_sync.h"
#include "rx_power_monitor.h"
#include "preamble.h"

#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

using namespace fun;

static int fails = 0;
#define CHECK(c, m) do { bool chk_ = (c); \
    std::printf("    %-62s %s\n", m, chk_ ? "PASS" : "*** FAIL ***"); \
    if (!chk_) ++fails; } while (0)

// ---------------------------------------------------------------------------
// Signal construction
// ---------------------------------------------------------------------------
static std::vector<std::complex<double>> lts_samples() {
    std::vector<std::complex<double>> l(64);
    for (int k = 0; k < 64; ++k) l[k] = std::conj(LTS_TIME_DOMAIN_CONJ[k]);
    return l;
}

// Build one continuous stream: quiet lead-in, long preamble (CP+LTS+LTS) at
// position P, then a constant unit carrier, all rotated by e^{j*omega*n}.
// Tag STS_END at P+12 (12 samples into the CP, like a lagging detector).
static std::vector<tagged_sample> build_stream(size_t total, size_t P,
                                               double omega,
                                               bool with_preamble = true) {
    auto lts = lts_samples();
    std::vector<tagged_sample> s(total);
    for (size_t n = 0; n < total; ++n) {
        std::complex<double> base(1e-4, 0.0);        // quiet floor
        if (with_preamble) {
            if (n >= P && n < P + 32)                 // CP = last 32 of LTS
                base = lts[32 + (n - P)];
            else if (n >= P + 32 && n < P + 96)       // LTS #1
                base = lts[n - (P + 32)];
            else if (n >= P + 96 && n < P + 160)      // LTS #2
                base = lts[n - (P + 96)];
            else if (n >= P + 160)                    // payload carrier
                base = std::complex<double>(1.0, 0.0);
        } else {
            base = std::complex<double>(1.0, 0.0);    // pure carrier
        }
        double ph = omega * static_cast<double>(n);
        s[n].sample = base * std::complex<double>(std::cos(ph), std::sin(ph));
        s[n].tag = NONE;
    }
    if (with_preamble) s[P + 12].tag = STS_END;
    return s;
}

// Feed a stream through the block in fixed batches; return concatenated output.
static std::vector<std::complex<double>> run(timing_sync& ts,
                                             const std::vector<tagged_sample>& in,
                                             size_t batch = 8192) {
    std::vector<std::complex<double>> out;
    out.reserve(in.size());
    for (size_t off = 0; off < in.size(); off += batch) {
        size_t n = std::min(batch, in.size() - off);
        if (n <= CARRYOVER_LENGTH) break;             // block requires > 160
        ts.input_buffer.assign(in.begin() + off, in.begin() + off + n);
        ts.work();
        for (auto& t : ts.output_buffer) out.push_back(t.sample);
        ts.input_buffer.clear();
    }
    return out;
}

// Mean per-sample phase increment of a (should-be) constant carrier region.
// Output index = fed index + 160 (pipeline shift).
static double residual_rotation(const std::vector<std::complex<double>>& out,
                                size_t fed_from, size_t fed_to) {
    size_t a = fed_from + CARRYOVER_LENGTH;
    size_t b = std::min(fed_to + CARRYOVER_LENGTH, out.size() - 1);
    double acc = 0; size_t cnt = 0;
    for (size_t i = a; i + 1 < b; ++i) {
        std::complex<double> d = out[i + 1] * std::conj(out[i]);
        if (std::abs(d) < 1e-9) continue;
        acc += std::arg(d); ++cnt;
    }
    return cnt ? acc / cnt : 0.0;
}

int main() {
    std::printf("=== timing_sync CFO fix self-test ===\n\n");
    const size_t P = 1000;                 // preamble position
    const size_t MEAS_FROM = P + 400;      // measure well past the preamble
    const size_t MEAS_TO   = P + 6000;

    // ------------------------------------------------------------------
    std::printf("T1: known CFO w=0.01 rad/sample is estimated and corrected\n");
    {
        // Monitor is a process-wide static with no reset: make sure it reads
        // clean BEFORE the first learn (T1 must run before T2 dirties it).
        for (int i = 0; i < 60; ++i) RxPowerMonitor::publish(0.05, 0.0);

        const double w = 0.01;
        timing_sync ts;
        auto out = run(ts, build_stream(16384, P, w));
        double before = w;                              // uncorrected rotation
        double after  = residual_rotation(out, MEAS_FROM, MEAS_TO);
        std::printf("      rotation: uncorrected %.5f -> corrected %.6f rad/sample\n",
                    before, after);
        CHECK(std::fabs(after) < 1e-3, "residual rotation < 1e-3 rad/sample");
        CHECK(std::fabs(after) < std::fabs(before) / 50.0,
              "offset reduced by > 50x");
    }

    // ------------------------------------------------------------------
    std::printf("\nT2: clip gate — clipped preamble is NOT learned\n");
    {
        // Publish a heavy clip fraction (fast EMA reaches ~20% quickly).
        for (int i = 0; i < 20; ++i) RxPowerMonitor::publish(0.6, 0.20);

        const double w = 0.02;
        timing_sync ts;
        auto out = run(ts, build_stream(16384, P, w));
        double after = residual_rotation(out, MEAS_FROM, MEAS_TO);
        std::printf("      rotation with gate closed: %.5f (w=%.5f)\n", after, w);
        CHECK(std::fabs(after - w) < 1e-3,
              "output still rotates at ~w: estimate was not learned");

        // Clean the shared monitor for later tests (0.2 * 0.7^60 ~ 0).
        for (int i = 0; i < 60; ++i) RxPowerMonitor::publish(0.05, 0.0);
        CHECK(RxPowerMonitor::clip_fraction() < 0.005,
              "monitor cleaned for subsequent tests");
    }

    // ------------------------------------------------------------------
    std::printf("\nT3: watchdog — stale correction resets after silence\n");
    {
        const double w = 0.015;
        timing_sync ts;

        // Learn a real offset from a clean preamble...
        auto out1 = run(ts, build_stream(16384, P, w));
        double locked = residual_rotation(out1, MEAS_FROM, MEAS_TO);
        CHECK(std::fabs(locked) < 1e-3, "offset locked from clean preamble");

        // ...then > RESET_AFTER_SAMPLES of untagged near-silence.
        std::vector<tagged_sample> quiet(8192);
        for (auto& t : quiet) { t.sample = {1e-4, 0.0}; t.tag = NONE; }
        const size_t batches =
            static_cast<size_t>(RESET_AFTER_SAMPLES / 8192) + 3;
        for (size_t i = 0; i < batches; ++i) {
            ts.input_buffer = quiet;
            ts.work();
            ts.input_buffer.clear();
        }

        // A clean UNrotated carrier must now pass through unrotated: if the
        // stale -0.015 correction survived, we would measure ~ -0.015 here.
        auto out2 = run(ts, build_stream(16384, 0, 0.0, /*preamble=*/false));
        double after = residual_rotation(out2, 2000, 12000);
        std::printf("      rotation after watchdog reset: %.6f rad/sample\n", after);
        CHECK(std::fabs(after) < 1e-4, "stale correction cleared (< 1e-4)");
    }

    std::printf("\n=== %s (%d failure%s) ===\n",
                fails == 0 ? "ALL CFO TESTS PASSED" : "FAILURES",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}

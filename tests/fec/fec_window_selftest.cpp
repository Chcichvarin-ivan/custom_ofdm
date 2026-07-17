/*!
 * \file fec_window_selftest.cpp
 * \brief Self-test for the sync-hardened FEC decoder window logic.
 *
 * Uses a hermetic RaptorQ stub (stub/RaptorQ/RaptorQ_v1_hdr.hpp) whose
 * decode never succeeds — deliberate, because these tests exercise the
 * generation-window SYNC behavior, not the FEC math. Packets are built with
 * the real WireHeader layout via struct memcpy, exactly mirroring what the
 * decoder does on parse, so the tests are layout-agnostic.
 *
 * Build:  g++ -std=c++14 -pthread -Istub -I. fec_window_selftest.cpp -o t && ./t
 *
 * T1  normal flow: baseline advances, nothing resyncs
 * T2  single wild jump (+16384, a flipped gen_id bit): baseline UNCHANGED,
 *     packet not ingested  <-- the regression test for the field wedge
 * T3  three corroborating far packets: window follows the (real) jump
 * T4  wedge-heal: sustained "old" traffic after a bad jump -> watchdog
 *     resync -> acceptance resumes without a process restart
 * T5  TX restart (ids back to 0): heals the same way
 * T6  exact +/-32768 int16 edge: no crash, baseline unmoved
 */

#include "fec_decoder.h"

#include <cstdio>
#include <cstring>
#include <vector>

static int fails = 0;
#define CHECK(c, m) do { bool chk_ = (c); \
    std::printf("    %-62s %s\n", m, chk_ ? "PASS" : "*** FAIL ***"); \
    if (!chk_) ++fails; } while (0)

// Build a PHY packet exactly the way the decoder parses one: fill the real
// WireHeader struct and memcpy HEADER_BYTES of it to the wire.
static std::vector<unsigned char> mk_pkt(uint16_t gen, uint32_t esi) {
    std::vector<unsigned char> p(fec::PHY_PACKET_BYTES, 0);
    fec::WireHeader h;
    std::memset(&h, 0, sizeof(h));
    h.generation_id_be = fec::hton16(gen);
    h.esi_be           = fec::hton32(esi);
    h.k_be             = fec::hton16(fec::SOURCE_SYMBOLS_PER_GEN);
    std::memcpy(p.data(), &h, fec::HEADER_BYTES);
    return p;
}

static void feed_gen(fec::FecDecoder& d, uint16_t gen, int n = 48) {
    for (int e = 0; e < n; ++e) d.process_phy_packet(mk_pkt(gen, e));
}

int main() {
    std::printf("=== FEC window sync self-test (hermetic, stub RaptorQ) ===\n\n");
    fec::FecDecoder d;
    stats::LinkStats st;
    d.set_stats(&st);

    // ------------------------------------------------------------------
    std::printf("T1: normal flow — baseline tracks traffic, no resyncs\n");
    for (uint16_t g = 0; g <= 9; ++g) feed_gen(d, g);
    CHECK(d.sync_acquired(), "baseline acquired");
    CHECK(d.sync_baseline() == 6, "baseline == 9 - (WINDOW-1) == 6");
    CHECK(d.gens_in_flight() == 4, "exactly WINDOW generations in flight");
    CHECK(d.resync_count() == 0, "no watchdog resyncs in normal flow");
    CHECK(d.jump_accepts() == 0, "no large jumps believed in normal flow");

    // ------------------------------------------------------------------
    std::printf("\nT2: single corrupted gen_id (+16384) — the field wedge\n");
    {
        const uint16_t wild = static_cast<uint16_t>(10 + 16384);
        const uint16_t base_before = d.sync_baseline();
        const size_t   gens_before = d.gens_in_flight();
        d.process_phy_packet(mk_pkt(wild, 0));
        CHECK(d.sync_baseline() == base_before,
              "baseline UNCHANGED by a lone wild id (old code: wedged here)");
        CHECK(d.gens_in_flight() == gens_before,
              "outlier not ingested — no decoder-state pollution");
        feed_gen(d, 10);
        CHECK(d.sync_baseline() == 7, "real traffic continues unharmed");
        CHECK(d.resync_count() == 0, "no resync needed — wedge never happened");
    }

    // ------------------------------------------------------------------
    std::printf("\nT3: corroborated jump — three agreeing packets move the window\n");
    {
        d.process_phy_packet(mk_pkt(20000, 0));
        d.process_phy_packet(mk_pkt(20000, 1));
        CHECK(d.jump_accepts() == 0, "two votes are not yet enough");
        d.process_phy_packet(mk_pkt(20001, 0));   // 3rd, within +/-WINDOW
        CHECK(d.jump_accepts() == 1, "third corroborating packet confirms");
        CHECK(d.sync_baseline() == static_cast<uint16_t>(20001 - 3),
              "baseline moved to the confirmed region");
        CHECK(d.gens_in_flight() == 1, "old gens evicted, confirmed gen ingested");
        feed_gen(d, 20002);
        CHECK(d.sync_baseline() == static_cast<uint16_t>(20002 - 3),
              "traffic flows normally at the new region");
    }

    // ------------------------------------------------------------------
    std::printf("\nT4: wedge-heal — sustained real traffic below a bad baseline\n");
    {
        // The window now sits at ~20000 (pretend that jump was garbage that
        // got corroborated by bad luck). REAL traffic is at gens 11, 12, ...
        // Every packet reads "old" -> after REACQUIRE_AFTER_OLD drops the
        // watchdog must re-seed and acceptance must resume BY ITSELF.
        const uint64_t resyncs_before = d.resync_count();
        uint16_t g = 11;
        int fed = 0;
        for (int i = 0; i < 400; ++i) {          // > REACQUIRE_AFTER_OLD
            d.process_phy_packet(mk_pkt(g, i % 48));
            if (++fed % 48 == 0) ++g;
        }
        CHECK(d.resync_count() == resyncs_before + 1,
              "watchdog fired exactly once");
        CHECK(d.sync_acquired(), "baseline re-seeded from live traffic");
        const int16_t dist =
            static_cast<int16_t>(d.sync_baseline() - 11);
        CHECK(dist >= 0 && dist <= 12,
              "new baseline is at the REAL traffic region");
        CHECK(d.gens_in_flight() >= 1, "packets are being accepted again");
    }

    // ------------------------------------------------------------------
    std::printf("\nT5: TX restart (ids back to 0) — heals the same way\n");
    {
        const uint64_t resyncs_before = d.resync_count();
        uint16_t g = 0;
        int fed = 0;
        for (int i = 0; i < 400; ++i) {
            d.process_phy_packet(mk_pkt(g, i % 48));
            if (++fed % 48 == 0) ++g;
        }
        CHECK(d.resync_count() == resyncs_before + 1,
              "watchdog fired once for the restart");
        const int16_t dist = static_cast<int16_t>(d.sync_baseline() - 0);
        CHECK(dist >= 0 && dist <= 12,
              "baseline re-acquired at the restarted TX's ids");
        CHECK(d.gens_in_flight() >= 1, "restarted stream is being decoded");
    }

    // ------------------------------------------------------------------
    std::printf("\nT6: exact +/-32768 wraparound edge — no crash, no move\n");
    {
        const uint16_t base_before = d.sync_baseline();
        const size_t   gens_before = d.gens_in_flight();
        d.process_phy_packet(
            mk_pkt(static_cast<uint16_t>(base_before + 32768u), 0));
        CHECK(d.sync_baseline() == base_before, "baseline unmoved by the edge id");
        CHECK(d.gens_in_flight() == gens_before, "edge id not ingested");
    }

    std::printf("\n=== %s (%d failure%s) ===\n",
                fails == 0 ? "ALL WINDOW-SYNC TESTS PASSED" : "FAILURES",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}

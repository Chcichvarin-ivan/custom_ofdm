#include "phy_spreader.h"
#include <cstdio>
#include <map>
#include <set>
#include <vector>

// Build a fake FEC phy packet with the REAL wire layout: uint16 gen id BE at
// offset 0 (fec_common.h WireHeader). esi stored in byte[2] for test tracking.
static std::vector<unsigned char> mk(uint16_t gen, uint8_t esi) {
    std::vector<unsigned char> p(10, 0);
    p[0] = (unsigned char)(gen >> 8); p[1] = (unsigned char)(gen & 0xFF);
    p[2] = esi;
    return p;
}
static uint16_t gen_of(const std::vector<unsigned char>& p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}

static int fails = 0;
#define CHECK(c, m) do{ bool chk_=(c); printf("  %-62s %s\n", m, chk_?"PASS":"*** FAIL ***"); if(!chk_) ++fails; }while(0)

int main() {
    const int R = 16;   // current repair budget per generation

    // ---- T1: FULL generations, depth 4: adjacency + burst spreading ----
    printf("T1: 4 full generations (48 pkts each), depth 4\n");
    {
        PhySpreader sp(4);
        for (uint16_t g = 0; g < 4; ++g)
            for (int e = 0; e < 48; ++e) sp.push(mk(g, e));
        sp.push(mk(4, 0));                 // 5th gen's first pkt triggers emission
        std::vector<uint16_t> order;
        std::vector<unsigned char> q;
        while (sp.pop(q)) order.push_back(gen_of(q));
        CHECK(order.size() == 4*48, "all 192 packets emitted, none lost");
        int same_adjacent = 0;
        for (size_t i = 1; i < order.size(); ++i)
            if (order[i] == order[i-1]) ++same_adjacent;
        CHECK(same_adjacent == 0, "no two consecutive on-air packets share a generation");
        // burst of 40 consecutive lost -> per-gen loss must be <= R
        std::map<uint16_t,int> lost;
        for (size_t i = 60; i < 100; ++i) lost[order[i]]++;
        bool ok = true; for (auto& kv : lost) if (kv.second > R) ok = false;
        CHECK(ok, "40-packet burst: every generation loses <= R=16 (recoverable)");
    }

    // ---- T2: PARTIAL generations (flush-closed), varying sizes — the v1 killer ----
    printf("T2: partial generations of sizes 20/48/31/48, depth 4\n");
    {
        PhySpreader sp(4);
        int sizes[4] = {20, 48, 31, 48};
        for (uint16_t g = 0; g < 4; ++g)
            for (int e = 0; e < sizes[g]; ++e) sp.push(mk(g, e));
        sp.push(mk(4, 0));
        std::vector<uint16_t> order;
        std::vector<unsigned char> q;
        while (sp.pop(q)) order.push_back(gen_of(q));
        CHECK((int)order.size() == 20+48+31+48, "all packets of ragged generations emitted");
        // rows keyed by ID: each generation's packets stay grouped per row.
        // Interleaving property: within the first 4*20 outputs (all rows alive)
        // there must be no same-gen adjacency.
        int same_adj_head = 0;
        for (int i = 1; i < 80; ++i) if (order[i] == order[i-1]) ++same_adj_head;
        CHECK(same_adj_head == 0, "no same-gen adjacency while all rows contribute");
        // a 30-pkt burst early on spreads across all four gens
        std::map<uint16_t,int> lost;
        for (int i = 10; i < 40; ++i) lost[order[i]]++;
        bool ok = true; for (auto& kv : lost) if (kv.second > R) ok = false;
        CHECK(ok, "30-packet burst on ragged block: every gen loses <= R");
    }

    // ---- T3: idle flush with fewer than depth generations buffered ----
    printf("T3: idle flush_partial() with only 2 of depth-4 generations buffered\n");
    {
        PhySpreader sp(4);
        for (int e = 0; e < 25; ++e) sp.push(mk(0, e));
        for (int e = 0; e < 18; ++e) sp.push(mk(1, e));
        std::vector<unsigned char> q;
        CHECK(!sp.pop(q), "nothing emitted before flush (block not full)");
        sp.flush_partial();
        std::vector<uint16_t> order;
        while (sp.pop(q)) order.push_back(gen_of(q));
        CHECK(order.size() == 43, "flush emits all buffered packets");
        int same_adj = 0;
        for (int i = 1; i < 36; ++i) if (order[i] == order[i-1]) ++same_adj;
        CHECK(same_adj == 0, "flushed remainder still interleaved over 2 gens");
        CHECK(sp.buffered_generations() == 0, "buffer empty after flush");
    }

    // ---- T4: integrity — nothing duplicated, per-gen counts preserved ----
    printf("T4: packet-count integrity across a long mixed run\n");
    {
        PhySpreader sp(3);
        std::map<uint16_t,int> in_counts, out_counts;
        int sizes[7] = {48, 12, 48, 33, 48, 7, 48};
        for (uint16_t g = 0; g < 7; ++g) {
            in_counts[g] = sizes[g];
            for (int e = 0; e < sizes[g]; ++e) sp.push(mk(g, e));
        }
        sp.flush_partial();
        std::vector<unsigned char> q;
        while (sp.pop(q)) out_counts[gen_of(q)]++;
        CHECK(in_counts == out_counts, "every generation's packet count in == out");
    }

    // ---- T5: depth 2 (the recommended start) burst math ----
    printf("T5: depth 2, burst of 30 consecutive lost\n");
    {
        PhySpreader sp(2);
        for (uint16_t g = 0; g < 2; ++g)
            for (int e = 0; e < 48; ++e) sp.push(mk(g, e));
        sp.push(mk(2, 0));
        std::vector<uint16_t> order;
        std::vector<unsigned char> q;
        while (sp.pop(q)) order.push_back(gen_of(q));
        std::map<uint16_t,int> lost;
        for (int i = 20; i < 50; ++i) lost[order[i]]++;
        bool ok = true;
        for (auto& kv : lost) { ok = ok && kv.second <= R;
            printf("    gen %u loses %d/48 (R=%d) %s\n", kv.first, kv.second, R,
                   kv.second <= R ? "recoverable" : "LOST"); }
        CHECK(ok, "depth 2 spreads a 30-burst to <= R per generation");
    }

    printf("\n%s (%d failure%s)\n", fails==0 ? "ALL SPREADER TESTS PASSED" : "FAILURES",
           fails, fails==1?"":"s");
    return fails ? 1 : 0;
}

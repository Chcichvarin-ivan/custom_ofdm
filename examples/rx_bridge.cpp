// rx_bridge.cpp — Receives PHY packets from fun_ofdm, FEC-decodes, sends
// recovered RTP packets back to a local UDP port for GStreamer playback.

#include "fec_shim.hpp"
#include "receiver.h"      // fun_ofdm
#include "usrp.h"          // fun_ofdm

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>

constexpr int       UDP_OUT_PORT         = 5005;
constexpr std::size_t MAX_APP_PACKET_LEN = 1200;

static std::atomic<bool> g_run{true};
static int g_sock = -1;
static sockaddr_in g_dest{};
static fec::FecDecoder* g_dec = nullptr;
static std::mutex g_dec_mtx;

void on_phy_packets(std::vector<std::vector<unsigned char>> packets) {
    std::lock_guard<std::mutex> lk(g_dec_mtx);
    for (const auto& p : packets) {
        g_dec->feed(p.data(), p.size());
    }
}

void on_app_packet(const uint8_t* data, std::size_t len) {
    sendto(g_sock, data, len, 0, (sockaddr*)&g_dest, sizeof(g_dest));
}

int main() {
    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) { perror("socket"); return 1; }
    g_dest.sin_family = AF_INET;
    g_dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    g_dest.sin_port = htons(UDP_OUT_PORT);
    std::printf("Will forward recovered packets to 127.0.0.1:%d\n", UDP_OUT_PORT);

    fec::FecDecoder dec(MAX_APP_PACKET_LEN, on_app_packet,
                        /*max_inflight_blocks=*/8,
                        /*block_timeout_ms=*/200);
    g_dec = &dec;

    fun::usrp_params params;
    params.freq = 3.3e9;
    params.sample_rate = 10e6;
    params.rx_gain = 40.0;
    params.tx_gain = 0.0;
    params.device_addr = "";
    fun::receiver rx(&on_phy_packets, params);
    std::printf("USRP receiver running at 3.3 GHz, 10 MHz\n");

    std::thread tick_thread([&]{
        while (g_run.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::lock_guard<std::mutex> lk(g_dec_mtx);
            dec.tick();
        }
    });

    while (g_run.load()) std::this_thread::sleep_for(std::chrono::seconds(1));
    tick_thread.join();
    close(g_sock);
    return 0;
}

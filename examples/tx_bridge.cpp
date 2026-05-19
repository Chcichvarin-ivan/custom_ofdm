// tx_bridge.cpp — Reads RTP-over-UDP from GStreamer, FEC-encodes, hands
// PHY-ready packets to fun_ofdm's transmitter at 3.3 GHz QPSK 1/2.

#include "fec_common.h"
#include "fec_encoder.h"
#include "transmitter.h"   // fun_ofdm
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
#include <queue>
#include <mutex>
#include <condition_variable>

constexpr int       UDP_PORT             = 5004;
constexpr int       FEC_K                = 10;
constexpr int       FEC_M                = 3;
constexpr std::size_t MAX_APP_PACKET_LEN = 1200;
constexpr int       FLUSH_INTERVAL_MS    = 33;

static std::atomic<bool> g_run{true};

class PhyQueue {
public:
    void push(std::vector<uint8_t> p) {
        std::lock_guard<std::mutex> lk(m_);
        q_.push(std::move(p));
        cv_.notify_one();
    }
    bool pop(std::vector<uint8_t>& out) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, std::chrono::milliseconds(50),
                     [&]{ return !q_.empty() || !g_run.load(); });
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop();
        return true;
    }
private:
    std::queue<std::vector<uint8_t>> q_;
    std::mutex m_;
    std::condition_variable cv_;
};

int main() {
    PhyQueue phy_q;
    std::mutex enc_mtx;

    fec::FecEncoder enc(FEC_K, FEC_M, MAX_APP_PACKET_LEN,
        [&](const uint8_t* data, std::size_t len) {
            phy_q.push(std::vector<uint8_t>(data, data + len));
        });

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    int rcvbuf = 1 << 20;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(UDP_PORT);
    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }
    std::printf("Listening for RTP on 127.0.0.1:%d\n", UDP_PORT);

    fun::usrp_params params;
    params.freq = 3.3e9;
    params.sample_rate = 10e6;
    params.tx_gain = 70.0;
    params.rx_gain = 0.0;
    params.tx_amp = 0.7;
    params.phy_rate = fun::QPSK_1_2;
    params.device_addr = "";
    fun::transmitter tx(params);
    std::printf("USRP transmitter ready at 3.3 GHz, 10 MHz, QPSK 1/2\n");

    std::thread tx_thread([&]{
        std::vector<uint8_t> pkt;
        while (g_run.load()) {
            if (!phy_q.pop(pkt)) continue;
            tx.send_packet(pkt);
        }
    });

    std::thread flush_thread([&]{
        while (g_run.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(FLUSH_INTERVAL_MS));
            std::lock_guard<std::mutex> lk(enc_mtx);
            enc.flush();
        }
    });

    std::vector<uint8_t> buf(2048);
    while (g_run.load()) {
        ssize_t n = recv(sock, buf.data(), buf.size(), 0);
        if (n <= 0) continue;
        if (static_cast<std::size_t>(n) > MAX_APP_PACKET_LEN) {
            std::fprintf(stderr, "drop oversized RTP packet (%zd B)\n", n);
            continue;
        }
        std::lock_guard<std::mutex> lk(enc_mtx);
        enc.push(buf.data(), n);
    }

    tx_thread.join();
    flush_thread.join();
    close(sock);
    return 0;
}

// Transmit a continuous sine wave to USRP B200mini using UHD
// Compile: g++ -O2 -o tx_sine_wave tx_sine_wave.cpp -luhd
// Run example: ./tx_sine_wave --freq 1e9 --rate 10e6 --gain 40 --wave-freq 100e3 --ampl 0.4

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <complex>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

namespace po = boost::program_options;

int UHD_SAFE_MAIN(int argc, char *argv[])
{
    // Set real-time priority (recommended for low latency)
    uhd::set_thread_priority_safe();

    // --------------------- Command line options ---------------------
    double rate = 10e6;          // sample rate
    double freq = 1.0e9;         // RF center frequency (Hz)
    double wave_freq = 100e3;    // baseband sine frequency (Hz)
    double gain = 40.0;          // TX gain (dB) - start low!
    double ampl = 0.4;           // amplitude [0.0 - 0.7] recommended
    std::string args = "type=b200";  // for B200mini
    std::string ant = "TX/RX";
    std::string ref = "internal";

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("args", po::value<std::string>(&args), "device address args")
        ("rate", po::value<double>(&rate), "sample rate (sps)")
        ("freq", po::value<double>(&freq), "RF center frequency (Hz)")
        ("wave-freq", po::value<double>(&wave_freq), "sine wave frequency (Hz)")
        ("gain", po::value<double>(&gain), "TX gain (dB)")
        ("ampl", po::value<double>(&ampl), "baseband amplitude (0.0 - 0.7)")
        ("ant", po::value<std::string>(&ant), "antenna")
        ("ref", po::value<std::string>(&ref), "clock reference (internal/external/mimo)")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 0;
    }

    // --------------------- Create USRP ---------------------
    std::cout << "Creating USRP with args: " << args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    // Clock reference
    usrp->set_clock_source(ref);

    // Set TX parameters
    usrp->set_tx_rate(rate);
    usrp->set_tx_freq(freq);
    usrp->set_tx_gain(gain);
    usrp->set_tx_antenna(ant);

    std::cout << "TX Rate: " << usrp->get_tx_rate() << " sps" << std::endl;
    std::cout << "TX Freq: " << usrp->get_tx_freq() << " Hz" << std::endl;
    std::cout << "TX Gain: " << usrp->get_tx_gain() << " dB" << std::endl;

    // --------------------- Create TX streamer ---------------------
    uhd::stream_args_t stream_args("fc32");  // complex float32
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // --------------------- Generate sine wave buffer ---------------------
    const size_t spb = tx_stream->get_max_num_samps() * 8;  // reasonable buffer size
    std::vector<std::complex<float>> buff(spb);

    const double phase_inc = 2.0 * M_PI * wave_freq / rate;
    double phase = 0.0;

    for (size_t i = 0; i < spb; ++i) {
        float val = ampl * std::sin(phase);
        buff[i] = std::complex<float>(val, 0.0f);  // real sine wave
        phase += phase_inc;
        if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
    }

    // --------------------- Transmit continuously ---------------------
    std::cout << "Starting continuous sine wave transmission..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = false;

    while (true) {
        size_t num_sent = tx_stream->send(&buff.front(), spb, md);

        if (num_sent < spb) {
            std::cerr << "Warning: underflow detected!" << std::endl;
        }

        // After first packet, clear start_of_burst flag
        md.start_of_burst = false;

        // Small sleep to prevent 100% CPU if needed (usually not required)
        // std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    // Cleanup (never reached unless you add signal handling)
    uhd::tx_metadata_t eob_md;
    eob_md.end_of_burst = true;
    tx_stream->send("", 0, eob_md);

    return 0;
}
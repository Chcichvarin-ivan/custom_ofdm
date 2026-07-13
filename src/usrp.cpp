/*! \file usrp.cpp
 *  \brief C++ file for the usrp class.
 *
 *  The usrp class is a wrapper class for the UHD API. It provides a simple interface for transmitting
 *  and receiving samples to/from the USRP.
 *
 *  The usrp_params class is a container for holding the necessary parameters for the usrp
 *  such as center frequency, sample rate, tx/rx gain, etc..
 */

#include "usrp.h"

namespace fun
{
    /*!
     * -Initializations
     *  + #m_params -> Previously initialized usrp_params object containing the desired parameters
     *    for the USRP.
     */
    usrp::usrp(usrp_params params) :
        m_params(params)
    {
            // Build device args. master_clock_rate must be an integer multiple of sample_rate;
            // for OFDM the AD9361 likes ratios that are powers of two.
            std::string args = "type=b200";
            if (!params.device_addr.empty()) args += "," + params.device_addr;

            // For 5 MHz sample rate, set MCR to 40 MHz (decimation = 8, all halfband filters).
            // For 10 MHz, use 40 MHz (decim = 4). For 20 MHz, use 40 MHz (decim = 2).
            double mcr = std::max(20e6, params.rate * 2.0);
            args += ",master_clock_rate=" + std::to_string(mcr);

            m_usrp = uhd::usrp::multi_usrp::make(args);
            //  m_device = m_usrp->get_device();
            // Single channel, no daughterboards on the B200 mini.
            m_usrp->set_clock_source("external");
            m_usrp->set_time_source("internal");

            // After set_clock_source("external"):
            bool locked = false;
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start).count() < 1500) {
                    uhd::sensor_value_t ref = m_usrp->get_mboard_sensor("ref_locked");
                    if (ref.to_bool()) { locked = true; break; }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                   }
            std::cerr << "Ref locked: " << (locked ? "yes" : "NO") << std::endl;

           //m_usrp->set_rx_subdev_spec(uhd::usrp::subdev_spec_t("A:A"));
            //m_usrp->set_tx_subdev_spec(uhd::usrp::subdev_spec_t("A:A"));

            m_usrp->set_tx_rate(params.rate);
            m_usrp->set_rx_rate(params.rate);

            uhd::tune_request_t tune(params.freq);
            m_usrp->set_tx_freq(tune);
            m_usrp->set_rx_freq(tune);


            // Gain ranges are very different from the XCVR2450:
            // TX 0–89.75 dB, RX 0–76 dB on the B200 mini.
            m_usrp->set_tx_gain(params.tx_gain);  // start around 70
            m_usrp->set_rx_gain(params.rx_gain);  // start around 40

           // m_usrp->set_tx_antenna("TX/RX");
            m_usrp->set_rx_antenna("RX2");  // separate RX port; better isolation than TX/RX

            m_usrp->set_rx_agc(false);
            // Set analog filter bandwidth to match the OFDM signal width.
            m_usrp->set_tx_bandwidth(params.rate * 2);
            m_usrp->set_rx_bandwidth(params.rate * 2);

            m_usrp->set_rx_dc_offset(true);      // enable auto DC offset correction
           m_usrp->set_rx_iq_balance(true,uhd::usrp::multi_usrp::ALL_CHANS);     // enable auto IQ balance correction


            // Streamers — replaces N210's get_device()->get_max_recv_samps_per_packet() pattern.
            uhd::stream_args_t stream_args("fc32", "sc16");
            m_tx_streamer = m_usrp->get_tx_stream(stream_args);
            m_rx_streamer = m_usrp->get_rx_stream(stream_args);

            // Start the RX stream continuously; it stays running.
            uhd::stream_cmd_t stream_cmd(
                        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
            //const auto sensor = m_usrp->get_mboard_sensor("ref_locked");
            //std::cerr << "Ref locked: " << sensor.to_pp_string() << std::endl;
            stream_cmd.stream_now = true;
            m_rx_streamer->issue_stream_cmd(stream_cmd);

            sem_init(&m_tx_sem, 0, 0);
            sem_post(&m_tx_sem);
        /*// Instantiate the multi_usrp
        m_usrp = uhd::usrp::multi_usrp::make(params.device_addr);
        m_device = m_usrp->get_device();

        // Set the center frequency
        m_usrp->set_tx_freq(uhd::tune_request_t(m_params.freq));
        m_usrp->set_rx_freq(uhd::tune_request_t(m_params.freq));

        // Set the sample rate
        m_usrp->set_tx_rate(m_params.rate);
        m_usrp->set_rx_rate(m_params.rate);

        // Set the gains
        m_usrp->set_tx_gain(m_params.tx_gain);
        m_usrp->set_rx_gain(m_params.rx_gain);

        // Set the RX antenna
        m_usrp->set_rx_antenna("RX2");


        m_usrp->set_rx_dc_offset(true);      // enable auto DC offset correction
        m_usrp->set_rx_iq_balance(true,uhd::usrp::multi_usrp::ALL_CHANS);     // enable auto IQ balance correction

        m_usrp ->set_rx_bandwidth(7000000);
        // Get the TX and RX stream handles
        m_tx_streamer = m_usrp->get_tx_stream(uhd::stream_args_t("fc64"));
        m_rx_streamer = m_usrp->get_rx_stream(uhd::stream_args_t("fc64"));

        // Start the RX stream
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        stream_cmd.stream_now = true;
        m_usrp->issue_stream_cmd(stream_cmd);

        sem_init(&m_tx_sem, 0, 0);
        sem_post(&m_tx_sem);*/
    }

    /*!
     * Sends a burst of samples to the USRP which represent the digital base-band signal
     * to be up-converted and transmitted by the USRP.
     *
     * This function does not block after it is called. Due to the multi-threading nature of the
     * UHD API, this function may return before the USRP has finished transmitting all of the
     * samples. This is generally ok as subsequent calls to this method will just buffer more
     * samples in the USRP. However, if it is not called fast enough an underrun may occer.
     * See <a href="http://files.ettus.com/manual/page_general.html#general_ounotes"> link to ettus' website</a>
     * for more details.
     */
    void usrp::send_burst(std::vector<std::complex<double> > samples)
    {
        sem_wait(&m_tx_sem);

        uhd::tx_metadata_t tx_metadata;
        tx_metadata.start_of_burst = true;
        tx_metadata.end_of_burst = true;
        tx_metadata.has_time_spec = false;

        // CHG: When using fc32 wire format above we need a temporary
        // complex<float> buffer; the host samples are still
        // complex<double>. If you kept "fc64" above, delete the
        // conversion and send &samples[0] directly as in the original.
        std::vector<std::complex<float> > fbuf(samples.size());
        for (size_t i = 0; i < samples.size(); ++i)
            fbuf[i] = std::complex<float>(samples[i]);
        m_tx_streamer->send(&fbuf[0], fbuf.size(), tx_metadata);

        sem_post(&m_tx_sem);
    }


    /*!
     * Sends a burst of samples to the USRP which represent the digital base-band signal
     * to be up-converted and transmitted by the USRP.
     *
     * This function uses a semaphore to block until the USRP has responded with an acknowledgement
     * that all the samples have been transmitted over the air. This prevents the user from sending
     * too much data at once so that the user has some sense of when the transmission is finished.
     * If the user does not call this fast enough an underrun may occur.
     * See <a href="http://files.ettus.com/manual/page_general.html#general_ounotes"> link to ettus' website</a>
     * for more details.
     */
    void usrp::send_burst_sync(std::vector<std::complex<double> > samples)
    {
        // Scale the samples by m_amp
        if(m_params.tx_amp != 1.0)
            for(size_t x = 0; x < samples.size(); x++)
                samples[x] *= m_params.tx_amp;

        // CHG: Same fc32 conversion as send_burst.
        std::vector<std::complex<float> > fbuf(samples.size());
        for (size_t i = 0; i < samples.size(); ++i)
            fbuf[i] = std::complex<float>(samples[i]);

        // Send the samples
        uhd::tx_metadata_t tx_metadata;
        tx_metadata.start_of_burst = true;
        tx_metadata.end_of_burst = true;
        tx_metadata.has_time_spec = false;
        m_tx_streamer->send(&fbuf[0], fbuf.size(), tx_metadata);


        // CHG: In UHD 4.x the recommended path for async messages is
        // the tx_streamer, not the device handle. Functionally
        // identical otherwise.
        bool got_ack = false;
        bool got_underflow = false;
        uhd::async_metadata_t async_metadata;
        while (!got_ack && !got_underflow
               && m_tx_streamer->recv_async_msg(async_metadata, 1.0))
        {
            got_ack = (async_metadata.event_code
                       == uhd::async_metadata_t::EVENT_CODE_BURST_ACK);
            got_underflow = (async_metadata.event_code
                             == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW);
            // CHG: bug fix in original — the second clause was ANDed
            // with got_ack which made it impossible to ever set
            // got_underflow=true. We use plain comparison so an
            // underflow correctly breaks the loop.
        }
    }

    /*!
     * Gets num_samples from the USRP and places them in the buffer parameter. If this function
     * is not called "fast enough" the USRP will get upset because the computer is not consuming
     * samples fast enough to keep up with the USRPs receive sample rate.  This will cause the USRP
     * to indicate an overflow and thus not guarantee the integrity of the retrieved data.
     * See <a href="http://files.ettus.com/manual/page_general.html#general_ounotes"> link to ettus' website</a>
     * for more details.
     *
     */
  /*  void usrp::get_samples(int num_samples,
                          std::vector<std::complex<double> > & buffer)
    {
        // CHG: Same fc32 round-trip on RX side.
        std::vector<std::complex<float> > fbuf(num_samples);
        uhd::rx_metadata_t rx_meta;
        const size_t got = m_rx_streamer->recv(&fbuf[0], num_samples, rx_meta);
        for (size_t i = 0; i < got; ++i)
            buffer[i] = std::complex<double>(fbuf[i]);

        // CHG: Surface overflow conditions instead of silently dropping.
        if (rx_meta.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
            std::cerr << "O" << std::flush;       // matches Ettus convention
    }
*/
    void usrp::get_samples(int num_samples, std::vector<std::complex<double> > & buffer)
    {
        std::vector<std::complex<float> > fbuf(num_samples);
        uhd::rx_metadata_t rx_meta;

        // Loop until we have all samples — recv can short-write
        size_t total = 0;
        while (total < (size_t)num_samples) {
            size_t got = m_rx_streamer->recv(&fbuf[total], num_samples - total, rx_meta, 1.0);
            if (got == 0) {
                std::cerr << "recv timeout/error" << std::endl;
                break;
            }
            total += got;
            if (rx_meta.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
                std::cerr << "O" << std::flush;
        }

        if (buffer.size() < (size_t)num_samples) buffer.resize(num_samples);
        for (size_t i = 0; i < total; ++i)
            buffer[i] = std::complex<double>(fbuf[i]);

        // Zero anything we didn't fill
        for (size_t i = total; i < (size_t)num_samples; ++i)
            buffer[i] = std::complex<double>(0.0, 0.0);
    }
}


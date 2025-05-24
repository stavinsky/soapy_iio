#include "ad9361.hpp"

#include <SoapySDR/Logger.hpp>

int AD9361::set_frequency(long long freq, bool output = false) {
    if (output == true) {
        return set_channel_param(lo_channel_output, "frequency", freq);
    }
    return set_channel_param(lo_channel_input, "frequency", freq);
}

int AD9361::set_bandwidth_frequency(long long freq, bool output = false) {
    if (output == true) {
        return set_channel_param(phy_channel_output, "rf_bandwidth", freq);
    }
    return set_channel_param(phy_channel_input, "rf_bandwidth", freq);
}
double AD9361::get_bandwidth_frequency(size_t channel, bool output) {
    (void)channel;
    (void)output;
    iio_channel* chan = iio_device_find_channel(ad9361_phy, "voltage0", false);
    const struct iio_attr* attr = iio_channel_find_attr(chan, "rf_bandwidth");
    double value;
    iio_attr_read_double(attr, &value);
    return value;
}

double AD9361::get_sample_rate(bool output) {
    if (output == true) {
        return get_channel_param(phy_channel_output, "sampling_frequency");
    }
    return get_channel_param(phy_channel_input, "sampling_frequency");
}

double AD9361::get_frequency(bool output) {
    if (output == true) {
        return get_channel_param(lo_channel_output, "frequency");
    }
    return get_channel_param(lo_channel_input, "frequency");
}

int AD9361::set_gain(double value, bool output) {
    (void)output;  // TODO:
    try {
        iio_channel* chan = iio_device_find_channel(ad9361_phy, "voltage0", false);
        const struct iio_attr* attr = iio_channel_find_attr(chan, "hardwaregain");
        iio_attr_write_double(attr, value);

    } catch (...) {
    }
    return 0;
}

void AD9361::set_gain_mode(bool output, bool automatic) {
    if (output == true) {
        return;  // TODO:
    }
    iio_channel* chan = iio_device_find_channel(ad9361_phy, "voltage0", false);
    const struct iio_attr* attr = iio_channel_find_attr(chan, "gain_control_mode");
    iio_attr_write_string(attr, automatic ? "slow_attack" : "manual");
}

bool AD9361::get_gain_mode(bool output) {
    (void)output;  // TODO:
    iio_channel* chan = iio_device_find_channel(ad9361_phy, "voltage0", false);
    const struct iio_attr* attr = iio_channel_find_attr(chan, "gain_control_mode");
    char buf[64];
    iio_attr_read_raw(attr, buf, sizeof(buf));
    std::string mode(buf);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "current gain mode is %s ", buf);
    return mode != "manual";
}

double AD9361::get_gain(bool output) {
    (void)output;  // TODO:
    iio_channel* chan = iio_device_find_channel(ad9361_phy, "voltage0", false);
    const struct iio_attr* attr = iio_channel_find_attr(chan, "hardwaregain");
    double value;
    iio_attr_read_double(attr, &value);
    return value;
}

int AD9361::set_sample_rate(long long freq, bool output = false) {
    if (output == true) {
        return set_channel_param(phy_channel_output, "sampling_frequency", freq);
    }
    return set_channel_param(phy_channel_input, "sampling_frequency", freq);
}

AD9361::AD9361(std::string url) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "constructor start");
    ctx = iio_create_context(NULL, url.c_str());
    int error = iio_err(ctx);
    if (error != 0) {
        throw std::runtime_error("can't create context. check url");
    }
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    if (!ad9361_phy) {
        throw std::runtime_error("No ad9361-phy found");
    }
    phy_channel_input = iio_device_find_channel(ad9361_phy, "voltage0", false);
    phy_channel_output = iio_device_find_channel(ad9361_phy, "voltage0", true);

    lo_channel_input = iio_device_find_channel(ad9361_phy, "altvoltage0", true);
    if (!lo_channel_input) {
        throw std::runtime_error("No lo_channel_input found");
    }
    lo_channel_output = iio_device_find_channel(ad9361_phy, "altvoltage1", true);
    if (!lo_channel_output) {
        throw std::runtime_error("No lo_channel_output found");
    }
    rf_port_select(phy_channel_input, "A_BALANCED");
    rf_port_select(phy_channel_output, "A_BALANCED");

    device_output = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    device_input = iio_context_find_device(ctx, "cf-ad9361-lpc");
    if (!device_input) {
        throw std::runtime_error("No device_input");
    }

    rx_mask = iio_create_channels_mask(iio_device_get_channels_count(device_input));
    if (!rx_mask) {
        throw std::runtime_error("No rx_mask");
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AD9361 constructor end");
}

AD9361::~AD9361() {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AD9361 destructor start");
    if (rx_mask) {
        iio_channels_mask_destroy(rx_mask);
    }
    if (rx_stream) {
        iio_stream_destroy(rx_stream);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_stream destroed ");
    if (rx_buffer) {
        iio_buffer_destroy(rx_buffer);
    }
    if (ctx) {
        iio_context_destroy(ctx);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AD9361 destructor end");
}

int AD9361::set_channel_param(iio_channel* channel, const char* key, long long value) {
    const struct iio_attr* attr = iio_channel_find_attr(channel, key);
    return iio_attr_write_longlong(attr, value);
}
int AD9361::set_channel_param_double(iio_channel* channel, const char* key, double value) {
    const struct iio_attr* attr = iio_channel_find_attr(channel, key);
    return iio_attr_write_double(attr, value);
}
long long AD9361::get_channel_param(iio_channel* channel, const char* key) {
    const struct iio_attr* attr = iio_channel_find_attr(channel, key);
    long long val;
    iio_attr_read_longlong(attr, &val);
    return val;
}

ssize_t AD9361::rf_port_select(iio_channel* channel, std::string rf_port) {
    const struct iio_attr* attr = iio_channel_find_attr(channel, "rf_port_select");

    return iio_attr_write_string(attr, rf_port.c_str());
}

size_t AD9361::get_rx_sample_size() {
    ssize_t sample_rate = iio_device_get_sample_size(device_input, rx_mask);
    return static_cast<size_t>(sample_rate);
}
void AD9361::rx_channel_enable() {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_channel_enable");
    uint8_t channel = 0;
    if (!device_input) {
        throw std::runtime_error("devce_input is not created");
    }

    rx_chan[channel].rx_ch_i = iio_device_find_channel(device_input, "voltage0", false);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_channel_enable debug");
    if (!rx_chan[channel].rx_ch_i) {
        throw std::runtime_error("unable to get I channel");
    }
    rx_chan[channel].rx_ch_q = iio_device_find_channel(device_input, "voltage1", false);
    if (!rx_chan[channel].rx_ch_q) {
        throw std::runtime_error("unable to get Q channel");
    }

    iio_channel_enable(rx_chan[channel].rx_ch_i, rx_mask);
    iio_channel_enable(rx_chan[channel].rx_ch_q, rx_mask);
    // active.store(true, std::memory_order_release);
    rx_buffer = iio_device_create_buffer(device_input, 0, rx_mask);
    if (iio_err(rx_buffer) != 0) {
        throw std::runtime_error("No rx_buffer");
    }
    rx_stream = iio_buffer_create_stream(rx_buffer, 4, BLOCK_SIZE);
    if (iio_err(rx_stream) != 0) {
        throw std::runtime_error("No rx_stream");
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_channel_enable end");
}
void AD9361::rx_channel_disable() {
    // active.store(false, std::memory_order_release);
    uint8_t channel = 0;
    rx_chan[channel].rx_ch_i = iio_device_find_channel(device_input, "voltage0", false);
    if (!rx_chan[channel].rx_ch_i) {
        throw std::runtime_error("unable to get I channel");
    }
    rx_chan[channel].rx_ch_q = iio_device_find_channel(device_input, "voltage1", false);
    if (!rx_chan[channel].rx_ch_q) {
        throw std::runtime_error("unable to get Q channel");
    }
    if (rx_mask && rx_chan[channel].rx_ch_i && rx_chan[channel].rx_ch_q) {
        iio_channel_disable(rx_chan[channel].rx_ch_i, rx_mask);
        iio_channel_disable(rx_chan[channel].rx_ch_q, rx_mask);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_channel_disabled");
    }
}
BlockPointer AD9361::prepare_next_block() {
    // std::lock_guard<std::mutex> lock(mtx);
    // if (!active.load(std::memory_order_acquire)) {
    //     throw std::runtime_error("channel is not active");
    // }
    const iio_block* rx_block = iio_stream_get_next_block(rx_stream);
    int err = iio_err(rx_block);
    if (err) {
        throw std::runtime_error("unable to receive block");
    }

    int16_t* p_end = reinterpret_cast<int16_t*>(iio_block_end(rx_block));
    int16_t* p_start = reinterpret_cast<int16_t*>(iio_block_first(rx_block, rx_chan[0].rx_ch_i));
    return {p_start, p_end};
}
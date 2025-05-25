#include "ad9361.hpp"

#include <SoapySDR/Logger.hpp>
#include <sstream>
#include <vector>

enum class IQ : char {
    i = 'i',
    q = 'q'
};

std::vector<std::string> split_string(std::string str) {
    std::istringstream iss(str);
    std::string word;
    std::vector<std::string> words;
    while (iss >> word) {
        words.push_back(word);
    }
    return words;
}
std::string channel_voltage_name(uint8_t channel, IQ iq) {
    std::string attr = "voltage";
    if (channel == 0) {
        if (iq == IQ::i) {
            return attr + "0";
        }
        if (iq == IQ::q) {
            return attr + "1";
        }
    } else if (channel == 1) {
        if (iq == IQ::i) {
            return attr + "2";
        }
        if (iq == IQ::q) {
            return attr + "3";
        }
    } else {
        throw std::runtime_error("channel could be only 0 or 1 ");
    }
}

int AD9361::set_frequency(long long freq, bool output) {
    if (output == true) {
        return set_channel_param(lo_channel_output, "frequency", freq);
    }
    return set_channel_param(lo_channel_input, "frequency", freq);
}

int AD9361::set_bandwidth_frequency(long long freq, bool output) {
    if (output == true) {
        return set_channel_param(phy_channel_output, "rf_bandwidth", freq);
    }
    return set_channel_param(phy_channel_input, "rf_bandwidth", freq);
}
double AD9361::get_bandwidth_frequency(bool output) {
    iio_channel* chan = iio_device_find_channel(ad9361_phy, "voltage0", output);
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

int AD9361::set_gain(uint8_t channel, double value, bool output) {
    iio_channel* chan;
    if (channel == 0) {
        chan = iio_device_find_channel(ad9361_phy, "voltage0", output);
    } else if (channel == 1) {
        chan = iio_device_find_channel(ad9361_phy, "voltage1", output);
    } else {
        throw std::runtime_error("can't create context. check url");
    }

    const struct iio_attr* attr = iio_channel_find_attr(chan, "hardwaregain");
    try {
        iio_attr_write_double(attr, value);

    } catch (...) {
    }
    return 0;
}
double AD9361::get_gain(uint8_t channel, bool output) {
    iio_channel* chan;
    if (channel == 0) {
        chan = iio_device_find_channel(ad9361_phy, "voltage0", output);
    } else if (channel == 1) {
        chan = iio_device_find_channel(ad9361_phy, "voltage1", output);
    } else {
        throw std::runtime_error("can't create context. check url");
    }

    const struct iio_attr* attr = iio_channel_find_attr(chan, "hardwaregain");
    double value;
    iio_attr_read_double(attr, &value);
    return value;
}
void AD9361::set_gain_mode(uint8_t channel, bool output, bool automatic) {
    iio_channel* chan;
    if (channel == 0) {
        chan = iio_device_find_channel(ad9361_phy, "voltage0", output);
    } else if (channel == 1) {
        chan = iio_device_find_channel(ad9361_phy, "voltage1", output);
    } else {
        throw std::runtime_error("can't create context. check url");
    }
    const struct iio_attr* attr = iio_channel_find_attr(chan, "gain_control_mode");
    iio_attr_write_string(attr, automatic ? "slow_attack" : "manual");
}

bool AD9361::get_gain_mode(uint8_t channel, bool output) {
    iio_channel* chan;
    if (channel == 0) {
        chan = iio_device_find_channel(ad9361_phy, "voltage0", output);
    } else if (channel == 1) {
        chan = iio_device_find_channel(ad9361_phy, "voltage1", output);
    } else {
        throw std::runtime_error("can't create context. check url");
    }
    const struct iio_attr* attr = iio_channel_find_attr(chan, "gain_control_mode");
    char buf[64];
    iio_attr_read_raw(attr, buf, sizeof(buf));
    std::string mode(buf);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "current gain mode is %s ", buf);
    return mode != "manual";
}

int AD9361::set_sample_rate(long long freq, bool output) {
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

size_t AD9361::get_rx_sample_size() {
    ssize_t sample_rate = iio_device_get_sample_size(device_input, rx_mask);
    return static_cast<size_t>(sample_rate);
}
void AD9361::rx_channel_enable(uint8_t channel) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_channel_enable");
    if (!device_input) {
        throw std::runtime_error("devce_input is not created");
    }

    rx_chan[channel].rx_ch_i = iio_device_find_channel(device_input, channel_voltage_name(channel, IQ::i).c_str(), false);
    if (!rx_chan[channel].rx_ch_i) {
        throw std::runtime_error("unable to get I channel");
    }
    rx_chan[channel].rx_ch_q = iio_device_find_channel(device_input, channel_voltage_name(channel, IQ::q).c_str(), false);
    if (!rx_chan[channel].rx_ch_q) {
        throw std::runtime_error("unable to get Q channel");
    }

    iio_channel_enable(rx_chan[channel].rx_ch_i, rx_mask);
    iio_channel_enable(rx_chan[channel].rx_ch_q, rx_mask);
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
void AD9361::rx_channel_disable(uint8_t channel) {
    rx_chan[channel].rx_ch_i = iio_device_find_channel(device_input, channel_voltage_name(channel, IQ::i).c_str(), false);
    if (!rx_chan[channel].rx_ch_i) {
        throw std::runtime_error("unable to get I channel");
    }
    rx_chan[channel].rx_ch_q = iio_device_find_channel(device_input, channel_voltage_name(channel, IQ::q).c_str(), false);
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
    iio_channel* ch = (rx_chan[0].rx_ch_i) ? rx_chan[0].rx_ch_i : rx_chan[1].rx_ch_i;
    if (!ch) {
        throw std::runtime_error("can't prepare block. wrong channel selected");
    }
    int16_t* p_start = reinterpret_cast<int16_t*>(iio_block_first(rx_block, ch));
    return {p_start, p_end};
}

std::vector<std::string> AD9361::get_available_rf_ports(uint8_t channel, bool output) {
    iio_channel* chan;
    if (channel == 0) {
        chan = iio_device_find_channel(ad9361_phy, "voltage0", output);
    } else if (channel == 1) {
        chan = iio_device_find_channel(ad9361_phy, "voltage1", output);
    } else {
        throw std::runtime_error("can't create context. check url");
    }
    const struct iio_attr* attr = iio_channel_find_attr(chan, "rf_port_select_available");
    char buf[500];
    iio_attr_read_raw(attr, buf, sizeof(buf));
    return split_string(buf);
}
ssize_t AD9361::rf_port_select(uint8_t channel, bool output, std::string rf_port) {
    iio_channel* chan;
    if (channel == 0) {
        chan = iio_device_find_channel(ad9361_phy, "voltage0", output);
    } else if (channel == 1) {
        chan = iio_device_find_channel(ad9361_phy, "voltage1", output);
    } else {
        throw std::runtime_error("can't create context. check url");
    }
    const struct iio_attr* attr = iio_channel_find_attr(chan, "rf_port_select");

    return iio_attr_write_string(attr, rf_port.c_str());
}

std::string AD9361::get_rf_port(uint8_t channel, bool output) {
    iio_channel* chan;
    if (channel == 0) {
        chan = iio_device_find_channel(ad9361_phy, "voltage0", output);
    } else if (channel == 1) {
        chan = iio_device_find_channel(ad9361_phy, "voltage1", output);
    } else {
        throw std::runtime_error("can't create context. check url");
    }
    const struct iio_attr* attr = iio_channel_find_attr(chan, "rf_port_select");
    char buf[500];
    iio_attr_read_raw(attr, buf, sizeof(buf));
    return std::string(buf);
}

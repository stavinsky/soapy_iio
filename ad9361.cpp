#include "ad9361.hpp"

#include <SoapySDR/Logger.hpp>
#include <cerrno>
#include <sstream>
#include <vector>

#include "errors.hpp"
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
    }
    throw std::runtime_error("channel could be only 0 or 1 ");
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
    double value;
    iio_channel_attr_read_double(chan, "rf_bandwidth", &value);
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

    return iio_channel_attr_write_double(chan, "hardwaregain", value);
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

    double value;
    iio_channel_attr_read_double(chan, "hardwaregain", &value);
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
    iio_channel_attr_write(chan, "gain_control_mode", automatic ? "slow_attack" : "manual");
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
    char buf[64];
    iio_channel_attr_read(chan, "gain_control_mode", buf, sizeof(buf));
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
    ctx = iio_create_context_from_uri(url.c_str());
    if (!ctx) {
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
    if (!device_output) {
        throw std::runtime_error("No device_input");
    }
    device_input = iio_context_find_device(ctx, "cf-ad9361-lpc");
    if (!device_input) {
        throw std::runtime_error("No device_input");
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "AD9361 constructor end");
}

AD9361::~AD9361() {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AD9361 destructor start");
    if (rx_buffer) {
        iio_buffer_destroy(rx_buffer);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_buffer destroed ");
    }

    if (tx_buffer) {
        iio_buffer_destroy(tx_buffer);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "tx_buffer destroed ");
    }
    if (ctx) {
        iio_context_destroy(ctx);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AD9361 destructor end");
}

int AD9361::set_channel_param(iio_channel* channel, const char* key, long long value) {
    return iio_channel_attr_write_longlong(channel, key, value);
}
int AD9361::set_channel_param_double(iio_channel* channel, const char* key, double value) {
    return iio_channel_attr_write_double(channel, key, value);
}
long long AD9361::get_channel_param(iio_channel* channel, const char* key) {
    long long val;
    iio_channel_attr_read_longlong(channel, key, &val);
    return val;
}

size_t AD9361::get_rx_sample_size() {
    ssize_t sample_rate = iio_device_get_sample_size(device_input);
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

    if (rx_buffer) {
        iio_buffer_destroy(rx_buffer);
        rx_buffer = NULL;
    }
    iio_channel_enable(rx_chan[channel].rx_ch_i);
    iio_channel_enable(rx_chan[channel].rx_ch_q);
    rx_buffer = iio_device_create_buffer(device_input, BLOCK_SIZE, false);
    if (!rx_buffer) {
        throw std::runtime_error("No rx_buffer");
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
    if (rx_chan[channel].rx_ch_i && rx_chan[channel].rx_ch_q) {
        if (rx_buffer) {
            iio_buffer_destroy(rx_buffer);
            rx_buffer = NULL;
        }
        iio_channel_disable(rx_chan[channel].rx_ch_i);
        iio_channel_disable(rx_chan[channel].rx_ch_q);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_channel_disabled");
    }
}

void AD9361::tx_channel_enable(uint8_t channel) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "tx_channel_enable");
    if (!device_output) {
        throw std::runtime_error("devce_input is not created");
    }

    tx_chan[channel].tx_ch_i = iio_device_find_channel(device_output, channel_voltage_name(channel, IQ::i).c_str(), true);
    if (!tx_chan[channel].tx_ch_i) {
        throw std::runtime_error("unable to get I channel");
    }
    tx_chan[channel].tx_ch_q = iio_device_find_channel(device_output, channel_voltage_name(channel, IQ::q).c_str(), true);
    if (!tx_chan[channel].tx_ch_q) {
        throw std::runtime_error("unable to get Q channel");
    }

    if (tx_buffer) {
        iio_buffer_destroy(tx_buffer);
        tx_buffer = NULL;
    }
    iio_channel_enable(tx_chan[channel].tx_ch_i);
    iio_channel_enable(tx_chan[channel].tx_ch_q);
    tx_buffer = iio_device_create_buffer(device_output, BLOCK_SIZE, false);
    if (!tx_buffer) {
        throw std::runtime_error("No tx_buffer");
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "tx_channel_enable end");
}
void AD9361::tx_channel_disable(uint8_t channel) {
    tx_chan[channel].tx_ch_i = iio_device_find_channel(device_output, channel_voltage_name(channel, IQ::i).c_str(), true);
    if (!tx_chan[channel].tx_ch_i) {
        throw std::runtime_error("unable to get I channel");
    }
    tx_chan[channel].tx_ch_q = iio_device_find_channel(device_output, channel_voltage_name(channel, IQ::q).c_str(), true);
    if (!tx_chan[channel].tx_ch_q) {
        throw std::runtime_error("unable to get Q channel");
    }
    if (tx_chan[channel].tx_ch_i && tx_chan[channel].tx_ch_q) {
        if (tx_buffer) {
            iio_buffer_destroy(tx_buffer);
            tx_buffer = NULL;
        }
        iio_channel_disable(tx_chan[channel].tx_ch_i);
        iio_channel_disable(tx_chan[channel].tx_ch_q);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "tx_channel_disabled");
    }
}
BlockPointer AD9361::prepare_next_block() {
    // TODO: set timeout
    ssize_t bytes = iio_buffer_refill(rx_buffer);
    if (bytes < 0) {
        if (bytes == -ETIMEDOUT) {
            throw TimeoutError("get next block has timed out");
        }
        printf("error from libiio: %d ", static_cast<int>(bytes));
        throw std::runtime_error("unable to receive block %d");
    }

    int16_t* p_end = reinterpret_cast<int16_t*>(iio_buffer_end(rx_buffer));
    iio_channel* ch = (rx_chan[0].rx_ch_i) ? rx_chan[0].rx_ch_i : rx_chan[1].rx_ch_i;
    if (!ch) {
        throw std::runtime_error("can't prepare block. wrong channel selected");
    }
    int16_t* p_start = reinterpret_cast<int16_t*>(iio_buffer_first(rx_buffer, ch));
    return {p_start, p_end};
}

BlockPointer AD9361::prepare_next_block_tx() {
    if (!tx_buffer) {
        throw std::runtime_error("tx buffer is not created");
    }

    int16_t* p_end = reinterpret_cast<int16_t*>(iio_buffer_end(tx_buffer));
    iio_channel* ch = (tx_chan[0].tx_ch_i) ? tx_chan[0].tx_ch_i : tx_chan[1].tx_ch_i;
    if (!ch) {
        throw std::runtime_error("can't prepare block. wrong channel selected");
    }
    int16_t* p_start = reinterpret_cast<int16_t*>(iio_buffer_first(tx_buffer, ch));
    return {p_start, p_end};
}

void AD9361::push_tx_buffer() {
    if (!tx_buffer) {
        return;
    }
    ssize_t bytes = iio_buffer_push(tx_buffer);
    if (bytes < 0) {
        throw std::runtime_error("unable to push tx buffer");
    }
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
    char buf[500];
    iio_channel_attr_read(chan, "rf_port_select_available", buf, sizeof(buf));
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
    return iio_channel_attr_write(chan, "rf_port_select", rf_port.c_str());
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
    char buf[500];
    iio_channel_attr_read(chan, "rf_port_select", buf, sizeof(buf));
    return std::string(buf);
}

void AD9361::load_filter_from_buffer(std::vector<uint8_t> buffer) {
    ssize_t bytes = iio_device_attr_write_raw(ad9361_phy, "filter_fir_config", buffer.data(), buffer.size());
    if (bytes <= 0) {
        throw std::runtime_error("loading filter failed");
    }
}

void AD9361::fir_filter_enable(bool en) {
    iio_channel* chan_out = iio_device_find_channel(ad9361_phy, "out", false);
    int err = iio_channel_attr_write_bool(chan_out, "voltage_filter_fir_en", en);
    if (err != 0) {
        throw std::runtime_error("changing filter state failed");
    }
}

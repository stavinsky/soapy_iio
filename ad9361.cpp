#include "ad9361.hpp"

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

int AD9361::set_sample_rate(long long freq, bool output = false) {
    if (output == true) {
        return set_channel_param(phy_channel_output, "sampling_frequency", freq);
    }
    return set_channel_param(phy_channel_input, "sampling_frequency", freq);
}

AD9361::AD9361(std::string url) {
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
}

int AD9361::set_channel_param(iio_channel* channel, const char* key, long long value) {
    const struct iio_attr* attr = iio_channel_find_attr(channel, key);
    return iio_attr_write_longlong(attr, value);
}

int AD9361::rf_port_select(iio_channel* channel, std::string rf_port) {
    const struct iio_attr* attr = iio_channel_find_attr(channel, "rf_port_select");
    return iio_attr_write_string(attr, rf_port.c_str());
}

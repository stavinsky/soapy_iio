#pragma once

#include <iio/iio.h>

#include <string>
#define MHZ(x) ((long long)(x * 1000000.0 + .5))
#define GHZ(x) ((long long)(x * 1000000000.0 + .5))

using namespace std;
enum iodev { RX,
             TX };
class AD9361 {
   public:
    int set_frequency(long long freq, bool output);
    // int set_gain(long long freq);
    // int agc_on(bool agc);
    int set_bandwidth_frequency(long long freq, bool output);
    int set_sample_rate(long long freq, bool output);
    AD9361(std::string url);
    ~AD9361();
    iio_device* device_output;
    iio_device* device_input;
    iio_context* ctx;
    double get_sample_rate(bool output);
    double get_frequency(bool output);
    int set_gain(double value, bool output);

    // ssize_t get_rx_sample_size();
    // ssize_t get_tx_sample_size();

    iio_buffer* get_rx_buffer();

   private:
    iio_channel* phy_channel_input;
    iio_channel* phy_channel_output;  // TODO: set it up
    iio_channel* lo_channel_input;
    iio_channel* lo_channel_output;
    iio_device* ad9361_phy;

    int set_channel_param(iio_channel* channel, const char* key, long long value);
    long long get_channel_param(iio_channel* channel, const char* key);
    int set_channel_param_double(iio_channel* channel, const char* key, double value);
    int rf_port_select(iio_channel* channel, std::string rf_port);
};
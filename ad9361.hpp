#pragma once

#include <iio.h>

#include <string>
#define BLOCK_SIZE (125 * 1024)  // TODO: make parameter
using namespace std;
struct rx_channel {
    iio_channel* rx_ch_i;
    iio_channel* rx_ch_q;
};
struct BlockPointer {
    int16_t* current;
    int16_t* end;
};

class AD9361 {
   public:
    int set_frequency(long long freq, bool output);
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
    void set_gain_mode(bool output, bool automatic);
    bool get_gain_mode(bool output);
    double get_gain(bool output);
    double get_bandwidth_frequency(size_t channle, bool output);
    void rx_channel_enable();
    void rx_channel_disable();
    size_t get_rx_sample_size();
    BlockPointer prepare_next_block();

    rx_channel rx_chan[2];
    iio_channels_mask* rx_mask;
    struct iio_buffer* rx_buffer;
    struct iio_stream* rx_stream;

   private:
    iio_channel* phy_channel_input;
    iio_channel* phy_channel_output;  // TODO: set it up
    iio_channel* lo_channel_input;
    iio_channel* lo_channel_output;
    iio_device* ad9361_phy;

    int set_channel_param(iio_channel* channel, const char* key, long long value);
    long long get_channel_param(iio_channel* channel, const char* key);
    int set_channel_param_double(iio_channel* channel, const char* key, double value);
    ssize_t rf_port_select(iio_channel* channel, std::string rf_port);
};
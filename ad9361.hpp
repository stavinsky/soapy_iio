#pragma once

#include <iio.h>

#include <string>
#include <vector>
#define BLOCK_SIZE (125 * 1024)  // TODO: make parameter
using namespace std;
struct rx_channel {
    iio_channel* rx_ch_i;
    iio_channel* rx_ch_q;
};
struct tx_channel {
    iio_channel* tx_ch_i;
    iio_channel* tx_ch_q;
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
    int set_gain(uint8_t channel, double value, bool output);
    void set_gain_mode(uint8_t channel, bool output, bool automatic);
    bool get_gain_mode(uint8_t channel, bool output);
    double get_gain(uint8_t channe, bool output);
    double get_bandwidth_frequency(bool output);
    void rx_channel_enable(uint8_t channel);
    void rx_channel_disable(uint8_t channel);

    void tx_channel_enable(uint8_t channel);
    void tx_channel_disable(uint8_t channel);

    size_t get_rx_sample_size();
    BlockPointer prepare_next_block();
    BlockPointer prepare_next_block_tx();

    std::vector<std::string> get_available_rf_ports(uint8_t channel, bool output);
    ssize_t rf_port_select(uint8_t channel, bool output, std::string rf_port);
    std::string get_rf_port(uint8_t channel, bool output);

    rx_channel rx_chan[2];
    tx_channel tx_chan[2];

    iio_channels_mask* rx_mask;
    iio_channels_mask* tx_mask;

    struct iio_buffer* rx_buffer;
    struct iio_stream* rx_stream;

    struct iio_buffer* tx_buffer;
    struct iio_stream* tx_stream;

   private:
    iio_channel* phy_channel_input;
    iio_channel* phy_channel_output;  // TODO: set it up
    iio_channel* lo_channel_input;
    iio_channel* lo_channel_output;
    iio_device* ad9361_phy;

    int set_channel_param(iio_channel* channel, const char* key, long long value);
    long long get_channel_param(iio_channel* channel, const char* key);
    int set_channel_param_double(iio_channel* channel, const char* key, double value);
};
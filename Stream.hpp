#pragma once
#include <iio.h>

#include <string>
#include <vector>

class Stream {
   public:
    int direction;
    std::string format;
    std::vector<size_t> channels;
    struct iio_buffer* rx_buffer;
    struct iio_stream* rx_stream;
    bool current_buffer_finished = true;
    iio_channels_mask* rx_mask;

    int16_t* p_dat;
    int16_t* p_end;
    iio_channel* rx_ch_i;
    iio_channel* rx_ch_q;
    bool active = false;
    iio_device* device;
    const iio_block* rx_block;

    void rx_channel_enable();
    size_t get_rx_sample_size();
    Stream(iio_device* device);
    ~Stream();
    void prepare_next_block();
};
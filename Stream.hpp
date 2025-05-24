#pragma once
#include <iio.h>

#include <string>
#include <vector>

#include "ad9361.hpp"

class Stream {
   public:
    int direction;
    std::string format;
    std::vector<size_t> channels;

    bool current_buffer_finished = true;
    BlockPointer bp;
    // iio_channels_mask* rx_mask;

    // void rx_channel_enable();
    // void rx_channel_disable();
    // size_t get_rx_sample_size();
    Stream();
    ~Stream();
    std::atomic<bool> active{false};
    std::mutex mtx;
};

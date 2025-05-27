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
    Stream();
    ~Stream();
    std::atomic<bool> active{false};
    std::mutex mtx;
};

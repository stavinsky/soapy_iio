#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
extern "C" {
#include <iio/iio-debug.h>
}
#include "ad9361.hpp"
#include "iio.h"

bool readFileToBuffer(const std::string &path, std::vector<uint8_t> &buffer) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);  // open at end to get size
    if (!file) {
        std::cerr << "Failed to open file: " << path << "\n";
        return false;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);  // rewind to beginning

    buffer.resize(size);
    if (!file.read(reinterpret_cast<char *>(buffer.data()), size)) {
        std::cerr << "Failed to read file: " << path << "\n";
        return false;
    }

    return true;
}
int main(void) {
    std::string url = "ip:192.168.88.194";
    AD9361 device = AD9361(url);
    std::vector<uint8_t> buffer;
    readFileToBuffer("ad9361_custom.ftr", buffer);
    // readFileToBuffer("../test.ftr", buffer);
    device.fir_filter_enable(false);
    device.load_filter_from_buffer(buffer);
    device.fir_filter_enable(true);
    printf("success\n");
    return 0;
}

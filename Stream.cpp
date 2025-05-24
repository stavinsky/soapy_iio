#include "Stream.hpp"

#include <stdexcept>
#include <thread>

#include "SoapySDR/Logger.hpp"
using namespace std;

enum iodev { RX,
             TX };

static char tmpstr[64];
static char* get_ch_name(const char* type, int id) {
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return tmpstr;
}

static bool get_ad9361_stream_ch(enum iodev d, struct iio_device* dev, int chid, struct iio_channel** chn) {
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != NULL;
}

void Stream::rx_channel_enable() {
    bool success;
    success = get_ad9361_stream_ch(RX, device, 0, &rx_ch_i);
    if (!success) {
        throw std::runtime_error("unable to get I channel");
    }
    success = get_ad9361_stream_ch(RX, device, 1, &rx_ch_q);
    if (!success) {
        throw std::runtime_error("unable to get Q channel");
    }
    iio_channel_enable(rx_ch_i, rx_mask);
    iio_channel_enable(rx_ch_q, rx_mask);
    active.store(true, std::memory_order_release);
}
void Stream::rx_channel_disable() {
    active.store(false, std::memory_order_release);
    bool success;
    success = get_ad9361_stream_ch(RX, device, 0, &rx_ch_i);
    if (!success) {
        throw std::runtime_error("unable to get I channel");
    }
    success = get_ad9361_stream_ch(RX, device, 1, &rx_ch_q);
    if (!success) {
        throw std::runtime_error("unable to get Q channel");
    }
    if (rx_mask && rx_ch_i && rx_ch_q) {
        iio_channel_disable(rx_ch_i, rx_mask);
        iio_channel_disable(rx_ch_q, rx_mask);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_channel_disabled");
    }
}
size_t Stream::get_rx_sample_size() {
    ssize_t sample_rate = iio_device_get_sample_size(device, rx_mask);
    return static_cast<size_t>(sample_rate);
}
Stream::Stream(iio_device* dev) {
    device = dev;
    if (!device) {
        throw std::runtime_error("No device");
    }

    rx_mask = iio_create_channels_mask(iio_device_get_channels_count(device));
    rx_channel_enable();
    // SoapySDR_logf(SOAPY_SDR_DEBUG, "channel_count %d", iio_device_get_channels_count(device));
    if (!rx_mask) {
        throw std::runtime_error("No rx_mask");
    }
    rx_buffer = iio_device_create_buffer(device, 0, rx_mask);
    if (iio_err(rx_buffer) != 0) {
        throw std::runtime_error("No rx_buffer");
    }
    rx_stream = iio_buffer_create_stream(rx_buffer, 4, BLOCK_SIZE);
    if (iio_err(rx_stream) != 0) {
        throw std::runtime_error("No rx_stream");
    }
}

Stream::~Stream() {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Stream destructor start");
    if (rx_stream) {
        iio_stream_destroy(rx_stream);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_stream destroed ");
    if (rx_buffer) {
        iio_buffer_destroy(rx_buffer);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_buffer destroed ");
    if (rx_mask) {
        iio_channels_mask_destroy(rx_mask);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx_mask destroed ");
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Stream destructor stop");
}

void Stream::prepare_next_block() {
    std::lock_guard<std::mutex> lock(mtx);
    if (!active.load(std::memory_order_acquire)) {
        throw std::runtime_error("channel is not active");
    }
    const iio_block* rx_block = iio_stream_get_next_block(rx_stream);
    int err = iio_err(rx_block);
    if (err) {
        throw std::runtime_error("unable to receive block");
    }

    p_end = reinterpret_cast<int16_t*>(iio_block_end(rx_block));
    current_buffer_finished = false;
    p_dat = reinterpret_cast<int16_t*>(iio_block_first(rx_block, rx_ch_i));
}

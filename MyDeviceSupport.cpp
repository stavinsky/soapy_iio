
extern "C" {
#include <iio/iio-debug.h>
}
// #include <iio/iio.h>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "ad9361.hpp"

#define BLOCK_SIZE (32000 * 4)  // TODO: make parameter

#define IIO_ENSURE(expr)                                                             \
    {                                                                                \
        if (!(expr)) {                                                               \
            (void)fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
            (void)abort();                                                           \
        }                                                                            \
    }

class DummyStream {
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
    ssize_t get_rx_sample_size();
    DummyStream(iio_device* device);
    ~DummyStream();
    void prepare_next_block() {
        SoapySDR_logf(SOAPY_SDR_TRACE, "prepare block");
        rx_block = iio_stream_get_next_block(rx_stream);
        int err = iio_err(rx_block);
        if (err) {
            throw std::runtime_error("unable to receive block");
        }
        p_end = reinterpret_cast<int16_t*>(iio_block_end(rx_block));
        current_buffer_finished = false;
        p_dat = reinterpret_cast<int16_t*>(iio_block_first(rx_block, rx_ch_i));
    }
};
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

void DummyStream::rx_channel_enable() {
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
}
ssize_t DummyStream::get_rx_sample_size() {
    return iio_device_get_sample_size(device, rx_mask);
}
DummyStream::DummyStream(iio_device* dev) {
    device = dev;
    if (!device) {
        throw std::runtime_error("No device");
    }

    rx_mask = iio_create_channels_mask(iio_device_get_channels_count(device));
    rx_channel_enable();
    SoapySDR_logf(SOAPY_SDR_DEBUG, "channel_count %d", iio_device_get_channels_count(device));
    if (!rx_mask) {
        throw std::runtime_error("No rx_mask");
    }
    rx_buffer = iio_device_create_buffer(device, 0, rx_mask);
    if (iio_err(rx_buffer) != 0) {
        dev_perror(device, iio_err(rx_buffer), "Could not create RX buffer");
        throw std::runtime_error("No rx_buffer");
    }
    rx_stream = iio_buffer_create_stream(rx_buffer, 4, BLOCK_SIZE);
    if (iio_err(rx_stream) != 0) {
        throw std::runtime_error("No rx_stream");
    }
}

DummyStream::~DummyStream() {
    if (rx_stream) {
        iio_stream_destroy(rx_stream);
    }
    if (rx_buffer) {
        iio_buffer_destroy(rx_buffer);
    }
    if (rx_mask) {
        iio_channels_mask_destroy(rx_mask);
    }

    exit(0);
}

/***********************************************************************
 * Device interface
 **********************************************************************/
using namespace std;
class MyDevice : public SoapySDR::Device {
   public:
    std::string getDriverKey(void) const;
    std::string getHardwareKey(void) const;
    size_t getNumChannels(const int direction) const;
    bool getFullDuplex(const int direction, const size_t channel) const;
    SoapySDR::Stream* setupStream(const int direction, const std::string& format, const std::vector<size_t>& channels, const SoapySDR::Kwargs& args);
    void closeStream(SoapySDR::Stream* stream);
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double& fullScale) const;
    int readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags, long long& timeNs, const long timeoutUs = 100000);
    double getSampleRate(const int direction, const size_t channel) const;
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const;

    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;

    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string& name) const;
    int deactivateStream(SoapySDR::Stream* stream, const int, const long long);
    int activateStream(SoapySDR::Stream* stream, const int, const long long, const size_t);
    void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs& args = SoapySDR::Kwargs());
    void setFrequency(const int direction, const size_t channel, const std::string& name, const double frequency, const SoapySDR::Kwargs& args = SoapySDR::Kwargs());
    void setBandwidth(const int direction, const size_t channel, const double bw);
    void setSampleRate(const int direction, const size_t channel, const double rate);
    double getFrequency(const int direction, const size_t channel) const;
    void setGainMode(const int direction, const size_t channel, const bool automatic);
    bool hasGainMode(const int direction, const size_t channel) const;
    bool getGainMode(const int direction, const size_t channel) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string& name) const;
    void setGain(const int direction, const size_t channel, const double value);
    void setGain(const int direction, const size_t channel, const std::string& name, const double value);
    MyDevice(int i);
    ~MyDevice();

   private:
    AD9361* device;
};
std::string MyDevice::getDriverKey(void) const {
    return "driver_key";
}
std::string MyDevice::getHardwareKey(void) const {
    return "hardware_key";
}

size_t MyDevice::getNumChannels(const int direction) const {
    // if (direction == SOAPY_SDR_TX) {
    //     return 2;
    // }
    if (direction == SOAPY_SDR_RX) {
        return 1;
    }

    return 0;
}

bool MyDevice::getFullDuplex(const int direction, const size_t channel) const {
    (void)direction;
    (void)channel;
    return true;
}

SoapySDR::Stream* MyDevice::setupStream(const int direction, const std::string& format, const std::vector<size_t>& channels, const SoapySDR::Kwargs& args) {
    (void)args;
    if (direction != SOAPY_SDR_RX)
        throw std::runtime_error("Only RX direction is supported");
    if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32)
        throw std::runtime_error("Only CS16 and CF32 formats are supported");
    std::vector<size_t> chans = channels.empty() ? std::vector<size_t>{0} : channels;
    if (chans.size() > 1) {
        throw std::runtime_error("currently only one channel is supported");
    }

    DummyStream* stream = new DummyStream(device->device_input);
    stream->direction = direction;
    stream->channels = chans;
    stream->format = format;

    SoapySDR_logf(SOAPY_SDR_DEBUG, "setupStream done ");

    SoapySDR_logf(SOAPY_SDR_DEBUG, "setupStream done %f", getFrequency(SOAPY_SDR_RX, 0));
    return reinterpret_cast<SoapySDR::Stream*>(stream);
}

void MyDevice::closeStream(SoapySDR::Stream* stream) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "close stream  ");
    DummyStream* s = reinterpret_cast<DummyStream*>(stream);
    delete s;
}

std::vector<std::string> MyDevice::getStreamFormats(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getStreamFormats ");
    (void)direction;
    (void)channel;

    std::vector<std::string> formats;

    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);
    return formats;
}

std::string MyDevice::getNativeStreamFormat(const int direction, const size_t channel, double& fullScale) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getNativeFormat ");
    (void)direction;
    (void)channel;
    (void)fullScale;
    return SOAPY_SDR_CS16;
}

int MyDevice::readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags, long long& timeNs, const long timeoutUs) {
    (void)flags;
    (void)timeNs;

    SoapySDR_logf(SOAPY_SDR_TRACE, "readStream ");
    DummyStream* s = reinterpret_cast<DummyStream*>(stream);
    if (!s->active) {
        if (timeoutUs > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(timeoutUs));
        }
        return SOAPY_SDR_TIMEOUT;
    }
    size_t output_counter = 0;
    int16_t* buffer = reinterpret_cast<int16_t*>(buffs[0]);
    void* rawBuffer = buffs[0];
    SoapySDR_logf(SOAPY_SDR_TRACE, "before while");
    while (output_counter < numElems) {
        if (s->current_buffer_finished) {
            s->prepare_next_block();
        }
        size_t rx_sample = s->get_rx_sample_size();

        SoapySDR_logf(SOAPY_SDR_TRACE, "before for");
        while (s->p_dat < s->p_end) {
            int16_t i = s->p_dat[0];
            int16_t q = s->p_dat[1];

            // *buffer = i;
            // buffer++;
            // *buffer = q;
            // buffer++;
            output_counter++;
            if (s->format == SOAPY_SDR_CF32) {
                float* buffer = reinterpret_cast<float*>(rawBuffer);
                buffer[2 * output_counter] = static_cast<float>(i) / 32768.0f;
                buffer[2 * output_counter + 1] = static_cast<float>(q) / 32768.0f;
            } else {
                int16_t* buffer = reinterpret_cast<int16_t*>(rawBuffer);
                buffer[2 * output_counter] = i;
                buffer[2 * output_counter + 1] = q;
            }

            if (output_counter >= numElems) {
                SoapySDR_logf(SOAPY_SDR_TRACE, "early return");
                return static_cast<int>(numElems);
            }
            s->p_dat += rx_sample / sizeof(*s->p_dat);
        }
        s->current_buffer_finished = true;
    }
    SoapySDR_logf(SOAPY_SDR_TRACE, "after while");
    return static_cast<int>(numElems);
}

double MyDevice::getSampleRate(const int direction, const size_t channel) const {
    (void)channel;
    return device->get_sample_rate(direction == SOAPY_SDR_TX);
}

SoapySDR::RangeList MyDevice::getSampleRateRange(const int direction, const size_t channel) const {
    (void)direction;
    (void)channel;
    // TODO: show real numbers
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getSampleRateRange");
    SoapySDR::RangeList ranges;
    ranges.push_back(SoapySDR::Range(0, 60000000));

    return ranges;
}

std::vector<double> MyDevice::listSampleRates(const int direction, const size_t channel) const {
    std::vector<double> options;

    options.push_back(65105);  // 25M/48/8+1
    options.push_back(1e6);
    options.push_back(2e6);
    options.push_back(3e6);
    options.push_back(4e6);
    options.push_back(5e6);
    options.push_back(6e6);
    options.push_back(7e6);
    options.push_back(8e6);
    options.push_back(9e6);
    options.push_back(10e6);
    return (options);
}

void MyDevice::setGain(const int direction, const size_t channel, const double value) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setGain");
    device->set_gain(value, direction == SOAPY_SDR_TX);
}

void MyDevice::setGain(const int direction, const size_t channel, const std::string& name, const double value) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setGain");
    device->set_gain(value, direction == SOAPY_SDR_TX);
}
SoapySDR::Range MyDevice::getGainRange(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getGainRange");
    return SoapySDR::Range(-20, 90);
}

SoapySDR::Range MyDevice::getGainRange(const int direction, const size_t channel, const std::string& name) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getGainRange2");
    return SoapySDR::Range(-20, 90);
}
void MyDevice::setGainMode(const int direction, const size_t channel, const bool automatic) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setGainMode ");
    device->set_gain_mode(direction == SOAPY_SDR_TX, automatic);
}

bool MyDevice::hasGainMode(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "hasGainMode ");
    return true;
}

bool MyDevice::getGainMode(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getGainMode ");
    return device->get_gain_mode(direction == SOAPY_SDR_TX);
}
SoapySDR::KwargsList findMyDevice(const SoapySDR::Kwargs& args) {
    std::vector<SoapySDR::Kwargs> results;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "findMyDevice");
    (void)args;
    SoapySDR::Kwargs options;
    options["device"] = "MyDevice";
    results.push_back(options);
    // SoapySDR_logf(SOAPY_SDR_DEBUG, "hello world %s", args.at("test").c_str());
    SoapySDR_logf(SOAPY_SDR_DEBUG, "hello world ");

    return results;
}

SoapySDR::Device* makeMyDevice(const SoapySDR::Kwargs& args) {
    (void)args;
    SoapySDR::setLogLevel(SOAPY_SDR_DEBUG);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "makeMyDevice ");
    // create an instance of the device object given the args
    // here we will translate args into something used in the constructor
    return new MyDevice(1);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerMyDevice("my_device", &findMyDevice, &makeMyDevice, SOAPY_SDR_ABI_VERSION);

SoapySDR::RangeList MyDevice::getFrequencyRange(const int direction, const size_t channel, const std::string& name) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getFrequencyRange");

    if (name == "RF") {
        return {SoapySDR::Range(MHZ(70), GHZ(6))};
    }

    return {};
    // return (SoapySDR::RangeList(1, SoapySDR::Range(MHZ(70), GHZ(6))));
}
std::vector<std::string> MyDevice::listFrequencies(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "listFrequencies");
    std::vector<std::string> names;
    names.push_back("RF");
    return (names);
}
int MyDevice::activateStream(SoapySDR::Stream* stream, const int, const long long, const size_t) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "activateStream");
    auto* s = reinterpret_cast<DummyStream*>(stream);
    s->active = true;
    return 0;
}

void MyDevice::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs& args) {
    (void)channel;
    (void)frequency;
    (void)args;
    SoapySDR_logf(SOAPY_SDR_WARNING, "setFrequency %d", direction);
    device->set_frequency(static_cast<long long>(frequency), direction == SOAPY_SDR_TX);
}

void MyDevice::setFrequency(const int direction, const size_t channel, const std::string& name, const double frequency, const SoapySDR::Kwargs& args) {
    (void)channel;
    (void)frequency;
    (void)args;
    SoapySDR_logf(SOAPY_SDR_WARNING, "setFrequency %s %d", name.c_str(), direction);
    device->set_frequency(static_cast<long long>(frequency), direction == SOAPY_SDR_TX);
}

void MyDevice::setBandwidth(const int direction, const size_t channel, const double bw) {
    device->set_bandwidth_frequency(static_cast<long long>(bw), direction == SOAPY_SDR_TX);
}

void MyDevice::setSampleRate(const int direction, const size_t channel, const double rate) {
    device->set_sample_rate(static_cast<long long>(rate), direction == SOAPY_SDR_TX);
}

double MyDevice::getFrequency(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getFrequency ");
    return device->get_frequency(direction == SOAPY_SDR_TX);
}



MyDevice::MyDevice(int i) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "MyDevice Constructor %d", i);
    device = new AD9361("ip:192.168.88.194");
}

MyDevice::~MyDevice() {
}

int MyDevice::deactivateStream(SoapySDR::Stream* stream, const int, const long long) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "deactivateStream");
    auto* s = reinterpret_cast<DummyStream*>(stream);
    s->active = false;
    return 0;
}
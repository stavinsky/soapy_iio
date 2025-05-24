#include "IIODevice.hpp"

#include <memory>
#include <thread>

#include "Stream.hpp"
#include "ad9361.hpp"

#define MHZ(x) (x * 1000000.0)
#define GHZ(x) (x * 1000000000.0)

std::string IIODevice::getDriverKey(void) const {
    return "my_device";
}
std::string IIODevice::getHardwareKey(void) const {
    return "fmcomms3";
}
SoapySDR::Kwargs IIODevice::getChannelInfo(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getChannelInfo");
    SoapySDR::Kwargs info;
    if (direction == SOAPY_SDR_RX) {
        info["label"] = (channel == 0) ? "RX0" : "RX1";
        info["name"] = "iio:rx" + std::to_string(channel);
        info["rf_path"] = "main";
        info["direction"] = "RX";
    }
    return info;
}
size_t IIODevice::getNumChannels(const int direction) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getChannelInfo");
    // if (direction == SOAPY_SDR_TX) {
    //     return 2;
    // }
    if (direction == SOAPY_SDR_RX) {
        return 1;
    }

    return 0;
}

bool IIODevice::getFullDuplex(const int direction, const size_t channel) const {
    (void)direction;
    (void)channel;
    return true;
}

SoapySDR::Stream* IIODevice::setupStream(const int direction, const std::string& format, const std::vector<size_t>& channels, const SoapySDR::Kwargs& args) {
    for (auto ch : channels) {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "  -> channel %zu", ch);
    }
    (void)args;
    if (direction != SOAPY_SDR_RX)
        throw std::runtime_error("Only RX direction is supported");
    if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32)
        throw std::runtime_error("Only CS16 and CF32 formats are supported");
    std::vector<size_t> chans = channels.empty() ? std::vector<size_t>{0} : channels;
    if (chans.size() > 1) {
        throw std::runtime_error("currently only one channel is supported");
    }

    Stream* stream = new Stream(device->device_input);
    stream->direction = direction;
    stream->channels = chans;
    stream->format = format;

    SoapySDR_logf(SOAPY_SDR_DEBUG, "setupStream done ");

    SoapySDR_logf(SOAPY_SDR_DEBUG, "setupStream done %f", getFrequency(SOAPY_SDR_RX, 0));
    return reinterpret_cast<SoapySDR::Stream*>(stream);
}

void IIODevice::closeStream(SoapySDR::Stream* stream) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "close stream  ");
    Stream* s = reinterpret_cast<Stream*>(stream);
    std::lock_guard<std::mutex> lock(s->mtx);
    if (s) {
        delete s;
    }
    // device.reset();
    SoapySDR_logf(SOAPY_SDR_DEBUG, "close stream  done");
}

std::vector<std::string> IIODevice::getStreamFormats(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getStreamFormats ");
    (void)direction;
    (void)channel;

    std::vector<std::string> formats;

    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);
    return formats;
}

std::string IIODevice::getNativeStreamFormat(const int direction, const size_t channel, double& fullScale) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getNativeFormat ");
    (void)direction;
    (void)channel;
    fullScale = 8191.0;
    return SOAPY_SDR_CS16;
}

int IIODevice::readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags, long long& timeNs, const long timeoutUs) {
    flags = 0;
    timeNs = 0;

    SoapySDR_logf(SOAPY_SDR_TRACE, "readStream ");
    Stream* s = reinterpret_cast<Stream*>(stream);

    size_t output_counter = 0;
    void* rawBuffer = buffs[0];
    SoapySDR_logf(SOAPY_SDR_TRACE, "before while");
    while (output_counter < numElems) {
        if (!s->active.load(std::memory_order_acquire)) {
            if (timeoutUs > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(timeoutUs));
            }
            return SOAPY_SDR_TIMEOUT;
        }
        s->active.load(std::memory_order_acquire);
        if (s->current_buffer_finished) {
            s->prepare_next_block();
        }
        size_t rx_sample = s->get_rx_sample_size();

        SoapySDR_logf(SOAPY_SDR_TRACE, "before for");
        while (s->p_dat < s->p_end) {
            int16_t i = s->p_dat[0];
            int16_t q = s->p_dat[1];

            if (s->format == SOAPY_SDR_CF32) {
                float* buffer = reinterpret_cast<float*>(rawBuffer);
                buffer[2 * output_counter] = static_cast<float>(i) / 32768.0f;
                buffer[2 * output_counter + 1] = static_cast<float>(q) / 32768.0f;
            } else {
                int16_t* buffer = reinterpret_cast<int16_t*>(rawBuffer);
                buffer[2 * output_counter] = i;
                buffer[2 * output_counter + 1] = q;
            }
            output_counter++;

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

size_t IIODevice::getStreamMTU(SoapySDR::Stream* stream) const {
    return BLOCK_SIZE;
}

double IIODevice::getSampleRate(const int direction, const size_t channel) const {
    (void)channel;
    return device->get_sample_rate(direction == SOAPY_SDR_TX);
}

SoapySDR::RangeList IIODevice::getSampleRateRange(const int direction, const size_t channel) const {
    (void)direction;
    (void)channel;
    // TODO: show real numbers
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getSampleRateRange");
    SoapySDR::RangeList ranges;
    ranges.push_back(SoapySDR::Range(MHZ(3), MHZ(61)));

    return ranges;
}
std::vector<double> IIODevice::listSampleRates(const int direction, const size_t channel) const {
    (void)direction;
    (void)channel;

    std::vector<double> options;

    options.push_back(3e6);
    options.push_back(4e6);
    options.push_back(5e6);
    options.push_back(6e6);
    options.push_back(7e6);
    options.push_back(8e6);
    options.push_back(9e6);
    options.push_back(10e6);
    options.push_back(20e6);
    options.push_back(25e6);
    return (options);
}
void IIODevice::setGain(const int direction, const size_t channel, const double value) {
    (void)channel;  // TODO: second channel
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setGain");
    device->set_gain(value, direction == SOAPY_SDR_TX);
}

void IIODevice::setGain(const int direction, const size_t channel, const std::string& name, const double value) {
    (void)name;
    (void)channel;  // TODO: second channel
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setGain");
    device->set_gain(value, direction == SOAPY_SDR_TX);
}
double IIODevice::getGain(const int direction, const size_t channel) const {
    (void)channel;  // TODO: second channel
    return device->get_gain(direction == SOAPY_SDR_TX);
}
double IIODevice::getGain(const int direction, const size_t channel, const std::string& name) const {
    (void)channel;  // TODO: second channel
    (void)name;
    return device->get_gain(direction == SOAPY_SDR_TX);
}

SoapySDR::Range IIODevice::getGainRange(const int direction, const size_t channel) const {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getGainRange");
    return SoapySDR::Range(-20, 90);
}

SoapySDR::Range IIODevice::getGainRange(const int direction, const size_t channel, const std::string& name) const {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    (void)name;
    SoapySDR_logf(SOAPY_SDR_TRACE, "getGainRange2");
    return SoapySDR::Range(-20, 90);
}
void IIODevice::setGainMode(const int direction, const size_t channel, const bool automatic) {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setGainMode ");
    device->set_gain_mode(direction == SOAPY_SDR_TX, automatic);
}

bool IIODevice::hasGainMode(const int direction, const size_t channel) const {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_DEBUG, "hasGainMode ");
    return true;
}

bool IIODevice::getGainMode(const int direction, const size_t channel) const {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getGainMode ");
    return device->get_gain_mode(direction == SOAPY_SDR_TX);
}

SoapySDR::RangeList IIODevice::getFrequencyRange(const int direction, const size_t channel, const std::string& name) const {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getFrequencyRange");

    if (name == "RF") {
        return {SoapySDR::Range(MHZ(70), GHZ(6))};
    }

    return {};
    // return (SoapySDR::RangeList(1, SoapySDR::Range(MHZ(70), GHZ(6))));
}
SoapySDR::RangeList IIODevice::getFrequencyRange(const int direction, const size_t channel) const {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getFrequencyRange without name");

    return {SoapySDR::Range(MHZ(70), GHZ(6))};
}
std::vector<std::string> IIODevice::listFrequencies(const int direction, const size_t channel) const {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_DEBUG, "listFrequencies");
    std::vector<std::string> names;
    names.push_back("RF");
    return (names);
}
int IIODevice::activateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs, const size_t numElems) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "activateStream");
    auto* s = reinterpret_cast<Stream*>(stream);
    s->rx_channel_enable();

    return 0;
}

void IIODevice::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs& args) {
    (void)channel;
    (void)frequency;
    (void)args;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setFrequency %d", direction);
    device->set_frequency(static_cast<long long>(frequency), direction == SOAPY_SDR_TX);
}

void IIODevice::setFrequency(const int direction, const size_t channel, const std::string& name, const double frequency, const SoapySDR::Kwargs& args) {
    (void)channel;
    (void)frequency;
    (void)args;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setFrequency %s %d", name.c_str(), direction);
    device->set_frequency(static_cast<long long>(frequency), direction == SOAPY_SDR_TX);
}

void IIODevice::setSampleRate(const int direction, const size_t channel, const double rate) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setSampleRate channel:%d frequency: %f", channel, rate);
    (void)channel;  // TODO: second channel
    device->set_sample_rate(static_cast<long long>(rate), direction == SOAPY_SDR_TX);
}

double IIODevice::getFrequency(const int direction, const size_t channel) const {
    (void)channel;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getFrequency ");
    return device->get_frequency(direction == SOAPY_SDR_TX);
}

std::vector<std::string> IIODevice::listGains(const int direction, const size_t channel) const {
    (void)channel;    // TODO: second channel
    (void)direction;  // TODO: Direction
    SoapySDR_logf(SOAPY_SDR_TRACE, "listGains");
    return {"hardwaregain"};
}

IIODevice::IIODevice() {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "MyDevice Constructor ");
    device = new AD9361("ip:192.168.88.194");
}

IIODevice::~IIODevice() {
    if (device) {
        delete device;
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "MyDevice Destructor ");
}

int IIODevice::deactivateStream(SoapySDR::Stream* stream, const int, const long long) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "deactivateStream");
    auto* s = reinterpret_cast<Stream*>(stream);

    s->rx_channel_disable();

    return 0;
}

double IIODevice::getBandwidth(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getBandwidth");
    return device->get_bandwidth_frequency(channel, direction == SOAPY_SDR_TX);
}

std::vector<double> IIODevice::listBandwidths(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "listBandwidths");
    (void)direction;
    (void)channel;
    std::vector<double> list;
    list.push_back(200000);
    list.push_back(MHZ(1));
    list.push_back(MHZ(2));
    list.push_back(MHZ(3));
    list.push_back(MHZ(4));
    list.push_back(MHZ(5));
    list.push_back(MHZ(6));
    list.push_back(MHZ(7));
    list.push_back(MHZ(8));
    list.push_back(MHZ(9));
    list.push_back(MHZ(10));
    list.push_back(MHZ(12));
    list.push_back(MHZ(16));
    list.push_back(MHZ(20));
    list.push_back(MHZ(30));
    list.push_back(MHZ(40));
    return list;
}

SoapySDR::RangeList IIODevice::getBandwidthRange(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getBandwidthRange");
    (void)direction;
    (void)channel;
    SoapySDR::RangeList range_list = SoapySDR::RangeList();
    range_list.push_back(SoapySDR::Range(200000, MHZ(50)));
    return range_list;
}
void IIODevice::setBandwidth(const int direction, const size_t channel, const double bw) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setBandwidth");
    (void)channel;  // TODO: second channel
    device->set_bandwidth_frequency(static_cast<long long>(bw), direction == SOAPY_SDR_TX);
}

std::vector<std::string> IIODevice::listAntennas(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "listAntennas");
    std::vector<std::string> list;
    if (direction == SOAPY_SDR_RX and channel == 0) {
        list.push_back("RX0");
    }
    return list;
}

void IIODevice::setAntenna(const int direction, const size_t channel, const std::string& name) {
    (void)direction;
    (void)channel;
    (void)name;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setAntenna");
}

std::string IIODevice::getAntenna(const int direction, const size_t channel) const {
    (void)direction;
    (void)channel;
    return std::string("RX0");
}

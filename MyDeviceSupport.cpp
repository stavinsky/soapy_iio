extern "C" {
#include <iio/iio-debug.h>
}

#include <iio/iio.h>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#define BLOCK_SIZE (32000 * 4)  // TODO: make parameter
#define MHZ(x) ((long long)(x * 1000000.0 + .5))
#define GHZ(x) ((long long)(x * 1000000000.0 + .5))
#define IIO_ENSURE(expr)                                                             \
    {                                                                                \
        if (!(expr)) {                                                               \
            (void)fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
            (void)abort();                                                           \
        }                                                                            \
    }

enum iodev { RX,
             TX };

struct stream_cfg {
    long long bw_hz;     // Analog banwidth in Hz
    long long fs_hz;     // Baseband sample rate in Hz
    long long lo_hz;     // Local oscillator frequency in Hz
    const char* rfport;  // Port name
};

struct DummyStream {
    int direction;
    std::string format;
    std::vector<size_t> channels;
    struct iio_device* rx;
    size_t rx_sample_sz;
    struct stream_cfg rxcfg;
    struct iio_context* ctx;
    struct iio_channel* rx_ch_i;
    struct iio_channel* rx_ch_q;
    struct iio_channels_mask* rx_mask;
    struct iio_buffer* rx_buffer;
    struct iio_stream* rx_stream;
    bool current_buffer_finished = true;

    int16_t* p_dat;
    int16_t* p_end;
    bool active = false;
};

static bool get_ad9361_stream_dev(enum iodev d, struct iio_device** dev, iio_context* ctx) {
    switch (d) {
        case TX:
            *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
            return *dev != NULL;
        case RX:
            *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
            return *dev != NULL;
        default:
            IIO_ENSURE(0);
            return false;
    }
}
static struct iio_device* get_ad9361_phy(iio_context* ctx) {
    struct iio_device* dev = iio_context_find_device(ctx, "ad9361-phy");
    IIO_ENSURE(dev && "No ad9361-phy found");
    return dev;
}
static char* get_ch_name(const char* type, int id) {
    static char tmpstr[64];
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return tmpstr;
}

static bool get_phy_chan(enum iodev d, int chid, struct iio_channel** chn, iio_context* ctx) {
    switch (d) {
        case RX:
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), false);
            return *chn != NULL;
        case TX:
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), true);
            return *chn != NULL;
        default:
            IIO_ENSURE(0);
            return false;
    }
}
static void shutdown(void) {
    printf("* Destroying streams\n");
    // TODO: make sure to clean up.
    exit(0);
}
static void errchk(int v, const char* what) {
    if (v < 0) {
        fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
        shutdown();
    }
}

static void wr_ch_lli(struct iio_channel* chn, const char* what, long long val) {
    const struct iio_attr* attr = iio_channel_find_attr(chn, what);

    errchk(attr ? iio_attr_write_longlong(attr, val) : -ENOENT, what);
}
/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(enum iodev d, struct iio_channel** chn, iio_context* ctx) {
    switch (d) {
            // LO chan is always output, i.e. true
        case RX:
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 0), true);
            return *chn != NULL;
        case TX:
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 1), true);
            return *chn != NULL;
        default:
            IIO_ENSURE(0);
            return false;
    }
}
bool cfg_ad9361_streaming_ch(struct stream_cfg* cfg, enum iodev type, int chid, iio_context* ctx) {
    const struct iio_attr* attr;
    struct iio_channel* chn = NULL;

    // Configure phy and lo channels
    printf("* Acquiring AD9361 phy channel %d\n", chid);
    if (!get_phy_chan(type, chid, &chn, ctx)) {
        return false;
    }

    attr = iio_channel_find_attr(chn, "rf_port_select");
    if (attr)
        errchk(iio_attr_write_string(attr, cfg->rfport), cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth", cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

    // Configure LO channel
    printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(type, &chn, ctx)) {
        return false;
    }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}
static bool get_ad9361_stream_ch(enum iodev d, struct iio_device* dev, int chid, struct iio_channel** chn) {
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != NULL;
}

/***********************************************************************
 * Device interface
 **********************************************************************/
using namespace std;
class MyDevice : public SoapySDR::Device {
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
    void setGain(const int direction, const size_t channel, const double value);
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string& name) const;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string& name) const;
    int deactivateStream(SoapySDR::Stream* stream, const int, const long long);
    int activateStream(SoapySDR::Stream* stream, const int, const long long, const size_t);

    // Implement constructor with device specific arguments...

    // Implement all applicable virtual methods from SoapySDR::Device
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
        return 2;
    }

    return 0;
}

bool MyDevice::getFullDuplex(const int direction, const size_t channel) const {
    (void)direction;
    (void)channel;
    return true;
}

SoapySDR::Stream* MyDevice::setupStream(const int direction, const std::string& format, const std::vector<size_t>& channels, const SoapySDR::Kwargs& args) {
    // SoapySDR::setLogLevel(SOAPY_SDR_DEBUG);
    if (direction != SOAPY_SDR_RX)
        throw std::runtime_error("Only RX direction is supported");
    if (format != SOAPY_SDR_CS16)
        throw std::runtime_error("Only CS16 format is supported");
    std::vector<size_t> chans = channels.empty() ? std::vector<size_t>{0} : channels;
    if (chans.size() > 1) {
        throw std::runtime_error("currently only one channel is supported");
    }
    DummyStream* stream = new DummyStream;
    stream->direction = direction;
    stream->format = format;
    stream->channels = chans;
    stream->rxcfg.bw_hz = MHZ(20);        // 2 MHz rf bandwidth
    stream->rxcfg.fs_hz = MHZ(4.4);       // 2.5 MS/s rx sample rate
    stream->rxcfg.lo_hz = MHZ(103.2);     // 2.5 GHz rf frequency
    stream->rxcfg.rfport = "A_BALANCED";  // port A (select for rf freq.)

    int err;
    stream->ctx = iio_create_context(NULL, "ip:192.168.88.194");
    err = iio_err(stream->ctx);
    IIO_ENSURE(!err && "No context");
    IIO_ENSURE(iio_context_get_devices_count(stream->ctx) > 0 && "No devices");
    IIO_ENSURE(get_ad9361_stream_dev(RX, &stream->rx, stream->ctx) && "No tx dev found");
    IIO_ENSURE(cfg_ad9361_streaming_ch(&stream->rxcfg, RX, 0, stream->ctx) && "RX port 0 not found");

    IIO_ENSURE(get_ad9361_stream_ch(RX, stream->rx, 0, &stream->rx_ch_i) && "RX chan i not found");
    IIO_ENSURE(get_ad9361_stream_ch(RX, stream->rx, 1, &stream->rx_ch_q) && "RX chan q not found");

    stream->rx_mask = iio_create_channels_mask(iio_device_get_channels_count(stream->rx));
    if (!stream->rx_mask) {
        fprintf(stderr, "Unable to alloc channels mask\n");
        shutdown();
    }
    printf("* Enabling IIO streaming channels\n");
    iio_channel_enable(stream->rx_ch_i, stream->rx_mask);
    iio_channel_enable(stream->rx_ch_q, stream->rx_mask);

    stream->rx_buffer = iio_device_create_buffer(stream->rx, 0, stream->rx_mask);
    err = iio_err(stream->rx_buffer);
    if (err) {
        stream->rx_buffer = NULL;
        dev_perror(stream->rx, err, "Could not create RX buffer");
        shutdown();
    }
    stream->rx_stream = iio_buffer_create_stream(stream->rx_buffer, 4, BLOCK_SIZE);
    err = iio_err(stream->rx_stream);
    if (err) {
        stream->rx_stream = NULL;
        dev_perror(stream->rx, iio_err(stream->rx_stream), "Could not create RX stream");
        shutdown();
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setupStream done ");
    return reinterpret_cast<SoapySDR::Stream*>(stream);
}

void MyDevice::closeStream(SoapySDR::Stream* stream) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "close stream  ");
    delete reinterpret_cast<DummyStream*>(stream);
}

std::vector<std::string> MyDevice::getStreamFormats(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getStreamFormats ");
    (void)direction;
    (void)channel;

    std::vector<std::string> formats;

    formats.push_back(SOAPY_SDR_CS16);
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
    SoapySDR_logf(SOAPY_SDR_DEBUG, "readStream ");
    DummyStream* s = reinterpret_cast<DummyStream*>(stream);
    if (!s->active) {
        if (timeoutUs > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(timeoutUs));
        }
        return SOAPY_SDR_TIMEOUT;
    }
    int err;
    size_t output_counter = 0;
    int16_t* buffer = reinterpret_cast<int16_t*>(buffs[0]);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "before while");
    while (output_counter < numElems) {
        int16_t* p_start;
        ptrdiff_t p_inc;

        if (s->current_buffer_finished) {
            const iio_block* rxblock = iio_stream_get_next_block(s->rx_stream);
            err = iio_err(rxblock);
            if (err) {
                ctx_perror(s->ctx, err, "Unable to receive block");
                throw std::runtime_error("unable to receive block");
            }
            s->p_end = reinterpret_cast<int16_t*>(iio_block_end(rxblock));
            p_start = reinterpret_cast<int16_t*>(iio_block_first(rxblock, s->rx_ch_i));
            s->current_buffer_finished = false;
        } else {
            p_start = s->p_dat;
        }
        size_t rx_sample = iio_device_get_sample_size(s->rx, s->rx_mask);

        SoapySDR_logf(SOAPY_SDR_DEBUG, "before for");
        for (s->p_dat = p_start; s->p_dat < s->p_end; s->p_dat += rx_sample / sizeof(*s->p_dat)) {
            int16_t i = s->p_dat[0];
            int16_t q = s->p_dat[1];

            *buffer = i;
            buffer++;
            *buffer = q;
            buffer++;
            output_counter++;

            if (output_counter >= numElems) {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "early return");
                return static_cast<int>(numElems);
            }

            // *buffer++ = static_cast<int16_t>(i); TODO: check this version
            // *buffer++ = static_cast<int16_t>(q);
        }
        s->current_buffer_finished = true;
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "after while");
    return static_cast<int>(numElems);
}

double MyDevice::getSampleRate(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getSampleRate");
    return 4400000;
}

SoapySDR::RangeList MyDevice::getSampleRateRange(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getSampleRateRange");
    SoapySDR::RangeList ranges;
    ranges.push_back(SoapySDR::Range(0, 60000000));

    return ranges;
}

void MyDevice::setGain(const int direction, const size_t channel, const double value) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setGain");
}

static std::vector<SoapySDR::Kwargs> results;
SoapySDR::KwargsList findMyDevice(const SoapySDR::Kwargs& args) {
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
    // create an instance of the device object given the args
    // here we will translate args into something used in the constructor
    return new MyDevice();
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerMyDevice("my_device", &findMyDevice, &makeMyDevice, SOAPY_SDR_ABI_VERSION);

SoapySDR::Range MyDevice::getGainRange(const int direction, const size_t channel) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getGainRange");
    return SoapySDR::Range(-20, 90);
}

SoapySDR::Range MyDevice::getGainRange(const int direction, const size_t channel, const std::string& name) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getGainRange2");
    return SoapySDR::Range(-20, 90);
}

SoapySDR::RangeList MyDevice::getFrequencyRange(const int direction, const size_t channel, const std::string& name) const {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "getFrequencyRange");
    // SoapySDR::RangeList ranges;
    // ranges.push_back(SoapySDR::Range(0, 6000000));

    // return ranges;
    // return SoapySDR::RangeList();
    return (SoapySDR::RangeList(1, SoapySDR::Range(70000000, 6000000000ull)));
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

int MyDevice::deactivateStream(SoapySDR::Stream* stream, const int, const long long) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "deactivateStream");
    auto* s = reinterpret_cast<DummyStream*>(stream);
    s->active = false;
    return 0;
}
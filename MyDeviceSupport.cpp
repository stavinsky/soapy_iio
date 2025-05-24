
// extern "C" {
// #include <iio/iio-debug.h>
// }
// #include <iio/iio.h>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <chrono>
#include <iostream>
#include <string>

#include "IIODevice.hpp"

#define IIO_ENSURE(expr)                                                             \
    {                                                                                \
        if (!(expr)) {                                                               \
            (void)fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
            (void)abort();                                                           \
        }                                                                            \
    }

SoapySDR::KwargsList findMyDevice(const SoapySDR::Kwargs& args) {
    std::vector<SoapySDR::Kwargs> results;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "findMyDevice");
    (void)args;
    SoapySDR::Kwargs options;
    options["device"] = "MyDevice";
    options["driver"] = "my_device";
    options["label"] = "my_device";
    results.push_back(options);

    return results;
}
SoapySDR::Device* makeMyDevice(const SoapySDR::Kwargs& args) {
    (void)args;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "makeMyDevice ");

    return new IIODevice();
}

static SoapySDR::Registry registerMyDevice("my_device", &findMyDevice, &makeMyDevice, SOAPY_SDR_ABI_VERSION);

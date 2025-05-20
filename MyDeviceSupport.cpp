
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

static SoapySDR::Registry registerMyDevice("my_device", &findMyDevice, &makeMyDevice, SOAPY_SDR_ABI_VERSION);

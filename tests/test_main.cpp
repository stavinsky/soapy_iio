
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Types.hpp>
#include <cassert>
#include <cstdio>  //stdandard output
#include <cstdlib>
#include <iostream>
#include <map>     // std::map< ... , ... >
#include <string>  // std::string
#include <vector>  // std::vector<...>

int main() {
    const uint16_t channel = 0;
    SoapySDR::setLogLevel(SOAPY_SDR_DEBUG);
    std::string loading_module_errors = SoapySDR::loadModule("../libMyDevice.so");
    assert(loading_module_errors.empty());
    SoapySDR::KwargsList devices = SoapySDR::Device::enumerate();
    assert(devices.size() == 1);
    SoapySDR::Kwargs::iterator it;

    for (size_t i = 0; i < devices.size(); ++i) {
        printf("Found device #%zu: ", i);
        for (it = devices[i].begin(); it != devices[i].end(); ++it) {
            printf("%s = %s\n", it->first.c_str(), it->second.c_str());
        }
        printf("\n");
    }
    SoapySDR::Kwargs args = devices[0];
    SoapySDR::Device *sdr = SoapySDR::Device::make(args);

    assert(sdr != NULL);

    std::vector<std::string> str_list;
    str_list = sdr->listAntennas(SOAPY_SDR_RX, channel);
    printf("Rx antennas: ");
    for (int i = 0; i < str_list.size(); ++i)
        printf("%s\n", str_list[i].c_str());
    printf("\n");

    assert(str_list.size() >= 1);  // list of antennas
    sdr->setAntenna(SOAPY_SDR_RX, 0, std::string("A_BALANCED"));
    std::string str = sdr->getAntenna(SOAPY_SDR_RX, 0);
    printf("current antenna is %s\n", str.c_str());
    assert(str == "A_BALANCED");

    str_list = sdr->listGains(SOAPY_SDR_RX, channel);
    printf("Rx Gains: ");
    for (int i = 0; i < str_list.size(); ++i)
        printf("%s, ", str_list[i].c_str());
    printf("\n");

    assert(str_list.size() > 0);  // list of gains only hardware gain in our case

    SoapySDR::RangeList ranges = sdr->getFrequencyRange(SOAPY_SDR_RX, channel);
    printf("Rx freq ranges: ");
    for (int i = 0; i < ranges.size(); ++i)
        printf("[%g Hz -> %g Hz], ", ranges[i].minimum(), ranges[i].maximum());
    printf("\n");

    assert(str_list.size() > 0);  // frequency_ranges without providing name

    ranges = sdr->getFrequencyRange(SOAPY_SDR_RX, channel, "RF");
    printf("Rx freq ranges: ");
    for (int i = 0; i < ranges.size(); ++i)
        printf("[%g Hz -> %g Hz], ", ranges[i].minimum(), ranges[i].maximum());
    printf("\n");

    assert(str_list.size() > 0);  // frequency_ranges with provided name

    double freq = 0;
    const double expected_sample_rate = 10e6;
    sdr->setSampleRate(SOAPY_SDR_RX, channel, expected_sample_rate);
    freq = sdr->getSampleRate(SOAPY_SDR_RX, channel);
    printf("set sample rate %f HZ, got sample rate %f HZ\n", expected_sample_rate, freq);
    assert(std::abs(freq - expected_sample_rate) < 100);

    const double expected_freq = 100e6;
    sdr->setFrequency(SOAPY_SDR_RX, channel, expected_freq);
    freq = sdr->getFrequency(SOAPY_SDR_RX, channel);
    printf("set freq %f HZ, got freq %f HZ\n", expected_freq, freq);
    assert(std::abs(freq - expected_freq) < 100);

    const double expected_bandwidth = 20e6;
    sdr->setBandwidth(SOAPY_SDR_RX, channel, expected_bandwidth);
    freq = sdr->getBandwidth(SOAPY_SDR_RX, channel);
    printf("set freq %f HZ, got freq %f HZ\n", expected_bandwidth, freq);
    assert(std::abs(freq - expected_bandwidth) < 100);

    const double expected_gain = 3.3;
    sdr->setGain(SOAPY_SDR_RX, channel, expected_gain);
    freq = sdr->getGain(SOAPY_SDR_RX, channel);
    printf("set freq %f HZ, got freq %f HZ\n", expected_gain, freq);
    assert(std::abs(freq - expected_gain) < 100);

    double fullscale;
    std::string native_format = sdr->getNativeStreamFormat(SOAPY_SDR_RX, channel, fullscale);
    assert(fullscale == 8191.0);  // 14 bit
    assert(native_format == SOAPY_SDR_CS16);

    std::vector<std::string> formats = sdr->getStreamFormats(SOAPY_SDR_RX, channel);
    assert(std::find(formats.begin(), formats.end(), SOAPY_SDR_CS16) != formats.end());
    assert(std::find(formats.begin(), formats.end(), SOAPY_SDR_CF32) != formats.end());

    /// Complex floats stream

    SoapySDR::Stream *rx_stream = sdr->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);  // todo figure out how to dial with channels
    assert(rx_stream != NULL);
    sdr->activateStream(rx_stream, 0, 0, 0);

    std::complex<float> buff[1024];
    for (int i = 0; i < 1000; ++i) {
        void *buffs[] = {buff};
        int flags;
        long long time_ns;
        int ret = sdr->readStream(rx_stream, buffs, 1024, flags, time_ns, 1e5);
        assert(ret > 0);
        // printf("ret = %d, flags = %d, time_ns = %lld\n", ret, flags, time_ns);
    }

    sdr->deactivateStream(rx_stream, 0, 0);  // stop streaming
    sdr->closeStream(rx_stream);

    // 8. cleanup device handle
    SoapySDR::Device::unmake(sdr);
    printf("Done\n");
}
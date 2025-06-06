#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include "ad9361.hpp"
#define BLOCK_SIZE (125 * 1024)  // TODO: make parameter

using namespace std;
class IIODevice : public SoapySDR::Device {
   public:
    std::string getDriverKey(void) const;
    std::string getHardwareKey(void) const;
    size_t getNumChannels(const int direction) const;
    SoapySDR::Kwargs getChannelInfo(const int direction, const size_t channel) const;
    bool getFullDuplex(const int direction, const size_t channel) const;
    SoapySDR::Stream* setupStream(const int direction, const std::string& format, const std::vector<size_t>& channels, const SoapySDR::Kwargs& args);
    void closeStream(SoapySDR::Stream* stream);
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double& fullScale) const;
    int readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags, long long& timeNs, const long timeoutUs = 100000);
    int writeStream(SoapySDR::Stream* stream, const void* const* buffs, const size_t numElems, int& flags, const long long timeNs = 0, const long timeoutUs = 100000);
    size_t getStreamMTU(SoapySDR::Stream* stream) const;
    double getSampleRate(const int direction, const size_t channel) const;
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const;
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string& name) const;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel) const;
    int deactivateStream(SoapySDR::Stream* stream, const int, const long long);
    int activateStream(SoapySDR::Stream* stream, const int, const long long, const size_t);
    void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs& args = SoapySDR::Kwargs());
    void setFrequency(const int direction, const size_t channel, const std::string& name, const double frequency, const SoapySDR::Kwargs& args = SoapySDR::Kwargs());
    void setSampleRate(const int direction, const size_t channel, const double rate);
    double getFrequency(const int direction, const size_t channel) const;
    double getFrequency(const int direction, const size_t channel, const std::string& name) const;
    std::vector<std::string> listGains(const int direction, const size_t channel) const;
    void setGainMode(const int direction, const size_t channel, const bool automatic);
    bool hasGainMode(const int direction, const size_t channel) const;
    bool getGainMode(const int direction, const size_t channel) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string& name) const;
    void setGain(const int direction, const size_t channel, const double value);
    void setGain(const int direction, const size_t channel, const std::string& name, const double value);
    double getGain(const int direction, const size_t channel) const;
    double getGain(const int direction, const size_t channel, const std::string& name) const;

    double getBandwidth(const int direction, const size_t channel) const;
    std::vector<double> listBandwidths(const int direction, const size_t channel) const;
    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const;
    void setBandwidth(const int direction, const size_t channel, const double bw);

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;
    void setAntenna(const int direction, const size_t channel, const std::string& name);
    std::string getAntenna(const int direction, const size_t channel) const;

    IIODevice();
    ~IIODevice();

   private:
    AD9361* device;
};
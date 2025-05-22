#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include "ad9361.hpp"

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

    IIODevice();
    ~IIODevice();

   private:
    AD9361* device;
};
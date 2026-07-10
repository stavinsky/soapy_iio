
// extern "C" {
// #include <iio/iio-debug.h>
// }
// #include <iio/iio.h>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <cctype>

#include <iio.h>

#include "IIODevice.hpp"

#define IIO_ENSURE(expr)                                                             \
    {                                                                                \
        if (!(expr)) {                                                               \
            (void)fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
            (void)abort();                                                           \
        }                                                                            \
    }

namespace {

const char* DEFAULT_SCAN_BACKENDS = "ip,usb";
const int SCAN_ATTEMPTS = 3;
const int SCAN_RETRY_DELAY_MS = 250;

std::string getArg(const SoapySDR::Kwargs& args, const std::string& key) {
    const auto it = args.find(key);
    return (it == args.end()) ? std::string() : it->second;
}

std::string getEnv(const char* key) {
    const char* value = std::getenv(key);
    return (value && *value) ? std::string(value) : std::string();
}

SoapySDR::Kwargs makeResult(const std::string& uri, const std::string& label, const std::string& description = std::string()) {
    SoapySDR::Kwargs options;
    const std::string displayName = label.empty() ? ("my_device " + uri) : label;
    options["device"] = displayName;
    options["driver"] = "my_device";
    options["hardware"] = "fmcomms3";
    options["uri"] = uri;
    options["label"] = displayName;
    options["name"] = displayName;
    options["serial"] = uri;
    options["backend"] = (uri.find("usb:") == 0) ? "usb" : "ip";
    if (!description.empty()) {
        options["description"] = description;
    }
    return options;
}

int uriPreference(const std::string& uri) {
    if (uri.find("ip:") == 0) {
        const std::string address = uri.substr(3);
        return (address.find(':') == std::string::npos) ? 0 : 2;
    }
    if (uri.find("usb:") == 0) {
        return 1;
    }
    return 3;
}

void sortAndDedupeResults(SoapySDR::KwargsList& results) {
    std::stable_sort(results.begin(), results.end(), [](const SoapySDR::Kwargs& lhs, const SoapySDR::Kwargs& rhs) {
        return uriPreference(lhs.at("uri")) < uriPreference(rhs.at("uri"));
    });

    std::set<std::string> seen;
    SoapySDR::KwargsList deduped;
    for (const auto& result : results) {
        if (seen.insert(result.at("uri")).second) {
            deduped.push_back(result);
        }
    }
    results.swap(deduped);
}

std::string connectUriFromScanResult(const char* uri) {
    if (!uri) {
        return std::string();
    }

    return std::string(uri);
}

bool descriptionLooksSupported(const char* description) {
    if (!description) {
        return true;
    }

    std::string desc(description);
    std::transform(desc.begin(), desc.end(), desc.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    return desc.find("ad9361") != std::string::npos ||
           desc.find("ad9363") != std::string::npos ||
           desc.find("ad9364") != std::string::npos ||
           desc.find("pluto") != std::string::npos ||
           desc.find("cf-ad9361-lpc") != std::string::npos ||
           desc.find("cf-ad9361-dds-core-lpc") != std::string::npos;
}

std::string discoverFirstUri(const char* backends) {
#ifdef HAVE_IIO_SCAN
    struct iio_scan_context* scan = iio_create_scan_context(backends, 0);
    if (!scan) {
        return std::string();
    }

    struct iio_context_info** info = NULL;
    const ssize_t count = iio_scan_context_get_info_list(scan, &info);
    if (count < 0) {
        iio_scan_context_destroy(scan);
        return std::string();
    }

    for (ssize_t i = 0; i < count; ++i) {
        const char* uri = iio_context_info_get_uri(info[i]);
        const char* description = iio_context_info_get_description(info[i]);
        const std::string connectUri = connectUriFromScanResult(uri);
        if (!connectUri.empty() && descriptionLooksSupported(description)) {
            std::string result(connectUri);
            iio_context_info_list_free(info);
            iio_scan_context_destroy(scan);
            return result;
        }
    }
    iio_context_info_list_free(info);
    iio_scan_context_destroy(scan);
#else
    (void)backends;
#endif
    return std::string();
}

}  // namespace

SoapySDR::KwargsList findMyDevice(const SoapySDR::Kwargs& args) {
    std::vector<SoapySDR::Kwargs> results;

    const std::string requestedUri = getArg(args, "uri");
    if (!requestedUri.empty()) {
        results.push_back(makeResult(requestedUri, "my_device " + requestedUri));
    } else {
        const std::string requestedSerial = getArg(args, "serial");
        if (!requestedSerial.empty()) {
            results.push_back(makeResult(requestedSerial, "my_device " + requestedSerial));
        } else {
            const std::string configuredUri = getEnv("MYDEVICE_IIO_URI");
            if (!configuredUri.empty()) {
                results.push_back(makeResult(configuredUri, "my_device " + configuredUri, "configured URI"));
            } else {

#ifdef HAVE_IIO_SCAN
                std::string backends = getArg(args, "backends");
                if (backends.empty()) {
                    backends = getEnv("MYDEVICE_IIO_SCAN_BACKENDS");
                }
                if (backends.empty()) {
                    backends = DEFAULT_SCAN_BACKENDS;
                }

                for (int attempt = 1; attempt <= SCAN_ATTEMPTS && results.empty(); ++attempt) {
                    struct iio_scan_context* scan = iio_create_scan_context(backends.c_str(), 0);
                    if (!scan) {
                        if (attempt != SCAN_ATTEMPTS) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(SCAN_RETRY_DELAY_MS));
                        }
                        continue;
                    }

                    struct iio_context_info** info = NULL;
                    const ssize_t count = iio_scan_context_get_info_list(scan, &info);
                    if (count >= 0) {
                        for (ssize_t i = 0; i < count; ++i) {
                            const char* uri = iio_context_info_get_uri(info[i]);
                            const char* description = iio_context_info_get_description(info[i]);
                            const std::string connectUri = connectUriFromScanResult(uri);
                            const bool supported = descriptionLooksSupported(description);
                            SoapySDR_logf(SOAPY_SDR_DEBUG,
                                          "MyDevice: IIO scan candidate uri='%s' description='%s' supported=%s",
                                          uri ? uri : "",
                                          description ? description : "",
                                          supported ? "true" : "false");
                            if (connectUri.empty() || !supported) {
                                continue;
                            }

                            SoapySDR::Kwargs result = makeResult(connectUri, "my_device " + connectUri, description ? description : "");
                            results.push_back(result);
                        }
                        iio_context_info_list_free(info);
                    } else {
                        SoapySDR_logf(SOAPY_SDR_WARNING,
                                      "MyDevice: IIO scan failed for backends '%s' on attempt %d/%d: %zd",
                                      backends.c_str(),
                                      attempt,
                                      SCAN_ATTEMPTS,
                                      count);
                    }
                    iio_scan_context_destroy(scan);
                    if (results.empty() && attempt != SCAN_ATTEMPTS) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(SCAN_RETRY_DELAY_MS));
                    }
                }
                sortAndDedupeResults(results);
#endif
            }
        }
    }

    SoapySDR_logf(SOAPY_SDR_INFO, "MyDevice: found %zu device(s)", results.size());
    return results;
}
SoapySDR::Device* makeMyDevice(const SoapySDR::Kwargs& args) {
    std::string uri = getArg(args, "uri");
    if (uri.empty()) {
        uri = getArg(args, "serial");
    }
    if (uri.empty()) {
        uri = getEnv("MYDEVICE_IIO_URI");
    }
    if (uri.empty()) {
        uri = discoverFirstUri(DEFAULT_SCAN_BACKENDS);
    }
    if (uri.empty()) {
        uri = "ip:";
    }

    return new IIODevice(uri);
}

static SoapySDR::Registry registerMyDevice("my_device", &findMyDevice, &makeMyDevice, SOAPY_SDR_ABI_VERSION);

# SoapySDR-MyDevice

This is an experimental [SoapySDR](https://github.com/pothosware/SoapySDR) module that interfaces with devices over IIO using a hardcoded configuration.  
Currently designed as a **proof-of-concept** (PoC), with limited functionality and no guarantees.

## ⚠️ Status

This project is **not production-ready**. It works for the author's setup but may require adjustments for others.  
Feel free to experiment or adapt — use at your own risk.

## Tested with:

- SDR++
- CubicSDR
- SDRAngel
- GNU Radio

## ✅ Features

- Connects to remote IIO device using hardcoded IP address
- Supports **only both RX channel** (channel 0, 1)
- Block size, IP, available frequencies, and gain settings are all hardcoded
- Uses [libiio](https://github.com/analogdevicesinc/libiio) v1 (still in beta)
- Works on **macOS** with Homebrew-installed dependencies
- Should work with any IIO-compatible URL (not limited to IP transport)

## ⚙️ Requirements

- **libiio v1** (latest master branch, as it's still in beta). It is not compatible with current stable versions 0.25-0.26
- A compatible IIO device (e.g. AD9361)
- [SoapySDR](https://github.com/pothosware/SoapySDR)

## 💠 Recommended Build

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/homebrew ../
make
make install
```

Make sure SoapySDR and libiio are installed in compatible paths.

## 🌍 Environment & Disclaimer

Ensure `PKG_CONFIG_PATH` is set so `pkg-config` can locate `libiio.pc`:

```bash
export PKG_CONFIG_PATH=/path/to/libiio/pkgconfig
```

Adjust the path based on your libiio installation.

This project is shared as-is. There is:

- No automatic configuration
- No runtime detection of capabilities
- No TX support (yet)

notes:

```sh
export DYLD_LIBRARY_PATH=/Users/stavinsky/dev/build/root/lib
export PATH=/Users/stavinsky/dev/build/root/bin/:$PATH
export PKG_CONFIG_PATH=/Users/stavinsky/dev/build/root/lib/pkgconfig/
install_name_tool -change @rpath/libiio.1.dylib /Users/stavinsky/dev/build/root/lib/libiio.dylib /opt/homebrew/lib/SoapySDR/modules0.8/libMyDevice.so
```

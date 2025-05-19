```sh
export DYLD_LIBRARY_PATH=/Users/stavinsky/dev/build/root/lib
export PATH=/Users/stavinsky/dev/build/root/bin/:$PATH
export PKG_CONFIG_PATH=/Users/stavinsky/dev/build/root/lib/pkgconfig/
install_name_tool -change @rpath/libiio.1.dylib /Users/stavinsky/dev/build/root/lib/libiio.dylib /opt/homebrew/lib/SoapySDR/modules0.8/libMyDevice.so
```

i

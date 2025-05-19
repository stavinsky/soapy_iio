cmake_minimum_required(VERSION 2.6...3.10)
project(SoapySDRMyDevice CXX)
enable_testing()

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")


find_package(SoapySDR CONFIG)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBIIO REQUIRED libiio)





# set(MY_INCLUDE_DIRS ${MY_DEVICE_INCLUDE_DIRS})

    foreach(dir ${LIBIIO_INCLUDE_DIRS})
        list(APPEND MY_INCLUDE_DIRS "${dir}/iio")
    endforeach()

link_directories(${LIBIIO_LIBRARY_DIRS})
include_directories(${LIBIIO_INCLUDE_DIRS} )
include_directories(${MY_INCLUDE_DIRS} )

SOAPY_SDR_MODULE_UTIL(
    TARGET MyDevice
    SOURCES MyDeviceSupport.cpp ad9361.cpp
    LIBRARIES ${LIBIIO_LIBRARIES}
)
if(APPLE)
    install(CODE "
        execute_process(COMMAND install_name_tool
            -change @rpath/libiio.1.dylib
                    ${LIBIIO_LIBRARY_DIRS}/libiio.dylib
                    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/SoapySDR/modules0.8/libMyDevice.so
        )
    ")

endif()
message(STATUS "MY_INCLUDE_DIRS: ${MY_INCLUDE_DIRS}")
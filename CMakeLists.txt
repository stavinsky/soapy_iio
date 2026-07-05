cmake_minimum_required(VERSION 2.6...3.10)
project(SoapySDRMyDevice CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")


find_package(SoapySDR REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBIIO REQUIRED "libiio < 1.0")
include(CheckCXXSourceCompiles)

set(LIBIIO_026_FRAMEWORK_DIR "/Library/Frameworks/iio.framework/Versions/0.26")
if(EXISTS "${LIBIIO_026_FRAMEWORK_DIR}/iio" AND EXISTS "${LIBIIO_026_FRAMEWORK_DIR}/Headers/iio.h")
    set(LIBIIO_INCLUDE_DIRS "${LIBIIO_026_FRAMEWORK_DIR}/Headers")
    set(LIBIIO_LIBRARY_DIRS "${LIBIIO_026_FRAMEWORK_DIR}")
    set(LIBIIO_LIBRARIES "${LIBIIO_026_FRAMEWORK_DIR}/iio")
    set(LIBIIO_LDFLAGS "${LIBIIO_026_FRAMEWORK_DIR}/iio")
endif()

set(MY_INCLUDE_DIRS)
foreach(dir ${LIBIIO_INCLUDE_DIRS})
    if(EXISTS "${dir}/iio.h")
        list(APPEND MY_INCLUDE_DIRS "${dir}")
    endif()
    if(EXISTS "${dir}/iio/iio.h")
        list(APPEND MY_INCLUDE_DIRS "${dir}/iio")
    endif()
endforeach()
if(NOT MY_INCLUDE_DIRS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../libiio/iio.h")
    list(APPEND MY_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../libiio")
endif()
if(NOT MY_INCLUDE_DIRS)
    message(FATAL_ERROR "Could not find iio.h. Install libiio 0.26 headers or place the libiio source tree next to this project.")
endif()

link_directories(${LIBIIO_LIBRARY_DIRS})
include_directories(SYSTEM ${MY_INCLUDE_DIRS} )

set(CMAKE_REQUIRED_INCLUDES ${MY_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${LIBIIO_LDFLAGS})
set(MY_SAVED_TRY_COMPILE_TARGET_TYPE ${CMAKE_TRY_COMPILE_TARGET_TYPE})
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
unset(HAVE_IIO_SCAN CACHE)
check_cxx_source_compiles("
    #include <iio.h>
    int main() {
        struct iio_scan_context *scan = iio_create_scan_context(\"ip,usb\", 0);
        if (scan) {
            struct iio_context_info **info = 0;
            ssize_t count = iio_scan_context_get_info_list(scan, &info);
            if (count >= 0) {
                iio_context_info_list_free(info);
            }
            iio_scan_context_destroy(scan);
        }
        return 0;
    }
" HAVE_IIO_SCAN)
set(CMAKE_TRY_COMPILE_TARGET_TYPE ${MY_SAVED_TRY_COMPILE_TARGET_TYPE})
if(HAVE_IIO_SCAN)
    message(STATUS "libiio scan support found; enabling ip/usb autodiscovery")
else()
    message(WARNING "libiio scan declarations were not found. Autodiscovery is disabled; pass uri=ip:<host> or uri=usb:, or point PKG_CONFIG_PATH at a complete libiio 0.26 install.")
endif()

set(MY_SOURCES
    MyDeviceSupport.cpp
    IIODevice.cpp
    Stream.cpp
    ad9361.cpp
)


SOAPY_SDR_MODULE_UTIL(
    TARGET MyDevice
    SOURCES ${MY_SOURCES}
    LIBRARIES ${LIBIIO_LIBRARIES}
)
if(APPLE AND EXISTS "${LIBIIO_026_FRAMEWORK_DIR}/iio")
    set_target_properties(MyDevice PROPERTIES
        BUILD_RPATH "/Library/Frameworks"
        INSTALL_RPATH "/Library/Frameworks"
    )
endif()

if(HAVE_IIO_SCAN)
    target_compile_definitions(MyDevice PRIVATE HAVE_IIO_SCAN=1)
endif()




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
# if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
#     target_compile_options(MyDevice PRIVATE
#         -Wall
#         -Wextra
#         -Wconversion
#         -Wsign-conversion
#         -Wpedantic
#     )
# endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(MyDevice PRIVATE
        -Wall -Wextra -Wconversion -Wsign-conversion -Wpedantic
        -Wcast-align -Wcast-qual -Wold-style-cast -Woverloaded-virtual
        -Wnon-virtual-dtor -Wnull-dereference -Wdouble-promotion -Wshadow
        -Wmissing-declarations -Wundef -Wunreachable-code -Wredundant-decls
    )

    # if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    #     target_compile_options(MyDevice PRIVATE -Werror -fsanitize=undefined,address)
    #     target_link_options(MyDevice PRIVATE -fsanitize=undefined,address)
    # endif()

    foreach(src ${MY_SOURCES})
        set_source_files_properties(${src} PROPERTIES
            COMPILE_OPTIONS
                -Wall
                -Wextra
                -Wconversion
                -Wsign-conversion
                -Wpedantic
                -Wcast-align
                -Wcast-qual
                -Wold-style-cast
                -Woverloaded-virtual
                -Wnon-virtual-dtor
                -Wnull-dereference
                -Wdouble-promotion
                -Wshadow
                -Wmissing-declarations
                -Wundef
                -Wunreachable-code
                -Wredundant-decls
        )
    endforeach()
    # foreach(src ${MY_SOURCES})
    #     set_source_files_properties(${src} PROPERTIES
    #         COMPILE_OPTIONS "-Wall;-Wextra;-Wconversion;-Wsign-conversion;-Wpedantic;-Wshadow"
    #     )
    # endforeach()
endif()
enable_testing()
add_subdirectory(tests)


add_executable(fir read_filter.cpp ad9361.cpp)
target_include_directories(fir PRIVATE ${MY_INCLUDE_DIRS})
target_link_libraries(fir ${LIBIIO_LIBRARIES} SoapySDR)

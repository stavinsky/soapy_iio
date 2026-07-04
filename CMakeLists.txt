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
pkg_check_modules(LIBIIO REQUIRED libiio)
include(CheckCXXSourceCompiles)


foreach(dir ${LIBIIO_INCLUDE_DIRS})
    list(APPEND MY_INCLUDE_DIRS "${dir}/iio")
endforeach()

link_directories(${LIBIIO_LIBRARY_DIRS})
include_directories(${LIBIIO_INCLUDE_DIRS} )
include_directories(SYSTEM ${MY_INCLUDE_DIRS} )

set(CMAKE_REQUIRED_INCLUDES ${LIBIIO_INCLUDE_DIRS} ${MY_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${LIBIIO_LDFLAGS})
check_cxx_source_compiles("
    #include <iio.h>
    int main() {
        struct iio_scan *scan = iio_scan(NULL, \"ip,usb\");
        if (iio_err(scan) == 0) {
            iio_scan_destroy(scan);
        }
        return 0;
    }
" HAVE_IIO_SCAN)
if(HAVE_IIO_SCAN)
    message(STATUS "libiio scan support found; enabling ip/usb autodiscovery")
else()
    message(WARNING "libiio scan support was not found. Autodiscovery is disabled; pass uri=ip:<host> or uri=usb:, or point PKG_CONFIG_PATH at a libiio build with iio_scan().")
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
target_include_directories(fir PRIVATE ${LIBIIO_INCLUDE_DIRS})
target_link_libraries(fir ${LIBIIO_LIBRARIES} SoapySDR)

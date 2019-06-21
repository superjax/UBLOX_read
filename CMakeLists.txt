cmake_minimum_required(VERSION 2.8.3)
project(ublox)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_subdirectory(lib/async_comm)

include_directories(include
  lib/async_comm/include
)

add_library(UBLOX
    src/ublox.cpp
    src/ubx.cpp
    src/rtcm.cpp
    src/nmea.cpp
)

find_package(Threads REQUIRED)

add_executable(ublox
  src/main.cpp
)
target_link_libraries(ublox UBLOX async_comm pthread)

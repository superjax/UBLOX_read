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

add_executable(base src/base.cpp)
target_link_libraries(base UBLOX async_comm pthread)

add_executable(rover src/rover.cpp)
target_link_libraries(rover UBLOX async_comm pthread)

add_executable(read src/read.cpp)
target_link_libraries(read UBLOX async_comm pthread)

cmake_minimum_required(VERSION 2.8.3)
project(ublox)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_subdirectory(lib/async_comm)

include_directories(include
  lib/async_comm/include
)

add_library(UBLOX
    include/ublox.h
    src/ublox.cpp
)

add_executable(ublox
  src/main.cpp
)
target_link_libraries(ublox UBLOX async_comm)

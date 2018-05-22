cmake_minimum_required(VERSION 2.8.3)
project(ublox)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_subdirectory(lib/async_comm)

include_directories(include
  lib/async_comm/include
)

add_executable(ublox
  include/ublox.h
  src/ublox.cpp
  src/main.cpp
)
target_link_libraries(ublox async_comm)

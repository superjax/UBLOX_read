# UBLOX Parser

This is a library for parsing UBLOX packets and interfacing with UBLOX gps receivers.  It has been designed for use with the M8N GPS receivers.

This library provides rather basic functionality and is designed to work under a linux environment, however it should probably work in Windows or Mac as it uses the cross-platform [`async_comm`](https://github.com/dpkoch/async_comm) library as the serial interface.

The UBX parsing functionality is abstracted into a library for easy integration in other projects.  Example usage is given in the `main.cpp` file.

## Functionality
 * Change baudrate of GPS
 * Enable messages
 * Parse messages
 * Change navigation and message publishing rate
 * auto baudrate detection
 * changing the dynamic mode
 * accessing lat/lon/altitude measurements, fix type and NED velocity


## Building

``` bash
mkdir build
cd build
cmake ..
make -j
```

## Running the example
``` bash
./ublox
```

# UBLOX Parser

This is a library for parsing UBLOX packets and interfacing with UBLOX gps receivers. It has been designed for use with the M8T/F9P GPS receivers.

This library provides rather basic functionality and is designed to work under a linux environment, however it should probably work in Windows or Mac as it uses the cross-platform [`async_comm`](https://github.com/dpkoch/async_comm) library as the serial interface.

The UBX, RTCM, NMEA and satellite ephemeris parsing functionalities are abstracted into separate libraries for easy integration in other projects. Example usage is given in the `main.cpp` file.

## Functionality

- Change baudrate of GPS
- Enable messages
- Parse messages
- Change navigation and message publishing rate
- auto baudrate detection
- changing the dynamic mode
- accessing lat/lon/altitude measurements, fix type and NED velocity
- Satellite ephemeris for GPS and GLONASS (\*T varieties only)
- Raw measurements from GPS, GLONASS (pseudorange, doppler, carrier phase) (\*T varieties only)
- Passing of RTCM between two receivers for RTK and moving-base RTK (compassing) operations (\*P varieties only)

## Building

```bash
mkdir build
cd build
cmake ..
make -j
```

## Running the examples

```bash
./ublox
```

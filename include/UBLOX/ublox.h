#ifndef UBLOX_H
#define UBLOX_H

#include <stdint.h>
#include <iostream>

#include "async_comm/serial.h"
#include "async_comm/udp.h"

#include "UBLOX/ubx.h"
#include "UBLOX/rtcm.h"
#include "UBLOX/nmea.h"


class UBLOX
{
public:
    enum
    {
        START_BYTE_UBX = 0xB5,
        START_BYTE_UBX2 = 0x62,

        START_BYTE_RTCM = 0xD3,

        START_BYTE_NMEA = '$',
        START_BYTE_NMEA2 = 'G',
    };

    typedef enum
    {
        NONE = 0,
        ROVER = 0b10,
        BASE = 0b11,
        RTK = 0b10,
    } rtk_type_t;

    UBLOX(rtk_type_t type, std::string port);
    ~UBLOX();

    async_comm::UDP* udp_ = nullptr;
    async_comm::Serial serial_;

    UBX ubx_;
    RTCM rtcm_;
    NMEA nmea_;

    inline void registerUBXCallback(uint8_t cls, uint8_t type, UBX::ubx_cb cb)
    {
        ubx_.registerCallback(cls, type, cb);
    }

    rtk_type_t type_;

    void serial_read_cb(const uint8_t* buf, size_t size);
    void udp_read_cb(const uint8_t *buf, size_t size);
    void rtcm_complete_cb(const uint8_t* buf, size_t size);

    void configRover();
    void configBase();

    int j = 1;
    int k = 1;
    int l = 1;
    int m = 1;
    int message;

};

#endif

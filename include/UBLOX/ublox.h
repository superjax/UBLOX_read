#ifndef UBLOX_H
#define UBLOX_H

#include <stdint.h>
#include <iostream>
#include <fstream>

#include "async_comm/serial.h"
#include "async_comm/udp.h"

#include "UBLOX/parsers/ubx.h"
#include "UBLOX/parsers/rtcm.h"
#include "UBLOX/parsers/nav.h"

namespace ublox
{

class UBLOX
{
public:

    typedef enum
    {
        NONE = 0,
        ROVER = 0b10,
        BASE = 0b11,
        RTK = 0b10,
    } rtk_type_t;

    UBLOX(const std::string& port);
    ~UBLOX();

    void initBase(std::string local_host, uint16_t local_port,
                  std::string remote_host, uint16_t remote_port,
                  std::string base_type);

    void initBase(std::string local_host, uint16_t local_port,
                  std::string remote_host, uint16_t remote_port,
                  std::string base_type, int rover_quantity);

    void initBase(std::string local_host, uint16_t local_port,
                  std::string remote_host, uint16_t remote_port,
                  std::string local_host2, uint16_t local_port2,
                  std::string remote_host2, uint16_t remote_port2);

    void initBase(std::string local_host, uint16_t local_port,
                  std::string remote_host, uint16_t remote_port,
                  std::string local_host2, uint16_t local_port2,
                  std::string remote_host2, uint16_t remote_port2,
                  std::string base_type);

    void initBase(std::string local_host[], int local_port[],
                    std::string remote_host[], int remote_port[],
                    std::string base_type, int rover_quantity);

    void initRover(std::string local_host, uint16_t local_port,
                   std::string remote_host, uint16_t remote_port);

    void initLogFile(const std::string& filename);
    void readFile(const std::string& filename);

    async_comm::UDP* udp_ = nullptr;
    async_comm::Serial serial_;

    UBX ubx_;
    RTCM rtcm_;
    NavParser nav_;

    std::ofstream log_file_;

    inline void registerUBXCallback(uint8_t cls, uint8_t type, UBX::ubx_cb cb)
    {
        ubx_.registerCallback(cls, type, cb);
    }
    inline void registerEphCallback(const NavParser::eph_cb& cb)
    {
        nav_.registerCallback(cb);
    }
    inline void registerGephCallback(const NavParser::geph_cb& cb)
    {
        nav_.registerCallback(cb);
    }


    rtk_type_t type_;

    void serial_read_cb(const uint8_t* buf, size_t size);
    void udp_read_cb(const uint8_t *buf, size_t size);
    void rtcm_complete_cb(const uint8_t* buf, size_t size);

    void config_f9p();
    void config_rover();
    void config_base(std::string base_type);
    void config_base_stationary(int on_off);
    void config_base_moving(int on_off);
    void poll_value();

    uint8_t byte = 1;
    uint8_t word = 2;
    int on_off;
};
}

#endif

#pragma once

#include "UBLOX/serial_interface.h"
#include "async_comm/comm.h"
#include "async_comm/serial.h"
#include "async_comm/udp.h"

namespace ac_adapter
{
class UDP : public async_comm::UDP, public SerialInterface, public async_comm::CommListener
{
public:
    UDP(std::string bind_host = "localhost",
        uint16_t bind_port = 16410,
        std::string remote_host = "localhost",
        uint16_t remote_port = 16415)
        : async_comm::UDP(bind_host, bind_port, remote_host, remote_port)
    {
        register_listener(*this);
    }

    void write(const uint8_t* buf, const size_t size) override
    {
        async_comm::Comm::send_bytes(buf, size);
    }

    void receive_callback(const uint8_t* buf, const size_t size) override
    {
        if (cb_)
            cb_(buf, size);
        for (auto& l : SerialInterface::listeners_)
        {
            l->read_cb(buf, size);
        }
    }
};

class Serial : public async_comm::Serial, public SerialInterface, public async_comm::CommListener
{
public:
    Serial(std::string port, unsigned int baud_rate) : async_comm::Serial(port, baud_rate)
    {
        async_comm::Comm::default_message_handler_.debug("TESTING");
        register_listener(*this);
    }

    void write(const uint8_t* buf, const size_t size) override
    {
        async_comm::Comm::send_bytes(buf, size);
    }

    void receive_callback(const uint8_t* buf, const size_t size) override
    {
        if (cb_)
            cb_(buf, size);
        for (auto& l : SerialInterface::listeners_)
        {
            l->read_cb(buf, size);
        }
    }
};
}  // namespace ac_adapter
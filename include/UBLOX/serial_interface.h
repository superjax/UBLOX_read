#pragma once

#include <cstdint>
#include <cstdlib>
#include <functional>

class SerialListener
{
public:
    virtual void read_cb(const uint8_t* buf, const size_t size) = 0;
};

class SerialInterface
{
public:
    typedef std::function<void(const uint8_t* buf, const size_t size)> serial_cb;

    virtual void write(const uint8_t* buf, const size_t size) = 0;
    void add_listener(SerialListener* l) { listeners_.push_back(l); }
    void add_callback(serial_cb cb) { cb_ = cb; }

    std::vector<SerialListener*> listeners_;
    serial_cb cb_;
};
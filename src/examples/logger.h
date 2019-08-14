#pragma once

#include <chrono>
#include <fstream>
#include <thread>

#include "UBLOX/serial_interface.h"

class Logger : public SerialListener, public SerialInterface
{
   public:
    enum class Type
    {
        READ,
        WRITE
    };
    Logger(std::string filename, Type type) : type_(type)
    {
        if (type == Type::READ)
        {
            read_file_.open(filename, std::ifstream::binary);
            if (!read_file_.is_open())
            {
                throw std::runtime_error("Unable to open file");
            }
        }
        else
        {
            log_file_.open(filename);
            if (!log_file_.is_open())
            {
                throw std::runtime_error("Unable to open file");
            }
        }
    }

    void play()
    {
        if (type_ != Type::READ)
        {
            throw std::runtime_error("Must be in READ mode to play log");
        }
        read_file_.seekg(0, read_file_.end);
        uint32_t len = read_file_.tellg();
        read_file_.seekg(0, read_file_.beg);
        char* buffer = new char[len];
        read_file_.read(buffer, len);
        int idx = 0;
        while (idx < len)
        {
            int chunk_size = ((len - idx) >= 14) ? 14 : len - idx;
            for (auto& l : listeners_)
            {
                l->read_cb(((const uint8_t*)buffer) + idx, chunk_size);
            }
            idx = idx + chunk_size;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void read_cb(const uint8_t* buf, const size_t size)
    {
        if (type_ != Type::WRITE)
        {
            throw std::runtime_error("Must be in write mode to record data");
        }
        else if (!log_file_.is_open())
        {
            throw std::runtime_error("Unable to write to file");
        }

        log_file_.write((const char*)buf, size);
    }

    // TODO: We don't record any data that the driver writes out
    void write(const uint8_t* buf, const size_t size)
    {
        static bool print_once = false;
        if (!print_once)
        {
            printf("Logging outgoing data is currently not supported\n");
            print_once = true;
        }
    }

   protected:
    Type type_;
    std::ofstream log_file_;
    std::ifstream read_file_;
};
#ifndef NMEA_H
#define NMEA_H

#include <stdint.h>
#include <stddef.h>

class NMEA
{
public:
    static constexpr int BUFFER_SIZE = 82;
    enum {
        START_BYTE1 = '$',
        START_BYTE2 = 'G',
        END_BYTE1 = '\r',
        END_BYTE2 = '\n',
    };


    NMEA();
    bool read_cb(uint8_t byte);
    bool parsing_message();
    inline volatile bool got_data() const { return got_data_; }
    volatile bool got_data_;

    size_t length_;
    bool start_message_;
    bool end_message_;
    uint8_t prev_byte_;
};

#endif

#ifndef NMEA_H
#define NMEA_H

#include <stdint.h>

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
    inline volatile bool got_data() const { return got_data_; }
    volatile bool got_data_;

    size_t length_;
    bool got_message_;
    uint8_t prev_byte_;
};

#endif

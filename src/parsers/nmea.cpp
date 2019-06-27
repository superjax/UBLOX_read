#include "UBLOX/parsers/nmea.h"


NMEA::NMEA()
{
    prev_byte_ = 0;
    length_ = 0;
    end_message_ = false;
    start_message_ = false;
    new_data_ = false;
}

bool NMEA::new_data()
{
    bool tmp = new_data_;
    new_data_ = false;
    return tmp;
}

bool NMEA::parsing_message()
{
    return (start_message_ == true && end_message_ == false);
}

bool NMEA::read_cb(uint8_t byte)
{
    buffer_[length_++] = byte;

    // found start of NMEA packet
    if ((byte == START_BYTE2 && prev_byte_ == START_BYTE1))
    {
        start_message_ = true;
        end_message_ = false;
        length_ = 0;
    }

    // found end of NMEA packet
    if (byte == END_BYTE2 && prev_byte_ == END_BYTE1)
    {
        start_message_ = false;
        end_message_ = true;
        length_ = 0;

        prev_byte_ = byte;
        //printf("%s", buffer_); //to see nmea messages
        return true;
    }

    // Bad message
    if (length_ >= BUFFER_SIZE)
    {
        start_message_ = false;
        end_message_ = false;
        length_ = 0;
    }

    prev_byte_ = byte;
    return false;
}

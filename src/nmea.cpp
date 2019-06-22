#include "UBLOX/nmea.h"


NMEA::NMEA()
{

}

NMEA::NMEA()
{
    length_++;
    got_message_ = true;

    if ((byte == END_BYTE2 && prev_byte_ == END_BYTE1)
            || (length >= BUFFER_SIZE))
    {
        looking_for_ubx_ = true;
        //looking_for_rtcm_ = true;
        NMEA = false;
        looking_for_nmea_ = false;
    }
}

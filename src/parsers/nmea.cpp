/* Copyright (c) 2019 James Jackson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "UBLOX/parsers/nmea.h"

#define DBG(...) fprintf(stderr, __VA_ARGS__)

NMEA::NMEA()
{
    prev_byte_ = 0;
    restart();
}

void NMEA::restart()
{
    start_message_ = false;
    end_message_ = false;
    length_ = 0;
}

bool NMEA::new_data()
{
    bool tmp = new_data_;
    new_data_ = false;
    return tmp;
}

bool NMEA::parsing_message() { return (start_message_ == true && end_message_ == false); }

bool NMEA::read_cb(uint8_t byte)
{
    buffer_[length_++] = byte;

    // found start of NMEA packet
    if ((byte == START_BYTE2 && prev_byte_ == START_BYTE1))
    {
        start_message_ = true;
        end_message_ = false;
        buffer_[0] = START_BYTE1;
        buffer_[1] = START_BYTE2;
        length_ = 2;
    }

    // found end of NMEA packet
    if (byte == END_BYTE2 && prev_byte_ == END_BYTE1)
    {
        start_message_ = false;
        end_message_ = true;
        length_ = 0;

        prev_byte_ = byte;
        printf("\nNMEA_MESSAGE: %s\n", buffer_);  // to see nmea messages
        for (int i = 0; i < BUFFER_SIZE; i++) buffer_[i] = 0;
        return true;
    }

    // Bad message
    if (length_ >= BUFFER_SIZE)
    {
        DBG("NMEA Bad Message\n");
        start_message_ = false;
        end_message_ = false;
        length_ = 0;
    }

    prev_byte_ = byte;
    return false;
}

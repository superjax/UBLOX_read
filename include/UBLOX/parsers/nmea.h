/* Copyright (c) 2019 James Jackson, Matt Rydalch
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

#ifndef NMEA_H
#define NMEA_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

class NMEA
{
   public:
    static constexpr int BUFFER_SIZE = 82;
    enum
    {
        START_BYTE1 = '$',
        START_BYTE2 = 'G',
        END_BYTE1 = '\r',
        END_BYTE2 = '\n',
    };

    NMEA();
    void restart();
    bool read_cb(uint8_t byte);
    bool parsing_message();
    bool new_data();
    bool new_data_;

    size_t length_;
    bool start_message_;
    bool end_message_;
    uint8_t prev_byte_;
    uint8_t buffer_[BUFFER_SIZE];
};

#endif

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

#include "UBLOX/parsers/rtcm.h"
#include "UBLOX/bit_tools.h"
#include "UBLOX/crc.h"

RTCM::RTCM()
{
    buffer_head_ = 0;
    parse_state_ = ParseState::START;
    prev_byte_ = 0;

    start_message_ = false;
    new_data_ = false;
    end_message_ = false;
}

void RTCM::restart()
{
    parse_state_ = ParseState::START;
    start_message_ = false;
    end_message_ = false;
}

bool RTCM::parsing_message() const
{
    return (start_message_ == true && end_message_ == false);
}

bool RTCM::new_data()
{
    bool tmp = new_data_;
    new_data_ = false;
    return tmp;
}

bool RTCM::check_crc()
{
    uint32_t read_crc = getBit<24>(in_buffer_, (buffer_head_ - 3) * 8);
    uint32_t computed_crc = crc24(in_buffer_, payload_len_ + 3);
    return read_crc == computed_crc;
}

void RTCM::read_cb(const uint8_t* buf, size_t size)
{
    for (int i = 0; i < size; ++i)
    {
        uint8_t byte = buf[i];
        switch (parse_state_)
        {
        case ParseState::START:
            buffer_head_ = 0;
            if (byte == START_BYTE)
            {
                parse_state_ = ParseState::GOT_START_FRAME;
                payload_len_ = 0;
                ck_a_ = 0;
                ck_b_ = 0;
                ck_c_ = 0;
                start_message_ = true;
                end_message_ = false;
                in_buffer_[buffer_head_++] = byte;
            }
            break;
        case ParseState::GOT_START_FRAME:
            in_buffer_[buffer_head_++] = byte;
            parse_state_ = ParseState::GOT_LENGTH1;
            break;
        case ParseState::GOT_LENGTH1:
            in_buffer_[buffer_head_++] = byte;
            payload_len_ = ((prev_byte_ & 0x3) << 8) | byte;
            parse_state_ = ParseState::GOT_LENGTH2;
            if (payload_len_ > BUFFER_SIZE || payload_len_ == 0)
            {
                num_errors_++;
                prev_byte_ = byte;
                restart();
            }
            break;
        case ParseState::GOT_LENGTH2:
            in_buffer_[buffer_head_++] = byte;
            if (buffer_head_ - 3 == payload_len_)  // remember that buffer_head includes the header
            {
                parse_state_ = ParseState::GOT_PAYLOAD;
            }
            if (buffer_head_ - 3 > payload_len_)  // remember that buffer_head includes the header
            {
                printf("buffer head > payload length");
                fflush(stdout);
                num_errors_++;
                parse_state_ = ParseState::START;
                prev_byte_ = byte;
                start_message_ = false;
                end_message_ = false;
            }
            break;
        case ParseState::GOT_PAYLOAD:
            in_buffer_[buffer_head_++] = byte;
            ck_a_ = byte;
            parse_state_ = ParseState::GOT_CK_A;
            break;
        case ParseState::GOT_CK_A:
            in_buffer_[buffer_head_++] = byte;
            ck_b_ = byte;
            parse_state_ = ParseState::GOT_CK_B;
            break;
        case ParseState::GOT_CK_B:
            in_buffer_[buffer_head_++] = byte;
            ck_c_ = byte;
            parse_state_ = ParseState::GOT_CK_C;
            if (buffer_head_ > BUFFER_SIZE)
            {
                buffer_head_ = 0;
                num_errors_++;
                parse_state_ = ParseState::START;
                start_message_ = false;
                end_message_ = true;
            }
            break;
        default:
            buffer_head_ = 0;
            num_errors_++;
            parse_state_ = ParseState::START;
            start_message_ = false;
            end_message_ = true;
            break;
        }

        // If we have a complete packet, then try to parse it
        if (parse_state_ == ParseState::GOT_CK_C)
        {
            parse_state_ = ParseState::START;
            start_message_ = false;
            end_message_ = true;
            decode();
            prev_byte_ = byte;
        }

        prev_byte_ = byte;
    }
}

void RTCM::decode()
{
    new_data_ = true;
    message_len_ = buffer_head_;
    if (check_crc())
    {
        for (auto& l : listeners_)
        {
            l->got_rtcm(in_buffer_, buffer_head_);
        }
    }
    else
    {
        printf("RTCM not pass CRC check\n");
    }
}

void RTCM::registerListener(RTCMListener* l)
{
    listeners_.push_back(l);
}

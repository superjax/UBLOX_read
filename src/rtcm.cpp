#include "UBLOX/rtcm.h"


RTCM::RTCM()
{
    new_data_ = false;
    buffer_head_ = 0;
    parse_state_ = START;
    start_message_ = false;
    end_message_ = false;
}

bool RTCM::parsing_message()
{
    return (start_message_ == true && end_message_ == false);
}

bool RTCM::new_data()
{
     return new_data_;
}

bool RTCM::read_cb(uint8_t byte)
{
    switch (parse_state_)
    {
    case START:
        if (byte == START_BYTE)
        {
            buffer_head_ = 0;
            parse_state_ = GOT_START_FRAME;
            payload_len_ = 0;
            ck_a_ = 0;
            ck_b_ = 0;
            ck_c_ = 0;
            start_message_ = true;
            end_message_ = false;
            in_buffer_[buffer_head_++] = byte;
        }
        break;
    case GOT_START_FRAME:
        in_buffer_[buffer_head_++] = byte;
        parse_state_ = GOT_LENGTH1;
        break;
    case GOT_LENGTH1:
        in_buffer_[buffer_head_++] = byte;
        payload_len_ = ((prev_byte_ & 0x3)<<8) | byte;
        parse_state_ = GOT_LENGTH2;
        if (payload_len_ > BUFFER_SIZE)
        {
            num_errors_++;
            parse_state_ = START;
            prev_byte_ = byte;
            return false;
        }
        break;
    case GOT_LENGTH2:
        in_buffer_[buffer_head_++] = byte;
        if (buffer_head_ == payload_len_-1)
        {
            parse_state_ = GOT_PAYLOAD;
        }
        break;
    case GOT_PAYLOAD:
        in_buffer_[buffer_head_++] = byte;
        ck_a_ = byte;
        parse_state_ = GOT_CK_A;
        break;
    case GOT_CK_A:
        in_buffer_[buffer_head_++] = byte;
        ck_b_ = byte;
        parse_state_ = GOT_CK_B;
        break;
    case GOT_CK_B:
        in_buffer_[buffer_head_++] = byte;
        ck_c_ = byte;
        parse_state_ = GOT_CK_C;
        break;
    default:
        num_errors_++;
        parse_state_ = START;
        break;
    }

    // If we have a complete packet, then try to parse it
    if (parse_state_ == GOT_CK_C)
    {
        parse_state_ = START;
        start_message_ = false;
        end_message_ = true;
        decode();
        prev_byte_ = byte;
        return true;
    }

    prev_byte_ = byte;
    return false;
}

void RTCM::decode()
{
    // just copy the data into the out buffer for future retreival
    // and set the new_data flag
    msg_lock_.lock();
    for (int i = 0; i < buffer_head_; i++)
    {
        out_buffer_[i] = in_buffer_[i];
    }
    new_data_ = true;
    message_len_ = buffer_head_;
    msg_lock_.unlock();
}

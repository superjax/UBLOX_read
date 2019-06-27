#include "UBLOX/parsers/rtcm.h"

#define DBG(...) fprintf(stderr, __VA_ARGS__)

RTCM::RTCM()
{
    buffer_head_ = 0;
    parse_state_ = START;
    prev_byte_ = 0;

    start_message_ = false;
    new_data_ = false;
    end_message_ = false;
}

bool RTCM::parsing_message()
{
    return (start_message_ == true && end_message_ == false);
}

bool RTCM::new_data()
{
    bool tmp = new_data_;
    new_data_ = false;
    return tmp;
}

bool RTCM::read_cb(uint8_t byte)
{
    switch (parse_state_)
    {
    case START:
        buffer_head_ = 0;
        if (byte == START_BYTE)
        {
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
        if (payload_len_ > BUFFER_SIZE || payload_len_ == 0)
        {
            num_errors_++;
            parse_state_ = START;
            prev_byte_ = byte;
            start_message_ = false;
            end_message_ = false;
            return false;
        }
        break;
    case GOT_LENGTH2:
        in_buffer_[buffer_head_++] = byte;
        if (buffer_head_ -3 == payload_len_) // remember that buffer_head includes the header
        {
            parse_state_ = GOT_PAYLOAD;
        }
        if (buffer_head_ -3 > payload_len_) // remember that buffer_head includes the header
        {
            printf("buffer head > payload length");
            fflush(stdout);
            num_errors_++;
            parse_state_ = START;
            prev_byte_ = byte;
            start_message_ = false;
            end_message_ = false;
            return false;
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
        if (buffer_head_ > BUFFER_SIZE)
        {
            buffer_head_ = 0;
            num_errors_++;
            parse_state_ = START;
            start_message_ = false;
            end_message_ = true;
        }
        break;
    default:
        buffer_head_ = 0;
        num_errors_++;
        parse_state_ = START;
        start_message_ = false;
        end_message_ = true;
        break;
    }

    if (canary_ != 0xCAFEBABE)
    {
        int debug = 1;
    }

    // If we have a complete packet, then try to parse it
    if (parse_state_ == GOT_CK_C)
    {
        parse_state_ = START;
        start_message_ = false;
        end_message_ = true;
        decode();
        prev_byte_ = byte;
        //DBG("RTCM length: %d\n", buffer_head_);
        return true;
    }

    prev_byte_ = byte;
    return false;
}

void RTCM::decode()
{
    new_data_ = true;
    message_len_ = buffer_head_;
    for (auto& cb : callbacks_)
        cb(in_buffer_, buffer_head_);
}

void RTCM::registerCallback(rtcm_cb cb)
{
    callbacks_.push_back(cb);
}

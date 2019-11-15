#include <chrono>
#include <stdio.h>

#include "async_comm/udp.h"

#include "UBLOX/parsers/ubx.h"

using namespace std::chrono;
using namespace std;

#define DEG2RAD (3.14159 / 180.0)
#ifndef NDEBUG
#define DBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define DBG(...)
#endif

namespace ublox
{

UBX::UBX(async_comm::Serial& ser) :
    serial_(ser)
{
    buffer_head_ = 0;
    parse_state_ = START;
    message_class_ = 0;
    message_type_ = 0;
    length_ = 0;
    ck_a_ = 0;
    ck_b_ = 0;
    prev_byte_ = 0;
    start_message_ = false;
    new_data_ = false;
    end_message_ = false;
}

bool UBX::parsing_message()
{
    return (start_message_ == true && end_message_ == false);
}

bool UBX::new_data()
{
    bool tmp = new_data_;
    new_data_ = false;
    return tmp;
}

bool UBX::read_cb(uint8_t byte)
{
    switch (parse_state_)
    {
    case START:
        if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1)
        {
            buffer_head_ = 0;
            parse_state_ = GOT_START_FRAME;
            message_class_ = 0;
            message_type_ = 0;
            length_ = 0;
            ck_a_ = 0;
            ck_b_ = 0;
            start_message_ = true;
            end_message_ = false;
        }
        break;
    case GOT_START_FRAME:
        message_class_ = byte;
        parse_state_ = GOT_CLASS;
        break;
    case GOT_CLASS:
        message_type_ = byte;
        parse_state_ = GOT_MSG_ID;
        break;
    case GOT_MSG_ID:
        length_ = byte;
        parse_state_ = GOT_LENGTH1;
        break;
    case GOT_LENGTH1:
        length_ |= (uint16_t) byte << 8;
        parse_state_ = GOT_LENGTH2;
        if (length_ > BUFFER_SIZE)
        {
            num_errors_++;
            parse_state_ = START;
            prev_byte_ = byte;
            end_message_ = false;
            start_message_ = false;
            return false;
        }
        break;
    case GOT_LENGTH2:
        if (buffer_head_ < length_)
        {
            // push the byte onto the data buffer
            in_message_.buffer[buffer_head_] = byte;
            if (buffer_head_ == length_-1)
            {
                parse_state_ = GOT_PAYLOAD;
            }
            buffer_head_++;
        }
        break;
    case GOT_PAYLOAD:
        ck_a_ = byte;
        parse_state_ = GOT_CK_A;
        break;
    case GOT_CK_A:
        ck_b_ = byte;
        parse_state_ = GOT_CK_B;
        break;
    default:
        num_errors_++;
        parse_state_ = START;
        end_message_ = false;
        start_message_ = false;
        break;
    }

    // If we have a complete packet, then try to parse it
    if (parse_state_ == GOT_CK_B)
    {
        if (decode_message())
        {
            parse_state_ = START;
            end_message_ = true;
            start_message_ = false;
            new_data_ = true;
            prev_byte_ = byte;
            return true;
        }
        else
        {
            // indicate error if it didn't work
            DBG("\n failed to parse message\n");
            num_errors_++;
            parse_state_ = START;
            start_message_ = false;
            end_message_ = false;
        }
    }
    prev_byte_ = byte;
    return false;
}

bool UBX::decode_message()
{
    // First, check the checksum
    uint8_t ck_a, ck_b;
    calculate_checksum(message_class_, message_type_, length_, in_message_, ck_a, ck_b);
    if (ck_a != ck_a_ || ck_b != ck_b_)
        return false;
    uint8_t version; //0 poll request, 1 poll (receiver to return config data key and value pairs)
    uint8_t layer;
    uint8_t reserved1[2];
    uint32_t cfgDataKey;
    uint64_t cfgData;
    num_messages_received_++;

    // Parse the payload
    switch (message_class_)
    {
    case CLASS_ACK:
        DBG("ACK_");
        switch (message_type_)
        {
        case ACK_ACK:
            got_ack_ = true;
            DBG("ACK\n");
            break;
        case ACK_NACK:
            got_nack_ = true;
            DBG("NACK\n");
            break;
        default:
            DBG("%d\n", message_type_);
            break;
        }
        break;
   case CLASS_CFG: //only needed for getting data
       DBG("CFG_");
       switch (message_type_)
       {
       case CFG_VALGET:
       {
           DBG("VALGET = ");
           int value = in_message_.CFG_VALGET.cfgData;
           DBG("%d \n", value);
           break;
       }
       default:
           DBG("unknown: %x\n", message_type_);
           break;
       }

    default:
        break;
    }

    // call callbacks
    for (auto& cb : callbacks)
    {
        if (message_class_ == cb.cls && message_type_ == cb.type)
            cb.cb(message_class_, message_type_, in_message_);
    }

    new_data_ = true;
    return true;
}

void UBX::registerCallback(uint8_t cls, uint8_t type,
                std::function<void(uint8_t, uint8_t, const UBX_message_t&)> cb)
{
    callbacks.push_back({cls, type, cb});
}

void UBX::calculate_checksum(const uint8_t msg_cls, const uint8_t msg_id, const uint16_t len, const UBX_message_t payload, uint8_t& ck_a, uint8_t& ck_b) const
{
    if (msg_cls == 5)
        volatile int debug =1;
    ck_a = ck_b = 0;

    // Add in class
    ck_a += msg_cls;
    ck_b += ck_a;

    // Id
    ck_a += msg_id;
    ck_b += ck_a;

    // Length
    ck_a += len & 0xFF;
    ck_b += ck_a;
    ck_a += (len >> 8) & 0xFF;
    ck_b += ck_a;

    // Payload
    for (int i = 0; i < len; i ++)
    {
        ck_a += payload.buffer[i];
        ck_b += ck_a;
    }
}

bool UBX::send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t& message, uint16_t len)
{
    // First, calculate the checksum
    uint8_t ck_a, ck_b;
    calculate_checksum(msg_class, msg_id, len, message, ck_a, ck_b);

    // Send message
    serial_.send_byte(START_BYTE_1);
    serial_.send_byte(START_BYTE_2);
    serial_.send_byte(msg_class);
    serial_.send_byte(msg_id);
    serial_.send_byte(len & 0xFF);
    serial_.send_byte((len >> 8) & 0xFF);
    serial_.send_bytes(message.buffer, len);
    serial_.send_byte(ck_a);
    serial_.send_byte(ck_b);
    return true;
}

void UBX::set_nav_rate(uint8_t period_ms)
{

    DBG("Setting nav rate to %d\n", period_ms);

    configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, period_ms, CFG_VALSET_t::RATE_MEAS, byte);
    configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RATE_NAV, byte);
    configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, CFG_VALSET_t::TIME_REF_UTC, CFG_VALSET_t::RATE_TIMEREF, byte);

}

void UBX::configure(uint8_t version, uint8_t layer, uint64_t cfgData, uint32_t cfgDataKey, uint8_t size)
{
    memset(&out_message_, 0, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.version = version;
    out_message_.CFG_VALSET.layer = layer;
    if(size == byte)
    {
        out_message_.CFG_VALSET.cfgData.bytes[0] = cfgData;
    }
    if(size == word)
    {
        out_message_.CFG_VALSET.cfgData.word = cfgData;
    }
    out_message_.CFG_VALSET.cfgDataKey = cfgDataKey;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
}

void UBX::get_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
{
       memset(&out_message_, 0, sizeof(CFG_VALGET_t));
       out_message_.CFG_VALGET.version = version;
       out_message_.CFG_VALGET.layer = layer;
       out_message_.CFG_VALGET.cfgDataKey = cfgDataKey;
       send_message(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t));
}

//Deletes configuration values specified by the key
void UBX::del_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
{
    memset(&out_message_, 0, sizeof(CFG_VALDEL_t));
    out_message_.CFG_VALDEL.version = version;
    out_message_.CFG_VALDEL.layer = layer;
    out_message_.CFG_VALDEL.cfgDataKey = cfgDataKey;
    send_message(CLASS_CFG, CFG_VALDEL, out_message_, sizeof(CFG_VALDEL_t));
}
}

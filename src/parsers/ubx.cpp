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

void UBX::registerCallback(uint8_t cls, uint8_t type,
                std::function<void(uint8_t, uint8_t, const UBX_message_t&)> cb)
{
    callbacks.push_back({cls, type, cb});
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

size_t UBX::num_messages_received()
{
    return num_messages_received_;
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

void UBX::set_dynamic_mode()
{
    memset(&out_message_, 0, sizeof(CFG_NAV5_t));
    out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
    out_message_.CFG_NAV5.dynModel = CFG_NAV5_t::DYNMODE_AIRBORNE_4G;
    DBG("Setting dynamic mode\n");
    send_message(CLASS_CFG, CFG_NAV5, out_message_, sizeof(CFG_NAV5_t));
}

void UBX::set_nav_rate(uint8_t period_ms)
{
    memset(&out_message_, 0, sizeof(CFG_RATE_t));
    out_message_.CFG_RATE.measRate = period_ms;
    out_message_.CFG_RATE.navRate = 1;
    out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_GPS;
    DBG("Setting nav rate to %d\n", period_ms);
    send_message(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
}

void UBX::enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate)
{
    memset(&out_message_, 0, sizeof(CFG_MSG_t));
    out_message_.CFG_MSG.msgClass = msg_cls;
    out_message_.CFG_MSG.msgID = msg_id;
    out_message_.CFG_MSG.rate = rate;
    DBG("Requesting %x:%x message with period=%d\n", msg_cls, msg_id, rate);
    send_message(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
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
        //DBG("Started %x-%x\n", message_class_, message_type_);
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
            //printf("UBX length: %d \n", length_);
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

    num_messages_received_++;
//    DBG("recieved message %d: ", num_messages_received_);
//    DBG("of type 0x%2x:0x%2x\n", message_class_, message_type_);

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
      // case CLASS_RXM:
      //   DBG("RXM_");
      //   switch(message_type_)
      //   {
      //   case RXM_RAWX:
      //       DBG("RAWX\n");
      //       break;
      //   case RXM_SFRBX:
      //       DBG("SFRBX\n");
      //       break;
      //   }
//    case CLASS_NAV:
//        DBG("NAV_");
//        switch (message_type_)
//        {
//        case NAV_PVT:
//            DBG("PVT \n");
//            break;
//        case NAV_RELPOSNED:
//            DBG("RELPOSNED \n");
//            break;
//        default:
//            DBG("%d \n", message_type_);
//            break;
//        }
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

void UBX::turnOnRTCM()
{
    memset(&out_message_, 0, CFG_VALSET_t::LEN_BYTE);
    out_message_.CFG_VALSET.version = CFG_VALSET_t::VALSET_0;
    out_message_.CFG_VALSET.layer = CFG_VALSET_t::VALSET_RAM;
    out_message_.CFG_VALSET.cfgData.bytes[0] = CFG_VALSET_t::DYNMODE_AIRBORNE_1G;
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::VALSET_DYNMODEL;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));

    bool poll = true;
    if(poll == true)
        poll_value();
}

void UBX::config_rover()
{
    memset(&out_message_, 0, CFG_VALSET_t::LEN_BYTE);
    out_message_.CFG_VALSET.version = CFG_VALSET_t::VALSET_0;
    out_message_.CFG_VALSET.layer = CFG_VALSET_t::VALSET_RAM;
    out_message_.CFG_VALSET.cfgData.bytes[0] = 1;
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::VALSET_MSGOUT_RELPOSNED;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));

}

void UBX::config_base()
{
    bool mobile = true;
    if(mobile == true)
        config_base_mobile();
    else
        config_base_stationary();

}

void UBX::config_base_stationary()
{

    memset(&out_message_, 0, CFG_VALSET_t::LEN_BYTE);
    out_message_.CFG_VALSET.version = CFG_VALSET_t::VALSET_0;
    out_message_.CFG_VALSET.layer = CFG_VALSET_t::VALSET_RAM;
    out_message_.CFG_VALSET.cfgData.bytes[0] = 1;
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1005USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1074USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1084USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1094USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1124USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1230USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));

    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::TMODE_MODE;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::VALSET_MSGOUT_SVIN;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    memset(&out_message_, 0, CFG_VALSET_t::LEN_4BYTE);
    out_message_.CFG_VALSET.version = CFG_VALSET_t::VALSET_0;
    out_message_.CFG_VALSET.layer = CFG_VALSET_t::VALSET_RAM;
    out_message_.CFG_VALSET.cfgData.word = 500000; //mm
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::TMODE_SVIN_ACC_LIMIT;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgData.word = 120;
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::TMODE_SVIN_MIN_DUR;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
}

void UBX::config_base_mobile()
{


    memset(&out_message_, 0, CFG_VALSET_t::LEN_BYTE);
    out_message_.CFG_VALSET.version = CFG_VALSET_t::VALSET_0;
    out_message_.CFG_VALSET.layer = CFG_VALSET_t::VALSET_RAM;
    out_message_.CFG_VALSET.cfgData.bytes[0] = 1;
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_4072_0USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_4072_1USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1077USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1087USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1097USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1127USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1230USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
}

void UBX::poll_value()
{
       in_message_.CFG_VALGET.cfgData = 0; //clear value
       memset(&out_message_, 0, sizeof(CFG_VALGET_t));
       out_message_.CFG_VALGET.version = CFG_VALGET_t::VALGET_REQUEST;
       out_message_.CFG_VALGET.layer = CFG_VALGET_t::VALGET_RAM;
       out_message_.CFG_VALGET.cfgDataKey = CFG_VALGET_t::RXM_SFRBX;
       send_message(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t));
}
}

//void UBX::set_baudrate(const uint32_t baudrate)
//{
//  DBG("Setting baudrate to %d\n", baudrate);
//  // Now that we have the right baudrate, let's configure the thing
//  memset(&out_message_, 0, sizeof(CFG_PRT_t));
//  out_message_.CFG_PRT.portID = CFG_PRT_t::PORT_USB;
//  out_message_.CFG_PRT.baudrate = baudrate;
//  out_message_.CFG_PRT.inProtoMask = CFG_PRT_t::IN_UBX | CFG_PRT_t::IN_NMEA | CFG_PRT_t::IN_RTCM | CFG_PRT_t::IN_RTCM3;
//  out_message_.CFG_PRT.outProtoMask = CFG_PRT_t::OUT_UBX | CFG_PRT_t::OUT_NMEA | CFG_PRT_t::OUT_RTCM3;
//  out_message_.CFG_PRT.mode = CFG_PRT_t::CHARLEN_8BIT | CFG_PRT_t::PARITY_NONE | CFG_PRT_t::STOP_BITS_1;
//  out_message_.CFG_PRT.flags = 0;
//  send_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CFG_PRT_t));
//  usleep(10000);
//  serial_.set_baud_rate(baudrate);
//  current_baudrate_ = baudrate;
//}

//bool UBX::detect_baudrate()
//{
//  serial_.init();
//  current_baudrate_ = 0;
//  for (int i = 0; i < sizeof(baudrates)/sizeof(uint32_t); i++)
//  {
//    DBG("Trying %d baudrate\n", baudrates[i]);
//    serial_.set_baud_rate(baudrates[i]);
//    milliseconds timeout_ms(1000);
//    milliseconds start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
//    milliseconds now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
//    while (now < start + timeout_ms)
//    {
//      now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
//      if (got_message_)
//      {
//        DBG("Found UBX at %d baud\n", baudrates[i]);
//        current_baudrate_ = baudrates[i];
//        break;
//      }
//    }
//    if (current_baudrate_ != 0)
//      break;
//  }
//  return got_message_;
//}

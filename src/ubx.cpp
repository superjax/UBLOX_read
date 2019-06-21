#include "ublox.h"
#include <chrono>
#include <stdio.h>
#include <async_comm/udp.h>

using namespace std::chrono;

#define DEG2RAD (3.14159 / 180.0)
#define DBG(...) fprintf(stderr, __VA_ARGS__)
//#define DBG(...)

bool UBX::read_ubx(uint8_t byte)
{
    switch (parse_state_)
    {
    case START:

        buffer_head_ = 0;
        parse_state_ = GOT_START_FRAME;
        message_class_ = 0;
        message_type_ = 0;
        length_ = 0;
        ck_a_ = 0;
        ck_b_ = 0;
        got_message_ = true;
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
      if (length_ > UBLOX_BUFFER_SIZE)
      {
        num_errors_++;
        parse_state_ = START;
        std::cout << "\n the message is too big" << "\n";
        buffer_head_ = 0;
        //looking_for_rtcm_ = true;
        //ubx = false;
        return(false);
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
      DBG("default");
      num_errors_++;
      parse_state_ = START;
      //looking_for_rtcm_ = true;
      //ubx = false;
      return(false);
    }

    // If we have a complete packet, then try to parse it
    if (parse_state_ == GOT_CK_B)
    {
      if (1)//decode_message())
      {
        parse_state_ = START;
        DBG("SUCCESS");
        return(false);
      }
//      else
//      {
//        // indicate error if it didn't work
//        num_errors_++;
//        DBG("\n failed to parse message\n");
//        parse_state_ = START;
//      }
      //looking_for_rtcm_ = true;
      //ubx = false;
    }
    return(true);
}

#include "ublox.h"
#include <chrono>
#include <stdio.h>
using namespace std::chrono;

#define DEG2RAD (3.14159 / 180.0)
//#define DBG(...) fprintf(stderr, __VA_ARGS__)
#define DBG(...)

UBLOX::UBLOX(std::string port) :
  serial_(port, 115200)
{}

void UBLOX::init()
{
  // Reset message parser
  buffer_head_ = 0;
  parse_state_ = START;
  message_class_ = 0;
  message_type_ = 0;
  length_ = 0;
  ck_a_ = 0;
  ck_b_ = 0;
  
  // Find the right baudrate
  looking_for_nmea_ = true;
  auto cb = [this](const uint8_t* buffer, size_t size)
  {
    for (int i = 0; i < size; i++)
      this->read_cb(buffer[i]);
  };

  serial_.register_receive_callback(cb);

  if (!detect_baudrate())
    return;
  
  // Otherwise, Configure the GPS
  set_baudrate(115200);
  set_dynamic_mode();
  set_nav_rate(100);
  enable_message(CLASS_NAV, NAV_PVT, 10);
  enable_message(CLASS_NAV, NAV_POSECEF, 10);
  enable_message(CLASS_NAV, NAV_VELECEF, 10);
}

bool UBLOX::detect_baudrate()
{
  serial_.init();
  current_baudrate_ = 0;
  for (int i = 0; i < sizeof(baudrates)/sizeof(uint32_t); i++)
  {
    DBG("Trying %d baudrate\n", baudrates[i]);
    serial_.set_baud_rate(baudrates[i]);
    milliseconds timeout_ms(1000);
    milliseconds start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
    milliseconds now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
    while (now < start + timeout_ms)
    {
      now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
      if (got_message_)
      {
        DBG("Found UBLOX at %d baud\n", baudrates[i]);
        current_baudrate_ = baudrates[i];
        break;
      }
    }
    if (current_baudrate_ != 0)
      break;
  }
  return got_message_;
}

bool UBLOX::send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t& message, uint16_t len)
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

void UBLOX::set_baudrate(const uint32_t baudrate)
{
  DBG("Setting baudrate to %d\n", baudrate);
  // Now that we have the right baudrate, let's configure the thing
  memset(&out_message_, 0, sizeof(CFG_PRT_t));
  out_message_.CFG_PRT.portID = CFG_PRT_t::PORT_UART1;
  out_message_.CFG_PRT.baudrate = baudrate;
  out_message_.CFG_PRT.inProtoMask = CFG_PRT_t::IN_UBX | CFG_PRT_t::IN_NMEA | CFG_PRT_t::IN_RTCM;
  out_message_.CFG_PRT.outProtoMask = CFG_PRT_t::OUT_UBX | CFG_PRT_t::OUT_NMEA;
  out_message_.CFG_PRT.mode = CFG_PRT_t::CHARLEN_8BIT | CFG_PRT_t::PARITY_NONE | CFG_PRT_t::STOP_BITS_1;
  out_message_.CFG_PRT.flags = 0;
  send_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CFG_PRT_t));
  usleep(10000);
  serial_.set_baud_rate(baudrate);
  current_baudrate_ = baudrate;
}

void UBLOX::set_dynamic_mode()
{
  memset(&out_message_, 0, sizeof(CFG_NAV5_t));
  out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
  out_message_.CFG_NAV5.dynModel = CFG_NAV5_t::DYNMODE_AIRBORNE_4G;
  DBG("Setting dynamic mode\n");
  send_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CFG_NAV5_t));
}

void UBLOX::set_nav_rate(uint8_t period_ms)
{
  memset(&out_message_, 0, sizeof(CFG_RATE_t));
  out_message_.CFG_RATE.measRate = period_ms;
  out_message_.CFG_RATE.navRate = 1;
  out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_GPS;
  DBG("Setting nav rate to %d\n", period_ms);
  send_message(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
}

void UBLOX::enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate)
{
  memset(&out_message_, 0, sizeof(CFG_MSG_t));
  out_message_.CFG_MSG.msgClass = msg_cls;
  out_message_.CFG_MSG.msgID = msg_id;
  out_message_.CFG_MSG.rate = rate;
  DBG("Requesting %x:%x message at %d hz\n", msg_cls, msg_id, rate);
  send_message(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
}


void UBLOX::read_cb(uint8_t byte)
{
  // Look for a valid NMEA packet (do this at the beginning in case
  // UBX was disabled for some reason) and during autobaud
  // detection
  if (looking_for_nmea_)
  {
    if (byte == NMEA_START_BYTE2 && prev_byte_ == NMEA_START_BYTE1)
    {
      got_message_ = true;
      looking_for_nmea_ = false;
    }
  }

  // handle the UBX packet
  switch (parse_state_)
  {
  case START:
    if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1)
    {
      looking_for_nmea_ = false;
      buffer_head_ = 0;
      parse_state_ = GOT_START_FRAME;
      message_class_ = 0;
      message_type_ = 0;
      length_ = 0;
      ck_a_ = 0;
      ck_b_ = 0;
      got_message_ = true;
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
    if (length_ > UBLOX_BUFFER_SIZE)
    {
      num_errors_++;
      parse_state_ = START;
      return;
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
    break;
  }

  // If we have a complete packet, then try to parse it
  if (parse_state_ == GOT_CK_B)
  {
    if (decode_message())
    {
      parse_state_ = START;
    }
    else
    {
      // indicate error if it didn't work
      num_errors_++;
      DBG("failed to parse message\n");
      parse_state_ = START;
    }
  }

  prev_byte_ = byte;
}

void UBLOX::read(double* lla, float* vel, uint8_t& fix_type, uint32_t& t_ms)
{
  if (new_data_)
  {
    convert_data();
    new_data_ = false;
  }
  for (int i = 0; i < 3; i++)
  {
    lla[i] = lla_[i];
    vel[i] = vel_[i];
  }
  fix_type = nav_message_.fixType;
}

bool UBLOX::decode_message()
{
  // First, check the checksum
  uint8_t ck_a, ck_b;
  calculate_checksum(message_class_, message_type_, length_, in_message_, ck_a, ck_b);
  if (ck_a != ck_a_ || ck_b != ck_b_)
    return false;

  num_messages_received_++;
  DBG("recieved message %d: ", num_messages_received_);

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

  case CLASS_CFG:
    DBG("CFG_");
    switch (message_type_)
    {
    default:
      DBG("%d\n", message_type_);
      break;
    }
    break;

  case CLASS_NAV:
    DBG("NAV_");
    switch (message_type_)
    {
    case NAV_PVT:
      new_data_ = true;
      nav_message_ = in_message_.NAV_PVT;
      DBG("PVT\n");
      break;
    case NAV_POSECEF:
      new_data_ = true;
      pos_ecef_ = in_message_.NAV_POSECEF;
      DBG("POSECEF\n");
      break;
    case NAV_VELECEF:
      new_data_ = true;
      vel_ecef_ = in_message_.NAV_VELECEF;
      DBG("NAVELECEF\n");
      break;
    default:
      DBG("%d\n", message_type_);
      break;
    }
    break;
  default:
    DBG("%d_%d\n", message_class_, message_type_);
    break;
  }
  return true;
}

void UBLOX::convert_data()
{
  static const double scaling = 1e-7 * M_PI/180.0;
  lla_[0] = (double)(nav_message_.lat) * scaling;
  lla_[1] = (double)(nav_message_.lon) * scaling;
  lla_[2] = nav_message_.height * 1e-3;

  vel_[0] = nav_message_.velN * 1e-3;
  vel_[1] = nav_message_.velE * 1e-3;
  vel_[2] = nav_message_.velD * 1e-3;
}

void UBLOX::calculate_checksum(const uint8_t msg_cls, const uint8_t msg_id, const uint16_t len, const UBX_message_t payload, uint8_t& ck_a, uint8_t& ck_b) const
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


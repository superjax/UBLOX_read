#ifndef RTCM_H
#define RTCM_H

#include <stdio.h>
#include <stdint.h>

#include "UBLOX/bitfield.h"

#include "async_comm/comm.h"

class RTCM
{
public:
    const static uint32_t CRC24_TABLE[];
    static constexpr int BUFFER_SIZE = 1024;
    RTCM();



    bool read_cb(uint8_t byte);
    bool parsing_message();
    bool new_data();
    volatile bool new_data_;

    void decode();
    bool  check_crc();

    // Working Memory Variables
    size_t message_len_; // length, including header and footer
    size_t buffer_head_;
    size_t payload_len_;
    bool start_message_;
    bool end_message_;
    uint8_t ck_a_, ck_b_, ck_c_;
    uint16_t msg_type_;
    uint8_t prev_byte_;
    size_t num_errors_;

    enum {
        START_BYTE = 0xD3
    };

    typedef enum {
      START,
      GOT_START_FRAME,
      GOT_CLASS,
      GOT_MSG_ID,
      GOT_LENGTH1,
      GOT_LENGTH2,
      GOT_PAYLOAD,
      GOT_CK_A,
      GOT_CK_B,
      GOT_CK_C,
      DONE,
    } parse_state_t;
    parse_state_t parse_state_;

    typedef std::function<void(uint8_t*, size_t)> rtcm_cb;
    std::vector<rtcm_cb> callbacks_;
    void registerCallback(rtcm_cb cb);

private:
    struct RTCM1004
    {
        BitField<0, 8> header;
        BitField<8, 6> res;
        BitField<14, 10> len;
        BitField<24, 10> type;
    };

    union
    {
        uint8_t buf[BUFFER_SIZE];
        RTCM1004 rtcm1004;
    } in_buffer_;
    uint32_t canary_ = 0xCAFEBABE;

};



#endif

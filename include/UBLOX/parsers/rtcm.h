#ifndef RTCM_H
#define RTCM_H

#include <cstdio>
#include <cstdint>
#include <ctime>

#include "UBLOX/bitfield.h"
#include "UBLOX/utctime.h"
#include "UBLOX/parsers/msm.h"

#include "async_comm/comm.h"
#include "UBLOX/parsers/rtcm_defs.h"

namespace rtcm
{

class RTCM
{
public:
    const static uint32_t CRC24_TABLE[];
    static constexpr int BUFFER_SIZE = 1024;

    union RTCM_message_t
    {
        uint8_t buf[BUFFER_SIZE];
//        RTCM1001 rtcm1001;
//        RTCM1002 rtcm1002;
//        RTCM1003 rtcm1003;
//        RTCM1004 rtcm1004;
//        RTCM1005 rtcm1005;
//        RTCM1006 rtcm1006;
//        RTCM1009 rtcm1009;
//        RTCM1010 rtcm1010;
//        RTCM1011 rtcm1011;
//        RTCM1012 rtcm1012;
//        RTCM1019 rtcm1019;
//        RTCM1020 rtcm1020;

//        BitField<0,8> hdr;
//        BitField<8,6> res;
//        BitField<14,10> len;
//        BitField<24,12> type;

//        MSM4<GPS> rtcm1074;
//        MSM4<Galileo> rtcm1084;
//        MSM4<Glonass> rtcm1094;
//        MSM4<Beidou> rtcm1124;
//        MSM5<GPS> rtcm1075;
//        MSM5<Galileo> rtcm1085;
//        MSM5<Glonass> rtcm1095;
//        MSM5<Beidou> rtcm1125;
//        MSM7<GPS> rtcm1077;
//        MSM7<Galileo> rtcm1087;
//        MSM7<Glonass> rtcm1097;
//        MSM7<Beidou> rtcm1127;

//        RTCM_message_t() {}
    } in_buffer_;
    uint32_t canary_ = 0xCAFEBABE;

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
        START_BYTE = 0xD3,
        ID_ALL = 0xFFFF
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

//    typedef std::function<void(uint8_t, const RTCM_message_t&)> rtcm_cb;
//    struct callback_t
//    {
//        uint16_t rtcm_msg;
//        rtcm_cb cb;
//    };
//    void registerCallback(uint16_t msg_id, rtcm_cb cb);
//    std::vector<callback_t> callbacks_;

    typedef std::function<void(uint8_t*, size_t)> buffer_cb;
    void registerBufferCallback(buffer_cb cb);
    std::vector<buffer_cb> buffer_callbacks_;
};
}

#endif

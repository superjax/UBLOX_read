#ifndef RTCM_H
#define RTCM_H

#include <stdio.h>
#include <stdint.h>

#include "async_comm/comm.h"

class RTCM
{
public:
    static constexpr int BUFFER_SIZE = 1022;
    RTCM();
    uint8_t in_buffer_[BUFFER_SIZE];
    uint8_t out_buffer_[BUFFER_SIZE];

    bool read_cb(uint8_t byte);
    inline volatile bool new_data() const { return new_data_; }
    volatile bool new_data_;

    void decode();

    // Working Memory Variables
    size_t message_len_; // length, including header and footer
    size_t buffer_head_;
    size_t payload_len_;
    bool got_message_;
    uint8_t ck_a_, ck_b_, ck_c_;
    uint8_t prev_byte_;
    size_t num_errors_;

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

    std::mutex msg_lock_;
};



#endif

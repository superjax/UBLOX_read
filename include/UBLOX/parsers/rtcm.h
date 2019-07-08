#ifndef RTCM_H
#define RTCM_H

#include <stdio.h>
#include <stdint.h>
#include <bitset>

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
    union RTCM1001x4_HDR
    {
      BitField<36,12> staid;
      BitField<48,30> tow; // 1e-3 sec
      BitField<78, 1> sync;
      BitField<79, 5> nsat;
      BitField<84, 1> GPS_smooth_indicator;
      BitField<85, 3> GPS_smooth_interval;
    };

    // GPS RTK L1-only
    union RTCM1001
    {
      RTCM1001x4_HDR hdr;
      BitField<88, 6> sat_id;
      BitField<94, 1> l1_code;
      BitField<95, 24> l1_pseudorange;
      BitField<119, 20> l1_phase_range;
      BitField<139, 7> l1_lock_time;
    };

    // GPS Extended RTK, L1-only
    union RTCM1002
    {
      RTCM1001x4_HDR hdr;
      BitField<88, 6> sat_id;
      BitField<94, 1> l1_code;
      BitField<95, 24> l1_pseudorange;
      BitField<119, 20> l1_phase_range;
      BitField<139, 7> l1_lock_time;
      BitField<146, 8> l1_psuedorange_modulus_ambiguity;
      BitField<154, 8> l1_CNR;
    };

    // GPS Basic RTK, L1/L2
    union RTCM1003
    {
      RTCM1001x4_HDR hdr;
      BitField<88, 6> sat_id;
      BitField<94, 1> l1_code;
      BitField<95, 24> l1_pseudorange;
      BitField<119, 20> l1_phase_range;
      BitField<139, 7> l1_lock_time;
      BitField<146, 2> l2_code;
      BitField<148, 14> l1_l2_psuedorange_diff;
      BitField<162, 20> l2_phaserange_l1_psueodrange;
      BitField<182, 7> l2_lock_time;
    };

    // GPS Extended RTK, L1/L2
    union RTCM1004
    {
      RTCM1001x4_HDR hdr;
      BitField<88, 6> sat_id;
      BitField<94, 1> l1_code;
      BitField<95, 24> l1_pseudorange;
      BitField<119, 20> l1_phase_range;
      BitField<139, 7> l1_lock_time;
      BitField<146, 8> l1_psuedorange_modulus_ambiguity;
      BitField<154, 8> l1_CNR;
      BitField<164, 2> l2_code;
      BitField<166, 14> l1_l2_psuedorange_diff;
      BitField<180, 20> l2_phaserange_l1_psueodrange;
      BitField<200, 7> l2_lock_time;
      BitField<207, 8> l2_cnr;
    };

    // Stationary Antenna Reference w/o Height
    union RTCM1005
    {
      BitField<36,12> staid;
      BitField<48,6> res;
      BitField<54,1> GPS_indicator;
      BitField<55,1> GLONASS_indicator;
      BitField<56,1> Galileo_indicator;
      BitField<57,1> Reference_indicator;
      BitField<58,38> ECEF_x;
      BitField<96,1> single_receiver_oscillator_indicator;
      BitField<97,1> res2;
      BitField<98,38> ECEF_y;
      BitField<136,2> res3;
      BitField<138,38> ECEF_z;
    };

    // Stationary Antenna Ref w/ Height
    union RTCM1006
    {
      BitField<36,12> staid;
      BitField<48,6> res;
      BitField<54,1> GPS_indicator;
      BitField<55,1> GLONASS_indicator;
      BitField<56,1> Galileo_indicator;
      BitField<57,1> Reference_indicator;
      BitField<58,38> ECEF_x;
      BitField<96,1> single_receiver_oscillator_indicator;
      BitField<97,1> res2;
      BitField<98,38> ECEF_y;
      BitField<136,2> res3;
      BitField<138,38> ECEF_z;
      BitField<176,16> height;
    };

    // 1007 Antenna Descriptor (not supported)
    // 1008 Antenna Descriptor & Serial Number (not supported)

    union RTCM1009x12_HDR
    {
      BitField<36,12> staid;
      BitField<48,27> GLONASS_epoch_time;
      BitField<75,1> synchronous_GNSS_flag;
      BitField<76,5> num_signals;
      BitField<81,1> smoothing_ind;
      BitField<82,3> smoothing_interval;
    };

    // Glonass RTK Basic RTK L1-only
    union RTCM1009
    {
      RTCM1009x12_HDR hdr;
      BitField<85,6> sat_id;
      BitField<91,1> code_ind;
      BitField<92,5> freq_channel;
      BitField<97,25> l1_pseudorange;
      BitField<122,20> l1_phaserange_l1_pseudorange;
      BitField<142,7> l1_lock_time;
    };

    // Glonass RTK Extended RTK L1-only
    union RTCM1010
    {
      RTCM1009x12_HDR hdr;
      BitField<85,6> sat_id;
      BitField<91,1> code_ind;
      BitField<92,5> freq_channel;
      BitField<97,25> l1_pseudorange;
      BitField<122,20> l1_phaserange_l1_pseudorange;
      BitField<142,7> l1_lock_time;
      BitField<149,7> l1_pseudorange_amb;
      BitField<156,8> l1_cnr;
    };

    // GLONASS Basic RTK L1/L2
    union RTCM1011
    {
      RTCM1009x12_HDR hdr;
      BitField<85,6> sat_id;
      BitField<91,1> l1_code_ind;
      BitField<92,5> freq_channel;
      BitField<97,25> l1_pseudorange;
      BitField<122,20> l1_phaserange_l1_pseudorange;
      BitField<142,7> l1_lock_time;
      BitField<149,2> l2_code_ind;
      BitField<151,14> l1_l2_pseudorange_diff;
      BitField<165,20> l1_phaserange_l2_pseudorange;
      BitField<185,7> l2_lock_time_ind;
    };

    // GLONASS Extended RTK L1/L2
    union RTCM1012
    {
      RTCM1009x12_HDR hdr;
      BitField<85,6> sat_id;
      BitField<91,1> l1_code_ind;
      BitField<92,5> freq_channel;
      BitField<97,25> l1_pseudorange;
      BitField<122,20> l1_phaserange_l1_pseudorange;
      BitField<142,7> l1_lock_time;
      BitField<149,7> l1_pseudorange_amb;
      BitField<156,8> l1_cnr;
      BitField<164,2> l2_code_ind;
      BitField<166,14> l1_l2_pseudorange_diff;
      BitField<180,20> l1_phaserange_l2_pseudorange;
      BitField<200,7> l2_lock_time_ind;
      BitField<207,8> l2_cnr;
    };

    // RTCM1013 - System Parameters (not supported)
    // RTCM1014 - Network Auxiliary Station Data Message (not supported)
    // RTCM1015 - GPS Ionospheric Correction Differences (not supported)
    // RTCM1016 - GPS Geometric Correction Differences (not supported)
    // RTCM1017 - GPS Combined Geometric and Ionospheric Correction Differences (not supported)

    // GPS Ephemerides
    // See GPS SPS 2.4.3 for descriptions
    union RTCM1019
    {
      BitField<36,6> sat_id;
      BitField<42,10> week;
      BitField<32,4> sv_acc;
      BitField<36,2> l2_code;
      BitField<38,14> idot;
      BitField<52,8> iode;
      BitField<60,16> toc;
      BitField<76,8> af2;
      BitField<84,16> af1;
      BitField<100,22> af0;
      BitField<122,10> iodc;
      BitField<132,16> crs;
      BitField<158,16> delta_n;
      BitField<174,32> M0;
      BitField<206,16> cuc;
      BitField<222,32> e;
      BitField<254,16> cus;
      BitField<270,32> sqrt_A;
      BitField<302,16> toe;
      BitField<318,16> cic;
      BitField<334,32> Omega0;
      BitField<366,16> cis;
      BitField<382,32> i0;
      BitField<414,16> crc;
      BitField<432,32> w;
      BitField<464,24> Omegadot;
      BitField<488,8> tgd;
      BitField<496,6> sv_health;
      BitField<512,1> l2_P;
      BitField<513,1> GPS_fit;
    };

    // GLONASS Ephemerides
    // See GLONASS ICD Version 5.0 for descriptions
    union RTCM1020
    {
      BitField<36,6> sat_id;
      BitField<42,5> freq_chan;
      BitField<47,1> alm_health;
      BitField<48,1> alm_health_avail;
      BitField<49,2> P1;
      BitField<51,12> tk;
      BitField<63,1> Bn_MSB;
      BitField<64,1> P2;
      BitField<65,7> tb;
      BitField<72,24> xdot;
      BitField<96,27> xt;
      BitField<123,5> xddot;
      BitField<128,24> ydot;
      BitField<152,27> y;
      BitField<179,5> yddot;
      BitField<184,24> zdot;
      BitField<208,27> z;
      BitField<235,5> zddot;
      BitField<240,1> P3;
      BitField<241,11> gm;
      BitField<252,2> MP;
      BitField<244,1> M1;
      BitField<245,22> tau;
      BitField<267,5> delta_tau;
      BitField<273,5> E;
      BitField<278,1> P4;
      BitField<279,4> F;
      BitField<283,11> N;
      BitField<294,2> M;
      BitField<296,1> avail;
      BitField<297,11> NA;
      BitField<306,32> tau_c;
      BitField<338,5> N4;
      BitField<343,22> tau_GPS;
      BitField<365,1> One_M;
    };

    union MSM_HDR
    {
      BitField<36,12> sta_id;
      BitField<48,30> gnss_time;
      BitField<78,1> multiple_msg;
      BitField<79,3> iods;
      BitField<82,7> res;
      BitField<89,2> clock_steering_ind;
      BitField<91,2> external_clk_ind;
      BitField<93,1> gnss_diverg_smooth_ind;
      BitField<94,3> gnss_smoothing_interval;
      BitField<97,64> gnss_sat_mask;
      BitField<161,32> gnss_signal_mask;
    };

    class MSM
    {
      MSM_HDR hdr;

      size_t nSat;
      size_t nSat()
      {
        std::bitset<64> bits(hdr.gnss_sat_mask);
        return bits.count();
      }


    };


    // 1074 GPS MSM4
    // 1075 GPS MSM5
    // 1077 GPS MSM7
    // 1084 GLONASS MSM4
    // 1085 GLONASS MSM5
    // 1096 GLONASS MSM7
    // 1094 Galileo MSM4
    // 1095 Galileo MSM5
    // 1097 Galileo MSM7
    // 1124 BeiDou MSM4
    // 1125 BeiDou MSM5
    // 1127 BeiDou MSM7
    // 1230 GLONASS code-phase biases


    union Buffer
    {
        uint8_t buf[BUFFER_SIZE];
        RTCM1001 rtcm1001;
        RTCM1002 rtcm1002;
        RTCM1003 rtcm1003;
        RTCM1004 rtcm1004;
        RTCM1005 rtcm1005;
        RTCM1006 rtcm1006;
        RTCM1009 rtcm1009;
        RTCM1010 rtcm1010;
        RTCM1011 rtcm1011;
        RTCM1012 rtcm1012;
        RTCM1019 rtcm1019;
        RTCM1020 rtcm1020;
        BitField<0,8> hdr;
        BitField<8,6> res;
        BitField<14,10> len;
        BitField<24,12> type;
    } in_buffer_;
    uint32_t canary_ = 0xCAFEBABE;

};



#endif

//#pragma once
//#include "UBLOX/bitfield.h"

//namespace rtcm
//{

//union RTCM1001x4_HDR
//{
//    BitField<36,12> staid;
//    BitField<48,30> tow; // 1e-3 sec
//    BitField<78, 1> sync;
//    BitField<79, 5> nsat;
//    BitField<84, 1> GPS_smooth_indicator;
//    BitField<85, 3> GPS_smooth_interval;
//};

//// GPS RTK L1-only
//union RTCM1001
//{
//    RTCM1001x4_HDR hdr;
//    BitField<88, 6> sat_id;
//    BitField<94, 1> l1_code;
//    BitField<95, 24> l1_pseudorange;
//    BitField<119, 20> l1_phase_range;
//    BitField<139, 7> l1_lock_time;
//};

//// GPS Extended RTK, L1-only
//union RTCM1002
//{
//    RTCM1001x4_HDR hdr;
//    BitField<88, 6> sat_id;
//    BitField<94, 1> l1_code;
//    BitField<95, 24> l1_pseudorange;
//    BitField<119, 20> l1_phase_range;
//    BitField<139, 7> l1_lock_time;
//    BitField<146, 8> l1_psuedorange_modulus_ambiguity;
//    BitField<154, 8> l1_CNR;
//};

//// GPS Basic RTK, L1/L2
//union RTCM1003
//{
//    RTCM1001x4_HDR hdr;
//    BitField<88, 6> sat_id;
//    BitField<94, 1> l1_code;
//    BitField<95, 24> l1_pseudorange;
//    BitField<119, 20> l1_phase_range;
//    BitField<139, 7> l1_lock_time;
//    BitField<146, 2> l2_code;
//    BitField<148, 14> l1_l2_psuedorange_diff;
//    BitField<162, 20> l2_phaserange_l1_psueodrange;
//    BitField<182, 7> l2_lock_time;
//};

//// GPS Extended RTK, L1/L2
//union RTCM1004
//{
//    RTCM1001x4_HDR hdr;
//    BitField<88, 6> sat_id;
//    BitField<94, 1> l1_code;
//    BitField<95, 24> l1_pseudorange;
//    BitField<119, 20> l1_phase_range;
//    BitField<139, 7> l1_lock_time;
//    BitField<146, 8> l1_psuedorange_modulus_ambiguity;
//    BitField<154, 8> l1_CNR;
//    BitField<164, 2> l2_code;
//    BitField<166, 14> l1_l2_psuedorange_diff;
//    BitField<180, 20> l2_phaserange_l1_psueodrange;
//    BitField<200, 7> l2_lock_time;
//    BitField<207, 8> l2_cnr;
//};

//// Stationary Antenna Reference w/o Height
//union RTCM1005
//{
//    BitField<36,12> staid;
//    BitField<48,6> res;
//    BitField<54,1> GPS_indicator;
//    BitField<55,1> GLONASS_indicator;
//    BitField<56,1> Galileo_indicator;
//    BitField<57,1> Reference_indicator;
//    BitField<58,38> ECEF_x;
//    BitField<96,1> single_receiver_oscillator_indicator;
//    BitField<97,1> res2;
//    BitField<98,38> ECEF_y;
//    BitField<136,2> res3;
//    BitField<138,38> ECEF_z;
//};

//// Stationary Antenna Ref w/ Height
//union RTCM1006
//{
//    BitField<36,12> staid;
//    BitField<48,6> res;
//    BitField<54,1> GPS_indicator;
//    BitField<55,1> GLONASS_indicator;
//    BitField<56,1> Galileo_indicator;
//    BitField<57,1> Reference_indicator;
//    BitField<58,38> ECEF_x;
//    BitField<96,1> single_receiver_oscillator_indicator;
//    BitField<97,1> res2;
//    BitField<98,38> ECEF_y;
//    BitField<136,2> res3;
//    BitField<138,38> ECEF_z;
//    BitField<176,16> height;
//};

//// 1007 Antenna Descriptor (not supported)
//// 1008 Antenna Descriptor & Serial Number (not supported)

//union RTCM1009x12_HDR
//{
//    BitField<36,12> staid;
//    BitField<48,27> GLONASS_epoch_time;
//    BitField<75,1> synchronous_GNSS_flag;
//    BitField<76,5> num_signals;
//    BitField<81,1> smoothing_ind;
//    BitField<82,3> smoothing_interval;
//};

//// Glonass RTK Basic RTK L1-only
//union RTCM1009
//{
//    RTCM1009x12_HDR hdr;
//    BitField<85,6> sat_id;
//    BitField<91,1> code_ind;
//    BitField<92,5> freq_channel;
//    BitField<97,25> l1_pseudorange;
//    BitField<122,20> l1_phaserange_l1_pseudorange;
//    BitField<142,7> l1_lock_time;
//};

//// Glonass RTK Extended RTK L1-only
//union RTCM1010
//{
//    RTCM1009x12_HDR hdr;
//    BitField<85,6> sat_id;
//    BitField<91,1> code_ind;
//    BitField<92,5> freq_channel;
//    BitField<97,25> l1_pseudorange;
//    BitField<122,20> l1_phaserange_l1_pseudorange;
//    BitField<142,7> l1_lock_time;
//    BitField<149,7> l1_pseudorange_amb;
//    BitField<156,8> l1_cnr;
//};

//// GLONASS Basic RTK L1/L2
//union RTCM1011
//{
//    RTCM1009x12_HDR hdr;
//    BitField<85,6> sat_id;
//    BitField<91,1> l1_code_ind;
//    BitField<92,5> freq_channel;
//    BitField<97,25> l1_pseudorange;
//    BitField<122,20> l1_phaserange_l1_pseudorange;
//    BitField<142,7> l1_lock_time;
//    BitField<149,2> l2_code_ind;
//    BitField<151,14> l1_l2_pseudorange_diff;
//    BitField<165,20> l1_phaserange_l2_pseudorange;
//    BitField<185,7> l2_lock_time_ind;
//};

//// GLONASS Extended RTK L1/L2
//union RTCM1012
//{
//    RTCM1009x12_HDR hdr;
//    BitField<85,6> sat_id;
//    BitField<91,1> l1_code_ind;
//    BitField<92,5> freq_channel;
//    BitField<97,25> l1_pseudorange;
//    BitField<122,20> l1_phaserange_l1_pseudorange;
//    BitField<142,7> l1_lock_time;
//    BitField<149,7> l1_pseudorange_amb;
//    BitField<156,8> l1_cnr;
//    BitField<164,2> l2_code_ind;
//    BitField<166,14> l1_l2_pseudorange_diff;
//    BitField<180,20> l1_phaserange_l2_pseudorange;
//    BitField<200,7> l2_lock_time_ind;
//    BitField<207,8> l2_cnr;
//};

//// RTCM1013 - System Parameters (not supported)
//// RTCM1014 - Network Auxiliary Station Data Message (not supported)
//// RTCM1015 - GPS Ionospheric Correction Differences (not supported)
//// RTCM1016 - GPS Geometric Correction Differences (not supported)
//// RTCM1017 - GPS Combined Geometric and Ionospheric Correction Differences (not supported)

//// GPS Ephemerides
//// See GPS SPS 2.4.3 for descriptions
//union RTCM1019
//{
//    BitField<36,6> sat_id;
//    BitField<42,10> week;
//    BitField<32,4> sv_acc;
//    BitField<36,2> l2_code;
//    BitField<38,14> idot;
//    BitField<52,8> iode;
//    BitField<60,16> toc;
//    BitField<76,8> af2;
//    BitField<84,16> af1;
//    BitField<100,22> af0;
//    BitField<122,10> iodc;
//    BitField<132,16> crs;
//    BitField<158,16> delta_n;
//    BitField<174,32> M0;
//    BitField<206,16> cuc;
//    BitField<222,32> e;
//    BitField<254,16> cus;
//    BitField<270,32> sqrt_A;
//    BitField<302,16> toe;
//    BitField<318,16> cic;
//    BitField<334,32> Omega0;
//    BitField<366,16> cis;
//    BitField<382,32> i0;
//    BitField<414,16> crc;
//    BitField<432,32> w;
//    BitField<464,24> Omegadot;
//    BitField<488,8> tgd;
//    BitField<496,6> sv_health;
//    BitField<512,1> l2_P;
//    BitField<513,1> GPS_fit;
//};

//// GLONASS Ephemerides
//// See GLONASS ICD Version 5.0 for descriptions
//union RTCM1020
//{
//    BitField<36,6> sat_id;
//    BitField<42,5> freq_chan;
//    BitField<47,1> alm_health;
//    BitField<48,1> alm_health_avail;
//    BitField<49,2> P1;
//    BitField<51,12> tk;
//    BitField<63,1> Bn_MSB;
//    BitField<64,1> P2;
//    BitField<65,7> tb;
//    BitField<72,24> xdot;
//    BitField<96,27> xt;
//    BitField<123,5> xddot;
//    BitField<128,24> ydot;
//    BitField<152,27> y;
//    BitField<179,5> yddot;
//    BitField<184,24> zdot;
//    BitField<208,27> z;
//    BitField<235,5> zddot;
//    BitField<240,1> P3;
//    BitField<241,11> gm;
//    BitField<252,2> MP;
//    BitField<244,1> M1;
//    BitField<245,22> tau;
//    BitField<267,5> delta_tau;
//    BitField<273,5> E;
//    BitField<278,1> P4;
//    BitField<279,4> F;
//    BitField<283,11> N;
//    BitField<294,2> M;
//    BitField<296,1> avail;
//    BitField<297,11> NA;
//    BitField<306,32> tau_c;
//    BitField<338,5> N4;
//    BitField<343,22> tau_GPS;
//    BitField<365,1> One_M;
//};
//}

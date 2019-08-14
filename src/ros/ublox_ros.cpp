/* Copyright (c) 2019 James Jackson, Matt Rydalch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <signal.h>
#include <fstream>
#include <iostream>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;

#include <UBLOX/ublox_ros.h>

#define createCallback(cls, type, fun, arg)                                                        \
    do                                                                                             \
    {                                                                                              \
        auto trampoline = [this](uint8_t _class, uint8_t _type, const ublox::UBX_message_t& msg) { \
            this->fun(msg.arg);                                                                    \
        };                                                                                         \
        ublox_->registerUBXCallback(cls, type, trampoline);                                        \
    } while (0)

constexpr double deg2rad(double x)
{
    return M_PI / 180.0 * x;
}

namespace ublox_ros
{
UBLOX_ROS::UBLOX_ROS() : nh_(), nh_private_("~")
{
    int rtk_type = nh_private_.param<int>("rtk_type", 0x00);
    std::string serial_port = nh_private_.param<std::string>("serial_port", "/dev/ttyACM0");
    std::string local_host = nh_private_.param<std::string>("local_host", "localhost");
    int local_port = nh_private_.param<int>("local_port", 16140);
    std::string remote_host = nh_private_.param<std::string>("remote_host", "localhost");
    int remote_port = nh_private_.param<int>("remote_port", 16145);
    std::string log_filename = nh_private_.param<std::string>("log_filename", "");

    // Connect ROS topics
    pvt_pub_ = nh_.advertise<ublox::PositionVelocityTime>("PosVelTime", 10);
    ecef_pub_ = nh_.advertise<ublox::PosVelEcef>("PosVelEcef", 10);
    eph_pub_ = nh_.advertise<ublox::Ephemeris>("Ephemeris", 10);
    geph_pub_ = nh_.advertise<ublox::GlonassEphemeris>("GlonassEphemeris", 10);
    obs_pub_ = nh_.advertise<ublox::ObsVec>("Obs", 10);

    //    nav_sat_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("NavSatFix");
    //    nav_sat_status_pub_ = nh_.advertise<sensor_msgs::NavSatStatus>("NavSatStatus");

    // Open the serial port
    ac_adapter::Serial ser(serial_port, 921600);
    ser.init();

    // create the parser
    ublox_ = std::make_unique<ublox::UBLOX>(ser);

    if (rtk_type & ublox::UBLOX::RTK)
    {
        // Set up UDP Connection
        udp_ = std::make_unique<ac_adapter::UDP>(local_host, local_port, remote_host, remote_port);
        if (!udp_->init())
            ROS_FATAL("Failed to initialize UDP");

        // set up RTK
        if (rtk_type == ublox::UBLOX::ROVER)
            ublox_->config_rover(udp_.get());
        else if (rtk_type == ublox::UBLOX::BASE)
            ublox_->config_base(udp_.get(), ublox::UBLOX::STATIONARY);
    }

    // subscribe to required UBX messages
    subscribe(ublox::CLASS_NAV, ublox::NAV_PVT);
    subscribe(ublox::CLASS_NAV, ublox::NAV_POSECEF);
    subscribe(ublox::CLASS_NAV, ublox::NAV_VELECEF);
    subscribe(ublox::CLASS_RXM, ublox::RXM_RAWX);

    if (rtk_type == ublox::UBLOX::BASE)
    {
        survey_status_pub_ = nh_.advertise<ublox::SurveyStatus>("SurveyStatus", 10);
        subscribe(ublox::CLASS_NAV, ublox::NAV_SVIN);
    }

    if (rtk_type == ublox::UBLOX::ROVER)
    {
        relpos_pub_ = nh_.advertise<ublox::RelPos>("RelPos", 10);
        subscribe(ublox::CLASS_NAV, ublox::NAV_RELPOSNED);
    }
}

void UBLOX_ROS::got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t& msg)
{
    switch (cls)
    {
    case ublox::CLASS_NAV:
        switch (id)
        {
        case ublox::NAV_PVT:
            pvtCB(msg.NAV_PVT);
            break;
        case ublox::NAV_RELPOSNED:
            relposCB(msg.NAV_RELPOSNED);
            break;
        case ublox::NAV_POSECEF:
            posECEFCB(msg.NAV_POSECEF);
            break;
        case ublox::NAV_VELECEF:
            velECEFCB(msg.NAV_VELECEF);
            break;
        case ublox::NAV_SVIN:
            svinCB(msg.NAV_SVIN);
            break;
        }
        break;
    case ublox::CLASS_RXM:
        switch (id)
        {
        case ublox::RXM_RAWX:
            obsCB(msg.RXM_RAWX);
            break;
        }
        break;
    }
}

void UBLOX_ROS::pvtCB(const ublox::NAV_PVT_t& msg)
{
    pos_tow_ = msg.iTOW;
    ublox::PositionVelocityTime out;
    // out.iTOW = msg.iTow;
    out.header.stamp = ros::Time::now();  /// TODO: Do this right
    out.year = msg.year;
    out.month = msg.month;
    out.day = msg.day;
    out.hour = msg.hour;
    out.min = msg.min;
    out.sec = msg.sec;
    out.nano = msg.nano;
    out.tAcc = msg.tAcc;
    out.valid = msg.valid;
    out.fixType = msg.fixType;
    out.flags = msg.flags;
    out.flags2 = msg.flags2;
    out.numSV = msg.numSV;
    out.lla[0] = msg.lat * 1e-7;
    out.lla[1] = msg.lon * 1e-7;
    out.lla[2] = msg.height * 1e-3;
    out.hMSL = msg.hMSL * 1e-3;
    out.hAcc = msg.hAcc * 1e-3;
    out.vAcc = msg.vAcc * 1e-3;
    out.velNED[0] = msg.velN * 1e-3;
    out.velNED[1] = msg.velE * 1e-3;
    out.velNED[2] = msg.velD * 1e-3;
    out.gSpeed = msg.gSpeed * 1e-3;
    out.headMot = msg.headMot * 1e-5;
    out.sAcc = msg.sAcc * 1e-3;
    out.headAcc = msg.headAcc * 1e-5;
    out.pDOP = msg.pDOP * 0.01;
    out.headVeh = msg.headVeh * 1e-5;
    pvt_pub_.publish(out);

    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.lla[0] = out.lla[0];
    ecef_msg_.lla[1] = out.lla[1];
    ecef_msg_.lla[2] = out.lla[2];
    ecef_msg_.horizontal_accuracy = out.hAcc;
    ecef_msg_.vertical_accuracy = out.vAcc;
    ecef_msg_.speed_accuracy = out.sAcc;
    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        ecef_pub_.publish(ecef_msg_);
}

void UBLOX_ROS::relposCB(const ublox::NAV_RELPOSNED_t& msg)
{
    ublox::RelPos out;
    // out.iTOW = msg.iTow*1e-3;
    out.header.stamp = ros::Time::now();  /// TODO: do this right
    out.refStationId = msg.refStationId;
    out.relPosNED[0] = msg.relPosN * 1e-2;
    out.relPosNED[1] = msg.relPosE * 1e-2;
    out.relPosNED[2] = msg.relPosD * 1e-2;
    out.relPosLength = msg.relPosLength * 1e-2;
    out.relPosHeading = deg2rad(msg.relPosHeading * 1e-5);
    out.relPosHPNED[0] = msg.relPosHPN * 1e-3 * .1;
    out.relPosHPNED[1] = msg.relPosHPE * 1e-3 * .1;
    out.relPosHPNED[2] = msg.relPosHPD * 1e-3 * .1;
    out.relPosHPLength = msg.relPosHPLength * 1e-3 * .1;
    out.accNED[0] = msg.accN * 1e-3 * .1;
    out.accNED[1] = msg.accE * 1e-3 * .1;
    out.accNED[2] = msg.accD * 1e-3 * .1;
    out.accLength = msg.accLength * 1e-3 * .1;
    out.accHeading = deg2rad(msg.accHeading * 1e-5);
    out.flags = msg.flags;
    relpos_pub_.publish(out);
}

void UBLOX_ROS::svinCB(const ublox::NAV_SVIN_t& msg)
{
    ublox::SurveyStatus out;
    out.header.stamp = ros::Time::now();  /// TODO: do this right
    out.dur = msg.dur;
    out.meanXYZ[0] = msg.meanX * 1e-2;
    out.meanXYZ[1] = msg.meanY * 1e-2;
    out.meanXYZ[2] = msg.meanZ * 1e-2;
    out.meanXYZHP[0] = msg.meanXHP * 1e-3;
    out.meanXYZHP[1] = msg.meanYHP * 1e-3;
    out.meanXYZHP[2] = msg.meanZHP * 1e-3;
    out.meanAcc = msg.meanAcc;
    out.obs = msg.obs;
    out.valid = msg.valid;
    out.active = msg.active;
    survey_status_pub_.publish(out);
}

void UBLOX_ROS::posECEFCB(const ublox::NAV_POSECEF_t& msg)
{
    pos_tow_ = msg.iTOW;
    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.position[0] = msg.ecefX;
    ecef_msg_.position[1] = msg.ecefY;
    ecef_msg_.position[2] = msg.ecefZ;
    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        ecef_pub_.publish(ecef_msg_);
    ecef_pub_.publish(ecef_msg_);
}

void UBLOX_ROS::velECEFCB(const ublox::NAV_VELECEF_t& msg)
{
    vel_tow_ = msg.iTOW;
    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.velocity[0] = msg.ecefVX;
    ecef_msg_.velocity[0] = msg.ecefVY;
    ecef_msg_.velocity[0] = msg.ecefVZ;

    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        ecef_pub_.publish(ecef_msg_);
    ecef_pub_.publish(ecef_msg_);
}

void UBLOX_ROS::obsCB(const ublox::RXM_RAWX_t& msg)
{
    ublox::ObsVec out;
    UTCTime utc = UTCTime::fromGPS(msg.week, msg.rcvTow * 1e3);
    out.header.stamp.sec = utc.sec;
    out.header.stamp.nsec = utc.nsec;
    for (int i = 0; i < msg.numMeas; i++)
    {
        out.obs[i].sat = msg.meas[i].svId;
        out.obs[i].gnssID = msg.meas[i].gnssId;
        out.obs[i].signal = ublox::sigId(msg.meas[i].gnssId, msg.meas[i].sigId);
        switch (out.obs[i].signal)
        {
        case ublox::Observation::GPS_L1_CA:
        case ublox::Observation::GALILEO_E1_B:
        case ublox::Observation::GALILEO_E1_C:
        case ublox::Observation::QZSS_L1_CA:
            out.obs[i].freq = Ephemeris::GPS_FREQL1;
            break;
        case ublox::Observation::GPS_L2_CL:
        case ublox::Observation::GPS_L2_CM:
            out.obs[i].freq = Ephemeris::GPS_FREQL2;
            break;
        case ublox::Observation::GLONASS_L1:
            out.obs[i].freq =
                GlonassEphemeris::FREQ1_GLO + msg.meas[i].freqId * GlonassEphemeris::DFRQ1_GLO;
            break;
        case ublox::Observation::GLONASS_L2:
            out.obs[i].freq =
                GlonassEphemeris::FREQ2_GLO + msg.meas[i].freqId * GlonassEphemeris::DFRQ2_GLO;
            break;
            // These may not be right
            //        case ublox::Observation::GALILEO_E5_BI:
            //        case ublox::Observation::GALILEO_E5_BQ:
            //            out.obs[i].freq = Ephemeris::GALILEO_FREQL5b;
            //            break;
            //        case ublox::Observation::BEIDOU_B1I_D1:
            //        case ublox::Observation::BEIDOU_B1I_D2:
            //            out.obs[i].freq = Ephemeris::BEIDOU_FREQ_B1;
            //            break;
            //        case ublox::Observation::BEIDOU_B2I_D1:
            //        case ublox::Observation::BEIDOU_B2I_D2:
            //            out.obs[i].freq = Ephemeris::BEIDOU_FREQ_B2;
            //            break;
        default:
            out.obs[i].freq = 0;
            break;
        }
        out.obs[i].cno = msg.meas[i].cno;
        out.obs[i].locktime = msg.meas[i].locktime;
        out.obs[i].P = msg.meas[i].prMeas;
        out.obs[i].L = msg.meas[i].cpMeas;
        out.obs[i].D = msg.meas[i].doMeas;
        out.obs[i].stdevP = 0.01 * pow(2, msg.meas[i].prStdev);
        out.obs[i].stdevL = 0.004 * msg.meas[i].cpStdev;
        out.obs[i].stdevD = 0.002 * pow(2, msg.meas[i].doStdev);

        // indicate cycle slip
        if (msg.meas[i].cpMeas != 0.0 && (msg.meas[i].trkStat & ublox::RXM_RAWX_t::trkStat_HalfCyc |
                                          ublox::RXM_RAWX_t::trkStat_subHalfCyc))
        {
            out.obs[i].LLI = ublox::Observation::LLI_HALF_CYCLE_AMB;
        }
        else
        {
            out.obs[i].LLI = 0;
        }
    }
    obs_pub_.publish(out);
}

void UBLOX_ROS::ephCB(const Ephemeris& eph)
{
    std::cerr << "Ephemeris!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    ublox::Ephemeris out;
    out.header.stamp = ros::Time::now();

    out.sat = eph.sat;
    out.gnssID = eph.gnssID;
    out.toe.sec = eph.toe.sec;
    out.toe.nsec = eph.toe.nsec;
    out.toc.sec = eph.toc.sec;
    out.toc.nsec = eph.toc.nsec;

    out.tow = eph.tow;
    out.iodc = eph.iodc;
    out.iode = eph.iode;
    out.week = eph.week;
    out.toes = eph.toes;
    out.tocs = eph.tocs;
    out.health = eph.health;
    out.alert_flag = eph.alert_flag;
    out.anti_spoof = eph.anti_spoof;
    out.code_on_L2 = eph.code_on_L2;
    out.ura = eph.ura;
    out.L2_P_data_flag = eph.L2_P_data_flag;
    out.fit_interval_flag = eph.fit_interval_flag;
    out.age_of_data_offset = eph.age_of_data_offset;
    out.tgd[0] = eph.tgd[0];
    out.tgd[1] = eph.tgd[1];
    out.tgd[2] = eph.tgd[2];
    out.tgd[3] = eph.tgd[3];
    out.af2 = eph.af2;
    out.af1 = eph.af1;
    out.af0 = eph.af0;
    out.m0 = eph.m0;
    out.delta_n = eph.delta_n;
    out.ecc = eph.ecc;
    out.sqrta = eph.sqrta;
    out.omega0 = eph.omega0;
    out.i0 = eph.i0;
    out.w = eph.w;
    out.omegadot = eph.omegadot;
    out.idot = eph.idot;
    out.cuc = eph.cuc;
    out.cus = eph.cus;
    out.crc = eph.crc;
    out.crs = eph.crs;
    out.cic = eph.cic;
    out.cis = eph.cis;

    eph_pub_.publish(out);
}

void UBLOX_ROS::gephCB(const GlonassEphemeris& eph)
{
    std::cerr << "Glonas Ephemeris!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    ublox::GlonassEphemeris out;
    out.header.stamp = ros::Time::now();

    out.sat = eph.sat;
    out.gnssID = eph.gnssID;

    out.toe.sec = eph.toe.sec;
    out.toe.nsec = eph.toe.nsec;
    out.tof.sec = eph.tof.sec;
    out.tof.nsec = eph.tof.nsec;

    out.iode = eph.iode;
    out.frq = eph.frq;
    out.svh = eph.svh;
    out.sva = eph.sva;
    out.age = eph.age;
    out.pos[0] = eph.pos[0];
    out.pos[1] = eph.pos[1];
    out.pos[2] = eph.pos[2];
    out.vel[0] = eph.vel[0];
    out.vel[1] = eph.vel[1];
    out.vel[2] = eph.vel[2];
    out.acc[0] = eph.acc[0];
    out.acc[1] = eph.acc[1];
    out.acc[2] = eph.acc[2];
    out.taun = eph.taun;
    out.gamn = eph.gamn;
    out.dtaun = eph.dtaun;

    geph_pub_.publish(out);
}

}  // namespace ublox_ros

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ublox_ros");

    ublox_ros::UBLOX_ROS Thing;

    ros::spin();
}

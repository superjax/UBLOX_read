#include <iostream>
#include <fstream>
#include <signal.h>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;


#include <UBLOX/ublox_ros.h>

#define createCallback(cls, type, fun, arg)\
do{\
    auto trampoline = [this](uint8_t _class, uint8_t _type, const UBX::UBX_message_t& msg)\
    {\
        this->fun(msg.arg);\
    };\
    ublox_->registerUBXCallback(cls, type, trampoline);\
}while(0)

constexpr double deg2rad(double x) { return M_PI/180.0 * x; }

namespace ublox_ros
{

UBLOX_ROS::UBLOX_ROS() :
    nh_(), nh_private_("~")
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
    relpos_pub_ = nh_.advertise<ublox::RelPos>("RelPos", 10);
    gnss_pub_ = nh_.advertise<ublox::GNSS>("ECEF", 10);
    svin_pub_ = nh_.advertise<ublox::SVIN>("SVIN", 10);
//    nav_sat_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("NavSatFix");
//    nav_sat_status_pub_ = nh_.advertise<sensor_msgs::NavSatStatus>("NavSatStatus");

    // create the parser
    ublox_ = new UBLOX(serial_port);

    if (!log_filename.empty())
        ublox_->initLogFile(log_filename);

    // set up RTK
    if (rtk_type == UBLOX::ROVER)
        ublox_->initRover(local_host, local_port, remote_host, remote_port);
    else if (rtk_type == UBLOX::BASE)
        ublox_->initBase(local_host, local_port, remote_host, remote_port);

    // connect callbacks
    createCallback(UBX::CLASS_NAV, UBX::NAV_PVT, pvtCB, NAV_PVT);
    createCallback(UBX::CLASS_NAV, UBX::NAV_RELPOSNED, relposCB, NAV_RELPOSNED);
    createCallback(UBX::CLASS_NAV, UBX::NAV_SVIN, svinCB, NAV_SVIN);
    createCallback(UBX::CLASS_NAV, UBX::NAV_POSECEF, posECEFCB, NAV_POSECEF);
    createCallback(UBX::CLASS_NAV, UBX::NAV_VELECEF, velECEFCB, NAV_VELECEF);
}

UBLOX_ROS::~UBLOX_ROS()
{
    if (ublox_)
        delete ublox_;
}

void UBLOX_ROS::pvtCB(const UBX::NAV_PVT_t& msg)
{
    pos_tow_ = msg.iTOW;
    ublox::PositionVelocityTime out;
    out.header.stamp = ros::Time::now(); ///TODO: Do this right
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
    out.lla[0] = msg.lat*1e-7;
    out.lla[1] = msg.lon*1e-7;
    out.lla[2] = msg.height*1e-3;
    out.hMSL = msg.hMSL*1e-3;
    out.hAcc = msg.hAcc*1e-3;
    out.vAcc = msg.vAcc*1e-3;
    out.velNED[0] = msg.velN*1e-3;
    out.velNED[1] = msg.velE*1e-3;
    out.velNED[2] = msg.velD*1e-3;
    out.gSpeed = msg.gSpeed*1e-3;
    out.headMot = msg.headMot*1e-5;
    out.sAcc = msg.sAcc*1e-3;
    out.headAcc = msg.headAcc*1e-5;
    out.pDOP = msg.pDOP*0.01;
    out.headVeh = msg.headVeh*1e-5;
    pvt_pub_.publish(out);

    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.lla[0] = out.lla[0];
    ecef_msg_.lla[1] = out.lla[1];
    ecef_msg_.lla[2] = out.lla[2];
    ecef_msg_.horizontal_accuracy = out.hAcc;
    ecef_msg_.vertical_accuracy = out.vAcc;
    ecef_msg_.speed_accuracy = out.sAcc;
    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        gnss_pub_.publish(ecef_msg_);
}


void UBLOX_ROS::relposCB(const UBX::NAV_RELPOSNED_t& msg)
{
    ublox::RelPos out;
    out.header.stamp = ros::Time::now(); /// TODO: do this right
    out.refStationId = msg.refStationId;
    out.relPosNED[0] = msg.relPosN*1e-2;
    out.relPosNED[1] = msg.relPosE*1e-2;
    out.relPosNED[2] = msg.relPosD*1e-2;
    out.relPosLength = msg.relPosLength*1e-2;
    out.relPosHeading = deg2rad(msg.relPosHeading*1e-5);
    out.accNED[0] = msg.accN;
    out.accNED[1] = msg.accE;
    out.accNED[2] = msg.accD;
    out.accLength = msg.accLength*1e-3;
    out.accHeading = deg2rad(msg.accHeading*1e-5);
    out.flags = msg.flags;

    // ofstream myfile;
    // myfile.open ("../ws/src/ublox/textfiles/10 8ft Pivot/data.txt", ios::app);
    // myfile << out.header.stamp
    //        << " " << out.relPosNED[0] << " " << out.relPosNED[1] << " " << out.relPosNED[2]
    //        << " " << out.relPosLength << " " << out.flags << " \n";
    // myfile.close();

    relpos_pub_.publish(out);
}

void UBLOX_ROS::svinCB(const UBX::NAV_SVIN_t& msg)
{
    ublox::SVIN out;
    out.header.stamp = ros::Time::now(); /// TODO: do this right
    out.dur = msg.dur;
    out.meanXYZ[0] = msg.meanX*1e-2;
    out.meanXYZ[1] = msg.meanY*1e-2;
    out.meanXYZ[2] = msg.meanZ*1e-2;
    out.meanXYZHP[0] = msg.meanXHP*1e-3;
    out.meanXYZHP[1] = msg.meanYHP*1e-3;
    out.meanXYZHP[2] = msg.meanZHP*1e-3;
    out.meanAcc = msg.meanAcc;
    out.obs = msg.obs;
    out.valid = msg.valid;
    out.active = msg.active;

    svin_pub_.publish(out);

}

void UBLOX_ROS::posECEFCB(const UBX::NAV_POSECEF_t& msg)
{
    pos_tow_ = msg.iTOW;
    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.position[0] = msg.ecefX;
    ecef_msg_.position[1] = msg.ecefY;
    ecef_msg_.position[2] = msg.ecefZ;
    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        gnss_pub_.publish(ecef_msg_);
}

void UBLOX_ROS::velECEFCB(const UBX::NAV_VELECEF_t& msg)
{
    vel_tow_ = msg.iTOW;
    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.velocity[0] = msg.ecefVX;
    ecef_msg_.velocity[0] = msg.ecefVY;
    ecef_msg_.velocity[0] = msg.ecefVZ;

    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
        gnss_pub_.publish(ecef_msg_);
}

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ublox_ros");

    ublox_ros::UBLOX_ROS Thing;

    ros::spin();
}


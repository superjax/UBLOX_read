#ifndef UBLOX_ROS_H
#define UBLOX_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "UBLOX/ublox.h"

#include "ublox/GNSS.h"
#include "ublox/PositionVelocityTime.h"
#include "ublox/RelPos.h"
#include "ublox/SVIN.h"

namespace ublox_ros
{

class UBLOX_ROS
{
public:
    UBLOX_ROS();
    ~UBLOX_ROS();

private:
    ublox::UBLOX* ublox_ = nullptr;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pvt_pub_;
    ros::Publisher svin_pub_;
    ros::Publisher relpos_pub_;
    ros::Publisher gnss_pub_;
    ros::Publisher nav_sat_fix_pub_;
    ros::Publisher nav_sat_status_pub_;

    void pvtCB(const ublox::NAV_PVT_t& msg);
    void relposCB(const ublox::NAV_RELPOSNED_t& msg);
    void posECEFCB(const ublox::NAV_POSECEF_t& msg);
    void velECEFCB(const ublox::NAV_VELECEF_t& msg);
    void svinCB(const ublox::NAV_SVIN_t& msg);

    uint32_t pos_tow_;
    uint32_t vel_tow_;
    uint32_t pvt_tow_;
    ublox::GNSS ecef_msg_;
};

}










#endif

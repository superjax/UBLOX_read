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

#ifndef UBLOX_ROS_H
#define UBLOX_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <memory>

#include "UBLOX/ublox.h"

#include "ublox/Ephemeris.h"
#include "ublox/GlonassEphemeris.h"
#include "ublox/ObsVec.h"
#include "ublox/Observation.h"
#include "ublox/PosVelEcef.h"
#include "ublox/PositionVelocityTime.h"
#include "ublox/RelPos.h"
#include "ublox/SurveyStatus.h"

#include "UBLOX/async_comm_adapter.h"

namespace ublox_ros
{
class UBLOX_ROS : public ublox::UBXListener
{
public:
    UBLOX_ROS();
    ~UBLOX_ROS() = default;

private:
    std::unique_ptr<ublox::UBLOX> ublox_ = nullptr;
    std::unique_ptr<ac_adapter::UDP> udp_ = nullptr;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pvt_pub_;
    ros::Publisher survey_status_pub_;
    ros::Publisher relpos_pub_;
    ros::Publisher ecef_pub_;
    ros::Publisher nav_sat_fix_pub_;
    ros::Publisher nav_sat_status_pub_;
    ros::Publisher eph_pub_;
    ros::Publisher geph_pub_;
    ros::Publisher obs_pub_;

    void got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t& msg);
    void pvtCB(const ublox::NAV_PVT_t& msg);
    void relposCB(const ublox::NAV_RELPOSNED_t& msg);
    void posECEFCB(const ublox::NAV_POSECEF_t& msg);
    void velECEFCB(const ublox::NAV_VELECEF_t& msg);
    void svinCB(const ublox::NAV_SVIN_t& msg);

    void obsCB(const ublox::RXM_RAWX_t& msg);
    void ephCB(const Ephemeris& eph);
    void gephCB(const GlonassEphemeris& eph);

    uint32_t pos_tow_;
    uint32_t vel_tow_;
    uint32_t pvt_tow_;
    uint32_t pvt_week_;
    ublox::PosVelEcef ecef_msg_;
};

}  // namespace ublox_ros

#endif

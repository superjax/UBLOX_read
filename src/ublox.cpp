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

#include "UBLOX/ublox.h"
#define DBG(...) fprintf(stderr, __VA_ARGS__)

namespace ublox
{
constexpr int UBLOX::MAX_NUM_TRIES;

UBLOX::UBLOX(SerialInterface &ser) : ser_(ser), ubx_(ser)
{
    ser_.add_listener(this);
    ubx_.registerListener(&nav_);

    type_ = NONE;

    //  We have to make sure we get the version before we do anything else
    int num_tries = 0;
    while (++num_tries < MAX_NUM_TRIES && !ubx_.get_version())
    {
        printf("Unable to query version.  Trying again (%d out of %d)\n", num_tries + 1,
               MAX_NUM_TRIES);
    }

    if (num_tries == MAX_NUM_TRIES)
    {
        throw std::runtime_error("Unable to connect.  Please check your serial port configuration");
    }

    // configure the parsers/Enable Messages
    ubx_.disable_nmea();
    ubx_.set_nav_rate(200);

    ubx_.enable_message(CLASS_NAV, NAV_PVT, 1);
    ubx_.enable_message(CLASS_NAV, NAV_RELPOSNED, 1);
    ubx_.enable_message(CLASS_NAV, NAV_VELECEF, 1);
    ubx_.enable_message(CLASS_RXM, RXM_RAWX, 1);
    ubx_.enable_message(CLASS_RXM, RXM_SFRBX, 1);
}

void UBLOX::config_base(SerialInterface *interface, const int type)
{
    assert(type == MOVING || type == STATIONARY);

    type_ = BASE;
    rtk_interface_ = interface;

    rtcm_.registerListener(this);

    // Request RTCM messages
    using CV = CFG_VALSET_t;
    ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::RTCM_4072_0USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::RTCM_4072_1USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::RTCM_1077USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::RTCM_1087USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::RTCM_1097USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::RTCM_1127USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::RTCM_1230USB, 1);

    if (type == STATIONARY)
    {
        ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::MSGOUT_SVIN, 1);
        ubx_.configure(CV::VERSION_0, CV::RAM, 1, CV::TMODE_MODE, 1);
        ubx_.configure(CV::VERSION_0, CV::RAM, 500000, CV::TMODE_SVIN_ACC_LIMIT, 2);
        ubx_.configure(CV::VERSION_0, CV::RAM, 119, CV::TMODE_SVIN_MIN_DUR, 2);
        ubx_.start_survey_in();
    }
}

void UBLOX::config_rover(SerialInterface *interface)
{
    type_ = ROVER;

    // Disable the output of extra RTCM messages
    using CV = CFG_VALSET_t;
    ubx_.configure(CV::VERSION_0, CV::RAM, 0, CV::RTCM_4072_0USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 0, CV::RTCM_4072_1USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 0, CV::RTCM_1077USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 0, CV::RTCM_1087USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 0, CV::RTCM_1097USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 0, CV::RTCM_1127USB, 1);
    ubx_.configure(CV::VERSION_0, CV::RAM, 0, CV::RTCM_1230USB, 1);

    static RTCM debug;

    // forward any information over the port
    rtk_interface_ = interface;
    interface->add_callback([this](const uint8_t *buf, size_t size) {
        this->ser_.write(buf, size);

        debug.read_cb(buf, size);
    });
}

UBLOX::~UBLOX()
{
    if (log_file_.is_open())
        log_file_.close();
}

void UBLOX::read_cb(const uint8_t *buf, size_t size)
{
    for (int i = 0; i < size; i++)
    {
        /// TODO: don't give parsers data they don't need
        if (ubx_.parsing_message())
        {
            ubx_.read_cb(buf[i]);
        }
        else if (rtcm_.parsing_message() && type_ == BASE)
        {
            rtcm_.read_cb(buf + i, 1);
        }
        else
        {
            ubx_.read_cb(buf[i]);
            rtcm_.read_cb(buf + i, 1);
        }
    }
}

void UBLOX::got_rtcm(const uint8_t *buf, const size_t size)
{
    // If we are a base, forward this RTCM message out over our interface
    if (type_ == BASE)
    {
        rtk_interface_->write(buf, size);
    }
}

}  // namespace ublox

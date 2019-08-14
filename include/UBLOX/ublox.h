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

#ifndef UBLOX_H
#define UBLOX_H

#include <stdint.h>
#include <fstream>
#include <functional>
#include <iostream>

#include "UBLOX/parsers/nav.h"
#include "UBLOX/parsers/rtcm.h"
#include "UBLOX/parsers/ubx.h"
#include "UBLOX/serial_interface.h"

namespace ublox
{
class UBLOX : public SerialListener, public RTCMListener
{
    static constexpr int MAX_NUM_TRIES = 3;

public:
    typedef enum
    {
        NONE = 0,
        ROVER = 0b10,
        BASE = 0b11,
        RTK = 0b10,
    } rtk_type_t;

    enum
    {
        MOVING,
        STATIONARY
    };

    UBLOX(SerialInterface& ser);
    ~UBLOX();

    void config_rover(SerialInterface* interface);
    void config_base(SerialInterface* interface, const int type = STATIONARY);

    // UBLOX receiver read/write
    void read_cb(const uint8_t* buf, const size_t size) override;

    inline void registerUBXListener(UBXListener* l) { ubx_.registerListener(l); }
    inline void registerEphCallback(const NavParser::eph_cb& cb) { nav_.registerCallback(cb); }
    inline void registerGephCallback(const NavParser::geph_cb& cb) { nav_.registerCallback(cb); }

    void got_rtcm(const uint8_t* buf, const size_t size) override;

private:
    void poll_value();

    SerialInterface& ser_;
    SerialInterface* rtk_interface_ = nullptr;

    UBX ubx_;
    RTCM rtcm_;
    NavParser nav_;
    rtk_type_t type_;
    std::ofstream log_file_;
};
}  // namespace ublox

#endif

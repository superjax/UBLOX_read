/* Copyright (c) 2019 James Jackson
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

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <ostream>

class UTCTime
{
public:
    int64_t sec;  // time since Jan 1 1970 (with leap-seconds)
    int64_t nsec;

    static constexpr uint64_t E9 = 1000000000;
    static constexpr uint64_t E6 = 1000000;

    static constexpr int SEC_IN_WEEK = 7 * 24 * 60 * 60;  // 604800
    static constexpr int SEC_IN_DAY = 24 * 60 * 60;       // 86400

    static constexpr int64_t LEAP_SECONDS = 18;   // GPS time does not have leap seconds, UNIX does
                                                  // (as of 1/1/2017 - next one is probably in 2020
                                                  // sometime unless there is some crazy earthquake
                                                  // or nuclear blast)
    static constexpr int GPS_BEIDOU_OFFSET = 14;  // Beidou is 14 seconds ahead of GPS, because it's
                                                  // synced with UTC in 2006, when UTC had 14 leap
                                                  // seconds wrt GPS
    static constexpr int GPS_WEEK_ROLLOVER = 2048;  // 10-bit week counter only goes to 1024.  As of
                                                    // Jul 2019, this has rolled over twice
                                                    // TODO: Make this automatic

    // T_UTC = T_G + G_UTC_OFFSET
    // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between
    // those in seconds
    static constexpr int64_t GPS_UTC_OFFSET = 315964800 + LEAP_SECONDS;

    // GLONASS is exactly 3 hours ahead of UTC (including leap seconds)
    static constexpr int64_t GLO_UTC_OFFSET = -3 * 3600;
    static constexpr int BD_UTC_OFFSET = GPS_UTC_OFFSET + GPS_BEIDOU_OFFSET;

    UTCTime();

    UTCTime(int _sec, int _nsec);

    UTCTime(double _sec);

    bool operator>(const UTCTime& other) const;
    bool operator<(const UTCTime& other) const;
    bool operator==(const UTCTime& other) const;

    UTCTime operator-(const UTCTime& other) const;
    UTCTime& operator-=(const UTCTime& other);
    UTCTime operator+(const UTCTime& other) const;
    UTCTime& operator+=(const UTCTime& other);

    UTCTime operator+(double sec_) const;
    UTCTime& operator+=(double sec_);
    UTCTime operator+(int sec_) const;
    UTCTime& operator+=(int sec_);

    UTCTime operator-(double sec_) const;
    UTCTime& operator-=(double sec_);
    UTCTime operator-(int sec_) const;
    UTCTime& operator-=(int sec_);

    double toSec();
    void wrapNsec();

    static UTCTime now();

    static UTCTime fromGPS(int week, int tow_ms);
    static UTCTime fromGlonass(int week, int tow_ms);

    // This function takes a reference time (in UTC) and computes
    // the closest UTC Time using the supplied tod_ms in Glonass time
    // (This is used to convert the t_k and t_b fields of the GLONASS ephemeris
    //  that come without a week count)
    static UTCTime fromGlonassTimeOfDay(const UTCTime& ref, int tod_ms);

    static UTCTime fromGalileo(int week, int tow_ms);
    //    static UTCTime fromBeidou(int week, int tow_ms);

    int week() const;
    int GpsWeek() const;
    int GlonassWeek() const;
    int GlonassDayOfWeek() const;
    //    int BeidouWeek();
};

std::ostream& operator<<(std::ostream& os, const UTCTime& t);

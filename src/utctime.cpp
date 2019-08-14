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

#include "UBLOX/utctime.h"

constexpr uint64_t UTCTime::E9;
constexpr uint64_t UTCTime::E6;
constexpr int UTCTime::SEC_IN_WEEK;
constexpr int UTCTime::SEC_IN_DAY;
constexpr int64_t UTCTime::LEAP_SECONDS;
constexpr int UTCTime::GPS_BEIDOU_OFFSET;
constexpr int UTCTime::GPS_WEEK_ROLLOVER;
constexpr int64_t UTCTime::GPS_UTC_OFFSET;
constexpr int64_t UTCTime::GLO_UTC_OFFSET;
constexpr int UTCTime::BD_UTC_OFFSET;

UTCTime::UTCTime()
{
    sec = 0;
    nsec = 0;
}

UTCTime::UTCTime(int _sec, int _nsec) : sec(_sec), nsec(_nsec)
{
    wrapNsec();
}

UTCTime::UTCTime(double _sec)
{
    sec = std::floor(_sec);
    nsec = (uint64_t)(_sec * E9) % E9;
    wrapNsec();
}

bool UTCTime::operator>(const UTCTime& other) const
{
    return (sec > other.sec) ? true
                             : (sec < other.sec) ? false : (nsec > other.nsec) ? true : false;
}
bool UTCTime::operator<(const UTCTime& other) const
{
    return (sec < other.sec) ? true
                             : (sec > other.sec) ? false : (nsec < other.nsec) ? true : false;
}

bool UTCTime::operator==(const UTCTime& other) const
{
    return (sec == other.sec && nsec == other.nsec);
}

UTCTime UTCTime::operator-(const UTCTime& other) const
{
    UTCTime out;
    out.sec = sec - other.sec;
    out.nsec = nsec - other.nsec;
    out.wrapNsec();
    return out;
}

UTCTime& UTCTime::operator-=(const UTCTime& other)
{
    sec = sec - other.sec;
    nsec = nsec - other.nsec;
    wrapNsec();
    return (*this);
}

UTCTime UTCTime::operator+(const UTCTime& other) const
{
    UTCTime out;
    out.sec = sec + other.sec;
    out.nsec = nsec + other.nsec;
    out.wrapNsec();
    return out;
}

UTCTime& UTCTime::operator+=(const UTCTime& other)
{
    sec = sec + other.sec;
    nsec = nsec + other.nsec;
    wrapNsec();
    return (*this);
}

double UTCTime::toSec()
{
    return (double)sec + (double)nsec * 1e-9;
}

UTCTime UTCTime::operator+(double sec_) const
{
    UTCTime out;
    out.nsec = nsec + (uint64_t)(sec_ * E9) % E9;
    out.sec = sec + std::floor(sec_);
    out.wrapNsec();
    return out;
}
UTCTime& UTCTime::operator+=(double sec_)
{
    nsec = nsec + (uint64_t)(sec_ * E9) % E9;
    sec = sec + std::floor(sec_);
    wrapNsec();
    return *this;
}
UTCTime UTCTime::operator+(int sec_) const
{
    UTCTime out;
    out.sec = sec + std::floor(sec_);
    return out;
}
UTCTime& UTCTime::operator+=(int sec_)
{
    UTCTime out;
    sec = sec + std::floor(sec_);
    return *this;
}

UTCTime UTCTime::operator-(double sec_) const
{
    UTCTime out;
    out.nsec = nsec - (uint64_t)(sec_ * E9) % E9;
    out.sec = sec - std::floor(sec_);
    out.wrapNsec();
    return out;
}
UTCTime& UTCTime::operator-=(double sec_)
{
    nsec = nsec - (uint64_t)(sec_ * E9) % E9;
    sec = sec - std::floor(sec_);
    wrapNsec();
    return *this;
}
UTCTime UTCTime::operator-(int sec_) const
{
    UTCTime out;
    out.sec = sec - sec_;
    return out;
}
UTCTime& UTCTime::operator-=(int sec_)
{
    UTCTime out;
    sec = sec - sec_;
    return *this;
}

int UTCTime::week() const
{
    return std::floor(sec / SEC_IN_WEEK);
}

int UTCTime::GpsWeek() const
{
    int64_t gps_sec = sec - GPS_UTC_OFFSET;
    return std::floor(gps_sec / SEC_IN_WEEK);
}

// int UTCTime::BeidouWeek()
//{
//    int64_t gps_sec = sec - GPS_UTC_OFFSET + GPS_BEIDOU_OFFSET;
//    return std::floor(gps_sec/SEC_IN_WEEK);
//}

int UTCTime::GlonassWeek() const
{
    int64_t glonass_sec = sec - GLO_UTC_OFFSET;
    return std::floor(glonass_sec / SEC_IN_WEEK);
}

int UTCTime::GlonassDayOfWeek() const
{
    int64_t glonass_sec = sec - GLO_UTC_OFFSET;
    int32_t sec_of_week = glonass_sec % SEC_IN_WEEK;
    int32_t day_of_week = std::floor(sec_of_week / SEC_IN_DAY);
    return day_of_week;
}

UTCTime UTCTime::now()
{
    timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    UTCTime out;
    out.sec = start.tv_sec;
    out.nsec = start.tv_nsec;
    return out;
}

UTCTime UTCTime::fromGPS(int week, int tow_ms)
{
    UTCTime out;
    out.sec = week * SEC_IN_WEEK + tow_ms / 1000 + GPS_UTC_OFFSET;
    out.nsec = (tow_ms % 1000) * E9;
    out.wrapNsec();
    return out;
}

UTCTime UTCTime::fromGlonassTimeOfDay(const UTCTime& ref, int tod_ms)
{
    uint32_t glonass_week = ref.GlonassWeek();
    uint32_t glonass_day = ref.GlonassDayOfWeek();

    int glonass_tow_ms = (glonass_day * SEC_IN_DAY) * 1000 + tod_ms;

    UTCTime out = UTCTime::fromGlonass(glonass_week, glonass_tow_ms);
    return out;
}

UTCTime UTCTime::fromGalileo(int week, int tow_ms)
{
    // Galileo and GPS use the same time system
    return fromGPS(week, tow_ms);
}

// UTCTime UTCTime::fromBeidou(int week, int tow_ms)
//{

//    int64_t new_tow = tow_ms - GPS_BEIDOU_OFFSET*1000;
//    if (new_tow < 0)
//    {
//        week -= SEC_IN_WEEK;
//        tow_ms += SEC_IN_WEEK;
//    }
//    return fromGPS(week, tow_ms);
//}

UTCTime UTCTime::fromGlonass(int week, int tow_ms)
{
    UTCTime out;
    out.sec = int64_t(week * SEC_IN_WEEK) + tow_ms / 1000 + GLO_UTC_OFFSET;
    out.nsec = (tow_ms % 1000) * E9;
    out.wrapNsec();
    return out;
}

void UTCTime::wrapNsec()
{
    if (nsec < 0)
    {
        sec -= 1;
        nsec += E9;
    }
    else if (nsec > E9)
    {
        sec += 1;
        nsec -= E9;
    }
}

std::ostream& operator<<(std::ostream& os, const UTCTime& t)
{
    double sec = (t.sec % UTCTime::SEC_IN_WEEK) + t.nsec * 1e-9;
    int week = std::floor(t.sec / UTCTime::SEC_IN_WEEK);

    time_t time(t.sec);
    tm* date = gmtime(&time);

    os << 1900 + date->tm_year << "/" << 1 + date->tm_mon << "/" << date->tm_mday << " "
       << date->tm_hour << ":" << date->tm_min << ":" << date->tm_sec;

    return os;
}

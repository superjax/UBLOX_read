#include <ctime>
#include <cstdint>
#include <cmath>


class UTCTime
{
public:
    int64_t sec;  // time since Jan 1 1970
    int64_t nsec;

    static constexpr uint64_t ONE_BILLION = 1000000000;
    static constexpr int SEC_IN_WEEK = 7*24*60*60;
    static constexpr int64_t GPS_UTC_OFFSET = 315964800; // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
    static constexpr int64_t LEAP_SECONDS = 18; // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
    static constexpr int64_t UTC_TO_GPS_OFFSET = (GPS_UTC_OFFSET - LEAP_SECONDS);
    static constexpr int GPS_BEIDOU_OFFSET = 14; // Beidou is 14 seconds ahead of GPS, because it's synced with UTC in 2006, when UTC had 14 leap seconds wrt GPS

    UTCTime()
    {
        sec = 0;
        nsec = 0;
    }

    UTCTime(int _sec, int _nsec) :
        sec(_sec),
        nsec(_nsec)
    {
        wrapNsec();
    }

    UTCTime(double _sec)
    {
        sec = std::floor(_sec);
        nsec = (uint64_t)(_sec*ONE_BILLION) % ONE_BILLION;
        wrapNsec();
    }

    bool operator>(const UTCTime& other) const
    {
        return (sec > other.sec)   ? true  :
               (sec < other.sec)   ? false :
               (nsec > other.nsec) ? true  :
                                     false;
    }
    bool operator<(const UTCTime& other) const
    {
        return (sec < other.sec)   ? true  :
               (sec > other.sec)   ? false :
               (nsec < other.nsec) ? true  :
                                     false;
    }

    UTCTime operator- (const UTCTime& other) const
    {
        UTCTime out;
        out.sec = sec - other.sec;
        out.nsec = nsec - other.nsec;
        out.wrapNsec();
        return out;
    }

    UTCTime operator+ (const UTCTime& other) const
    {
        UTCTime out;
        out.sec = sec + other.sec;
        out.nsec = nsec + other.nsec;
        out.wrapNsec();
        return out;
    }

    double toSec()
    {
        return (double)sec + (double)nsec*1e-9;
    }

    UTCTime operator+ (double sec_) const
    {
        UTCTime out;
        out.nsec = nsec + (uint64_t)(sec_*ONE_BILLION) % ONE_BILLION;
        out.sec = sec + std::floor(sec_);
        out.wrapNsec();
        return out;
    }
    UTCTime operator+= (double sec_)
    {
        nsec = nsec + (uint64_t)(sec_*ONE_BILLION) % ONE_BILLION;
        sec = sec +std::floor(sec_);
        wrapNsec();
        return *this;
    }
    UTCTime operator+ (int sec_) const
    {
        UTCTime out;
        out.sec = sec +std::floor(sec_);
        return out;
    }
    UTCTime operator+= (int sec_)
    {
        UTCTime out;
        sec = sec +std::floor(sec_);
        return *this;
    }

    static UTCTime now()
    {
        timespec start;
        clock_gettime(CLOCK_REALTIME, &start);
        UTCTime out;
        out.sec  = start.tv_sec;
        out.nsec = start.tv_nsec;
        return out;
    }

    static UTCTime fromGlonass(int dow, int tod_ms)
    {
        return now() + 10800.0 + dow + tod_ms*1e-3;

    }

    static UTCTime fromGPS(int week, int tow_ms)
    {

        UTCTime out;
        out.sec = week * SEC_IN_WEEK + tow_ms/1000 + UTC_TO_GPS_OFFSET;
        out.nsec = (tow_ms % 1000)*ONE_BILLION;
        out.wrapNsec();
        return out;
    }

    static UTCTime fromGalileo(int week, int tow_ms)
    {
        // Galileo and GPS use the same time system
        return fromGPS(week, tow_ms);
    }

    static UTCTime fromBeidou(int week, int tow_ms)
    {

        int64_t new_tow = tow_ms - GPS_BEIDOU_OFFSET*1000;
        if (new_tow < 0)
        {
            week -= SEC_IN_WEEK;
            tow_ms += SEC_IN_WEEK;
        }
        return fromGPS(week, tow_ms);
    }

    int week()
    {
        return std::floor(sec/(SEC_IN_WEEK));
    }

    int GpsWeek()
    {
        int64_t gps_sec = sec - GPS_UTC_OFFSET;
        return std::floor(gps_sec/SEC_IN_WEEK);
    }

    int BeidouWeek()
    {
        int64_t gps_sec = sec - GPS_UTC_OFFSET + GPS_BEIDOU_OFFSET;
        return std::floor(gps_sec/SEC_IN_WEEK);
    }

private:
    void wrapNsec()
    {
        if (nsec < 0)
        {
            sec -= 1;
            nsec += ONE_BILLION;
        }
        else if (nsec > ONE_BILLION)
        {
            sec += 1;
            nsec -= ONE_BILLION;
        }
    }





};

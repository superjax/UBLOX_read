#include <gtest/gtest.h>


#include "UBLOX/utctime.h"

TEST (Time, Now)
{
    std::cout << "now: " << UTCTime::now() << std::endl;
}

TEST (Time, CompareSec)
{
    UTCTime t1 = UTCTime(1564005746, 0); // random start time
    UTCTime t2 = UTCTime(1564005748, 0); // random start time

    EXPECT_TRUE(t2 > t1);
    EXPECT_TRUE(t1 < t2);
    EXPECT_FALSE(t2 < t1);
    EXPECT_FALSE(t1 > t2);
    EXPECT_FALSE(t2 == t1);
    EXPECT_FALSE(t1 == t2);
}

TEST (Time, CompareNsec)
{
    UTCTime t1 = UTCTime(1564005746, 1234); // random start time
    UTCTime t2 = UTCTime(1564005746, 1235); // random start time

    EXPECT_TRUE(t2 > t1);
    EXPECT_TRUE(t1 < t2);
    EXPECT_FALSE(t2 < t1);
    EXPECT_FALSE(t1 > t2);
    EXPECT_FALSE(t2 == t1);
    EXPECT_FALSE(t1 == t2);
}

TEST (Time, Equal)
{
    UTCTime t1 = UTCTime(1564005746, 1234); // random start time
    UTCTime t2 = UTCTime(1564005746, 1234); // random start time

    EXPECT_FALSE(t2 > t1);
    EXPECT_FALSE(t1 < t2);
    EXPECT_FALSE(t2 < t1);
    EXPECT_FALSE(t1 > t2);
    EXPECT_TRUE(t2 == t1);
    EXPECT_TRUE(t1 == t2);
}

TEST (Time, Plus)
{
    UTCTime t = UTCTime(1564005746, 1234);
    UTCTime t2 = t + 0.100;
    t += 0.1;

    EXPECT_EQ(t2.sec, 1564005746);
    EXPECT_EQ(t2.nsec, 100001234);
    EXPECT_TRUE(t2 == t);
}

TEST (Time, PlusWrap)
{
    UTCTime t = UTCTime(1564005746, uint64_t(0.5*1e9));
    UTCTime t2 = t + 3.6;
    t += 3.6;

    EXPECT_EQ(t2.sec, 1564005750);
    EXPECT_EQ(t2.nsec, int64_t(0.1*1e9));
    EXPECT_TRUE(t2 == t);
}

TEST (Time, Minus)
{
    UTCTime t = UTCTime(1564005746, uint64_t(0.5*1e9));
    UTCTime t2 = t - 0.100;
    t -= 0.1;

    EXPECT_EQ(t2.sec, 1564005746);
    EXPECT_EQ(t2.nsec, int64_t(0.4*1e9));
    EXPECT_TRUE(t2 == t);
}

TEST (Time, MinusWrap)
{
    UTCTime t = UTCTime(1564005746, uint64_t(0.5*1e9));
    UTCTime t2 = t - 3.6;
    t -= 3.6;

    EXPECT_EQ(t2.sec, 1564005742);
    EXPECT_EQ(t2.nsec, int64_t(0.9*1e9));
    EXPECT_TRUE(t2 == t);
}

TEST (Time, AddWrap)
{
    UTCTime t = UTCTime(1564005746, uint64_t(0.5*1e9));
    UTCTime dt = UTCTime(3, uint64_t(0.6*1e9));
    UTCTime t2 = t + dt;
    t += dt;

    EXPECT_EQ(t2.sec, 1564005750);
    EXPECT_EQ(t2.nsec, int64_t(0.1*1e9));
    EXPECT_TRUE(t2 == t);
}

TEST (Time, SubtractWrap)
{
    UTCTime t = UTCTime(1564005746, uint64_t(0.5*1e9));
    UTCTime dt = UTCTime(3, uint64_t(0.6*1e9));
    UTCTime t2 = t - dt;
    t -= dt;

    EXPECT_EQ(t2.sec, 1564005742);
    EXPECT_EQ(t2.nsec, int64_t(0.9*1e9));
    EXPECT_TRUE(t2 == t);
}

TEST (Time, getGPSWeek)
{
    // 21 Jul 2019 12:00:00 AM
    UTCTime t(1563667200,0);
    EXPECT_EQ((t+(double)(UTCTime::LEAP_SECONDS)).GpsWeek(), 2063); // 20 Jul 2019 23:59:42 AM
    EXPECT_EQ((t+(double)(UTCTime::LEAP_SECONDS-1)).GpsWeek(), 2062); // 20 Jul 2019 23:59:41 AM
}

TEST (Time, getGlonassWeek)
{
    UTCTime t0 = UTCTime(1564012800,0); // 25 Jul 2019 00:00:00
    UTCTime t = t0 - 3*3600; // 24 Jul 2019 21:00:00
    EXPECT_EQ(t.GlonassWeek(), 2586);
    EXPECT_EQ(t.GlonassWeek()-1.0, 2585);
}

TEST (Time, getGlonassDayOfWeek)
{
    UTCTime t0(1563667200,0); // 21 Jul 2019 00:00:00
    UTCTime t = t0 - 3*3600; // 20 Jul 2019 21:00:00
    EXPECT_EQ(t.GlonassDayOfWeek(), 3);
    EXPECT_EQ(t.GlonassDayOfWeek()-1.0, 2);
}

TEST (Time, GPSvsUTC)
{
    // 21 Jul 2019 12:00:00 AM
    UTCTime t(1563667200,0);
    t += (double)UTCTime::LEAP_SECONDS;
    int gps_week = 2063;
    int tow_ms = 0;

    UTCTime gps = UTCTime::fromGPS(gps_week, tow_ms);

    EXPECT_EQ(t.GpsWeek(), gps_week);
    EXPECT_EQ(gps, t);

    // 27 Jul 2019 11:59:59 PM
    UTCTime t_end(1564271999,0);
    t_end += (double)UTCTime::LEAP_SECONDS;

    uint64_t tow_ms_end = (UTCTime::SEC_IN_WEEK-1)*1000;
    UTCTime gps_end = UTCTime::fromGPS(gps_week, tow_ms_end);
    EXPECT_EQ(gps_end, t_end);
}

TEST (Time, GlonassvsUTC)
{
    // GLONASS is 3 hours ahead of UTC
    UTCTime t = UTCTime(1564012800,0); // 25 Jul 2019 00:00:00
    UTCTime t2 = t - 3*3600; // 24 Jul 2019 21:00:00

    int glo_week = 2586; // 24 Jul 2019 21:00:00
    int glo_tow_ms = 0;

    UTCTime glo = UTCTime::fromGlonass(glo_week, glo_tow_ms);

    EXPECT_EQ(t2.GlonassWeek(), glo_week);
    EXPECT_EQ(glo, t2);



    UTCTime t_end(1564012799,0); // 24 Jul 2019 23:59:59
    UTCTime t2_end = t_end - 3*3600; // 24 Jul 2019 20:59:59

    int glo_week_end = 2585;
    uint64_t glo_tow_ms_end = (UTCTime::SEC_IN_WEEK-1)*1000;

    UTCTime glo_end = UTCTime::fromGlonass(glo_week_end, glo_tow_ms_end);
    EXPECT_EQ(t2_end.GlonassWeek(), glo_week_end);
    EXPECT_EQ(glo_end, t2_end);
}

TEST (Time, GlonassFromTOD)
{
    UTCTime t0 = UTCTime(1563885000, 0); // 23 Jul 2019 12:30:00
    int glo_tod_ms = (15*3600 + 1800) * 1000;
    UTCTime glo = UTCTime::fromGlonassTimeOfDay(t0, glo_tod_ms);

    EXPECT_EQ(glo, t0);


    UTCTime t = UTCTime(1564012800,0); // 25 Jul 2019 00:00:00
    UTCTime t2 = t - 3*3600; // 24 Jul 2019 21:00:00

    glo_tod_ms = 24*3600*1000;
    glo = UTCTime::fromGlonassTimeOfDay(t2-10, glo_tod_ms);

    EXPECT_EQ(glo, t2);


    UTCTime t_end(1564012799,0); // 24 Jul 2019 23:59:59
    UTCTime t2_end = t_end - 3*3600; // 24 Jul 2019 20:59:59

    glo_tod_ms = (24*3600 - 1)*1000;
    glo = UTCTime::fromGlonassTimeOfDay(t2_end, glo_tod_ms);

    EXPECT_EQ(glo, t2_end);
}


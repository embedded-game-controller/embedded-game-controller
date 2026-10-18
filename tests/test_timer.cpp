#include "egc_timer_cpp.h"

#include <gtest/gtest.h>

static u32 s_mockClockUs = 0;

u32 egc_get_clock_us() {
    return s_mockClockUs;
}

TEST(Timer, noTimer)
{
    s_mockClockUs = 400;

    u32 timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, UINT_MAX);
}

TEST(Timer, single)
{
    s_mockClockUs = 400;

    int invocationCount = 0;
    Egc::Timer t(10, [&](Egc::Timer &timer) { invocationCount++; });

    s_mockClockUs = 409;
    u32 timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 1);
    ASSERT_EQ(invocationCount, 0);

    s_mockClockUs = 410;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 10);
    ASSERT_EQ(invocationCount, 1);
}

TEST(Timer, lateTrigger)
{
    s_mockClockUs = 400;

    int invocationCount = 0;
    Egc::Timer t(10, [&](Egc::Timer &timer) { invocationCount++; });

    s_mockClockUs = 409;
    u32 timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 1);
    ASSERT_EQ(invocationCount, 0);

    s_mockClockUs = 452;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 8);
    ASSERT_EQ(invocationCount, 5);
}

TEST(Timer, many)
{
    s_mockClockUs = 400;

    int invocationCountA = 0;
    Egc::Timer a(20, [&](Egc::Timer &timer) { invocationCountA++; });

    int invocationCountB = 0;
    Egc::Timer b(11, [&](Egc::Timer &timer) { invocationCountB++; });

    int invocationCountC = 0;
    Egc::Timer c(10, [&](Egc::Timer &timer) { invocationCountC++; });

    s_mockClockUs = 409;
    u32 timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 0);
    ASSERT_EQ(invocationCountB, 0);
    ASSERT_EQ(invocationCountC, 0);
    ASSERT_EQ(timeUntilNext, 1);

    s_mockClockUs = 410;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 0);
    ASSERT_EQ(invocationCountB, 0);
    ASSERT_EQ(invocationCountC, 1);
    ASSERT_EQ(timeUntilNext, 1);

    s_mockClockUs = 413;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 0);
    ASSERT_EQ(invocationCountB, 1);
    ASSERT_EQ(invocationCountC, 1);
    ASSERT_EQ(timeUntilNext, 7);

    s_mockClockUs = 419;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 0);
    ASSERT_EQ(invocationCountB, 1);
    ASSERT_EQ(invocationCountC, 1);
    ASSERT_EQ(timeUntilNext, 1);

    s_mockClockUs = 420;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 1);
    ASSERT_EQ(invocationCountB, 1);
    ASSERT_EQ(invocationCountC, 2);
    ASSERT_EQ(timeUntilNext, 2);

    s_mockClockUs = 510;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 5);
    ASSERT_EQ(invocationCountB, 10);
    ASSERT_EQ(invocationCountC, 11);
    ASSERT_EQ(timeUntilNext, 10);
}

TEST(Timer, clockWrapS32)
{
    s_mockClockUs = INT_MAX - 10;

    int invocationCount = 0;
    Egc::Timer t(10, [&](Egc::Timer &timer) { invocationCount++; });

    s_mockClockUs = INT_MAX - 1;
    u32 timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 1);
    ASSERT_EQ(invocationCount, 0);

    s_mockClockUs = INT_MAX;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 10);
    ASSERT_EQ(invocationCount, 1);

    s_mockClockUs = INT_MAX + 1;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 9);
    ASSERT_EQ(invocationCount, 1);

    s_mockClockUs = INT_MAX + 11;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 9);
    ASSERT_EQ(invocationCount, 2);
}

TEST(Timer, clockWrapU32)
{
    s_mockClockUs = UINT_MAX - 10;

    int invocationCount = 0;
    Egc::Timer t(10, [&](Egc::Timer &timer) { invocationCount++; });

    s_mockClockUs = UINT_MAX - 1;
    u32 timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 1);
    ASSERT_EQ(invocationCount, 0);

    s_mockClockUs = UINT_MAX;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 10);
    ASSERT_EQ(invocationCount, 1);

    s_mockClockUs = UINT_MAX + 1;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 9);
    ASSERT_EQ(invocationCount, 1);

    s_mockClockUs = UINT_MAX + 11;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 9);
    ASSERT_EQ(invocationCount, 2);
}

TEST(Timer, deletions)
{
    s_mockClockUs = 400;

    int invocationCountA = 0;
    Egc::Timer a(20, [&](Egc::Timer &timer) { invocationCountA++; });

    int invocationCountB = 0;
    Egc::Timer b(11, [&](Egc::Timer &timer) { invocationCountB++; });

    int invocationCountC = 0;
    Egc::Timer c(10, [&](Egc::Timer &timer) { invocationCountC++; });

    s_mockClockUs = 405;
    u32 timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 0);
    ASSERT_EQ(invocationCountB, 0);
    ASSERT_EQ(invocationCountC, 0);
    ASSERT_EQ(timeUntilNext, 5);

    c.stop();

    s_mockClockUs = 412;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 0);
    ASSERT_EQ(invocationCountB, 1);
    ASSERT_EQ(invocationCountC, 0);
    ASSERT_EQ(timeUntilNext, 8);

    a.stop();
    b.stop();
    s_mockClockUs = 423;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(invocationCountA, 0);
    ASSERT_EQ(invocationCountB, 1);
    ASSERT_EQ(invocationCountC, 0);
    ASSERT_EQ(timeUntilNext, UINT_MAX);
}

TEST(Timer, deletionFromCallback)
{
    s_mockClockUs = 400;

    int invocationCount = 0;
    Egc::Timer t(10, [&](Egc::Timer &timer) {
        invocationCount++;
        if (invocationCount == 2) {
            timer.stop();
        }
    });

    s_mockClockUs = 412;
    u32 timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, 8);
    ASSERT_EQ(invocationCount, 1);

    s_mockClockUs = 423;
    timeUntilNext = egc_timer_process();
    ASSERT_EQ(timeUntilNext, UINT_MAX);
    ASSERT_EQ(invocationCount, 2);
}

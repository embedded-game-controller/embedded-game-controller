#include <limits.h>

#include "egc_timer.h"
#include "utils.h"

static egc_timer_t *s_timers = NULL;

void egc_timer_start(egc_timer_t *timer, u32 period_us, egc_timer_cb callback)
{
    timer->period_us = period_us;
    timer->callback = callback;
    timer->reference_us = egc_get_clock_us();
    timer->next = s_timers;
    s_timers = timer;
}

void egc_timer_stop(egc_timer_t *timer)
{
    egc_timer_t **prev_ptr = &s_timers;
    for (egc_timer_t *t = s_timers; t != NULL; t = t->next) {
        if (t == timer) {
            *prev_ptr = timer->next;
            break;
        }
        prev_ptr = &t->next;
    }
}

u32 egc_timer_process()
{
    u32 time_now_us = egc_get_clock_us();
    u32 until_next_wakeup = UINT_MAX;

    egc_timer_t **prev_ptr = &s_timers;
    egc_timer_t *next = NULL;
    for (egc_timer_t *timer = s_timers; timer != NULL; timer = next) {
        next = timer->next;

        u32 timer_next_trigger = timer->reference_us + timer->period_us;

        bool was_deleted = false;
        while ((s32)(time_now_us - timer_next_trigger) >= 0) {
            /* careful, the callback could remove the timer! */
            timer->callback(timer);
            if (*prev_ptr != timer) {
                /* The timer was deleted */
                was_deleted = true;
                break;
            }
            timer->reference_us = timer_next_trigger;
            timer_next_trigger += timer->period_us;
        }

        if (was_deleted)
            continue;

        u32 remaining = timer_next_trigger - time_now_us;
        if (remaining < until_next_wakeup) {
            until_next_wakeup = remaining;
        }

        prev_ptr = &timer->next;
    }

    return until_next_wakeup;
}

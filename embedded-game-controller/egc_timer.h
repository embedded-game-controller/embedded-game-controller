#ifndef EGC_TIMER_H
#define EGC_TIMER_H

#include <assert.h>

#include "egc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct egc_timer_t egc_timer_t;

typedef void (*egc_timer_cb)(egc_timer_t *timer);

struct egc_timer_t {
    egc_timer_t *next;
    u32 period_us;
    u32 reference_us;
    egc_timer_cb callback;
};

void egc_timer_start(egc_timer_t *timer, u32 period_us, egc_timer_cb callback);
void egc_timer_stop(egc_timer_t *timer);

u32 egc_timer_process(void);

/* Implemented by the platform backend */
u32 egc_get_clock_us(void);

#ifdef __cplusplus
}
#endif

#endif

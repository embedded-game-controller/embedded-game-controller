#include <assert.h>
#include <ogc/system.h>
#include <stdio.h>
#include <string.h>

#include "hid_drivers/gamecube.h"
#include "platform.h"
#include "utils.h"

static egc_event_cb s_event_handler;

static int gc_init(egc_event_cb event_handler)
{
    s_event_handler = event_handler;
    return _egc_gc_init(event_handler);
}

static int gc_wait_events(u32 timeout_us)
{
    return _egc_gc_process_events(s_event_handler);
}

const egc_platform_backend_t _egc_platform_backend = {
    .init = gc_init,
    .wait_events = gc_wait_events,
};

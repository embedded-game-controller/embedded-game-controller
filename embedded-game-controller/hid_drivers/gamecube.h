#ifndef EGC_DRIVERS_GAMECUBE_H
#define EGC_DRIVERS_GAMECUBE_H

#include "platform.h"

int _egc_gc_init(egc_event_cb event_handler);
int _egc_gc_process_events(egc_event_cb event_handler);

extern const egc_device_driver_t gc_device_driver;

#endif /* EGC_DRIVERS_GAMECUBE_H */

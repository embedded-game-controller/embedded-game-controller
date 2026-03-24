#ifndef EGC_INTERNALS_H
#define EGC_INTERNALS_H

#include "egc.h"

#define EGC_INPUT_DEVICE_DRIVER_DATA_SIZE 64

typedef struct {
    egc_input_device_t pub;

    const egc_device_driver_t *driver ATTRIBUTE_ALIGN(4);
    u8 private_data[EGC_INPUT_DEVICE_DRIVER_DATA_SIZE] ATTRIBUTE_ALIGN(4);
} egc_device_priv_t;

extern bool _egc_sensor_bar_position_top;

static inline egc_device_priv_t *get_priv(egc_input_device_t *pub)
{
    return (egc_device_priv_t *)pub;
}

#endif

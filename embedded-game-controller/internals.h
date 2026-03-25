#ifndef EGC_INTERNALS_H
#define EGC_INTERNALS_H

#include "egc.h"
#include "utils.h"

#define EGC_INPUT_DEVICE_DRIVER_DATA_SIZE 64

typedef struct {
    egc_input_device_t pub;

    u8 endpoint_in;
    u8 endpoint_out;
    /* Intervals are in milliseconds */
    u8 interval_in;
    u8 interval_out;
    /* How much we should wait until the next transfer (in ms) */
    u8 wait_time_in;
    u8 wait_time_out;

    const egc_device_driver_t *driver ATTRIBUTE_ALIGN(4);
    u8 private_data[EGC_INPUT_DEVICE_DRIVER_DATA_SIZE] ATTRIBUTE_ALIGN(4);
} egc_device_priv_t;

extern bool _egc_sensor_bar_position_top;

static inline egc_device_priv_t *get_priv(egc_input_device_t *pub)
{
    return (egc_device_priv_t *)pub;
}

/* Returns true if any endpoint timeout expired */
static inline bool _egc_update_endpoint_timeout(egc_device_priv_t *priv, u32 elapsed_ms)
{
    bool endpoint_became_available = false;
    u8 *timeouts[] = { &priv->wait_time_in, &priv->wait_time_out };

    for (int i = 0; i < ARRAY_SIZE(timeouts); i++) {
        u8 *timeout = timeouts[i];
        if (*timeout > 0) {
            if (elapsed_ms >= *timeout) {
                *timeout = 0;
                endpoint_became_available = true;
            } else {
                *timeout -= elapsed_ms;
            }
        }
    }
    return endpoint_became_available;
}

#endif

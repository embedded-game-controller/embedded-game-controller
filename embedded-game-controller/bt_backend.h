#ifndef EGC_BT_BACKEND_H
#define EGC_BT_BACKEND_H

/* The Bluetooth HID interface is nearly equivelent to the USB one. We include
 * usb.h in order to reuse the protocol structures. */
#include "usb.h"

typedef struct egc_bt_backend_t egc_bt_backend_t;

typedef struct {
    u16 vendor_id;
    u16 product_id;
    /* We can add here other fields from the DID or HID information retrieved
     * with the SDP query, if needed. But then we'll also need to update the
     * probe() and init() methods of the egc_device_driver_t interface in order
     * to make use of the new fields. */
} egc_bt_device_desc_t;

/* Interface for platform-specific BT backends. */
struct egc_bt_backend_t {
    egc_input_device_t *(*device_alloc)(const egc_bt_device_desc_t *desc);
    int (*device_add)(egc_input_device_t *device);
    void (*device_free)(egc_input_device_t *device);
};

#endif /* EGC_BT_BACKEND_H */

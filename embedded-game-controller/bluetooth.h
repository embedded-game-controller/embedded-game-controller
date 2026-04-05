#ifndef EGC_BLUETOOTH_H
#define EGC_BLUETOOTH_H

#include "usb.h"

int _egc_bt_initialize();

const egc_usb_transfer_t *_egc_bt_ctrl_transfer(egc_input_device_t *device, u8 requesttype,
                                                u8 request, u16 value, u16 index, void *data,
                                                u16 length, egc_transfer_cb callback);

int _egc_bt_intr_transfer(egc_input_device_t *device, void *data, u16 length);

#endif /* EGC_BLUETOOTH_H */

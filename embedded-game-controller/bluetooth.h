#ifndef EGC_BLUETOOTH_H
#define EGC_BLUETOOTH_H

#include "usb.h"

int _egc_bt_initialize();

const egc_usb_transfer_t *_egc_bt_ctrl_transfer(egc_input_device_t *device, u8 requesttype,
                                                u8 request, u16 value, u16 index, void *data,
                                                u16 length, egc_transfer_cb callback);

int _egc_bt_intr_transfer(egc_input_device_t *device, void *data, u16 length);

typedef struct bte_hci_t BteHci;
typedef void (*egc_bt_initialized_cb)(BteHci *hci);
void _egc_bt_on_initialized(egc_bt_initialized_cb callback);

void _egc_bt_run_inquiry();

#endif /* EGC_BLUETOOTH_H */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bluetooth.h"
#include "driver_api.h"
#include "internals.h"
#include "platform.h"
#include "usb.h"
#include "usb_backend.h"
#include "utils.h"

static const egc_device_driver_t *usb_device_drivers[] = {
    &ds3_usb_device_driver,
    &ds4_usb_device_driver,
    &dr_usb_device_driver,
    &ns_usb_device_driver,
};

static egc_input_device_cb s_device_added_cb = NULL;
static egc_input_device_cb s_device_removed_cb = NULL;
static void *s_callbacks_userdata = NULL;

static void read_interrupts(egc_input_device_t *device);

static inline const egc_device_driver_t *get_usb_device_driver_for(u16 vid, u16 pid)
{
    for (int i = 0; i < ARRAY_SIZE(usb_device_drivers); i++) {
        if (usb_device_drivers[i]->probe(vid, pid))
            return usb_device_drivers[i];
    }

    return NULL;
}

static void interrupt_read_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED) {
        _egc_input_device_intr_data_received(device, transfer->data, transfer->length);
    }

    read_interrupts(device);
}

static void read_interrupts(egc_input_device_t *device)
{
    egc_device_priv_t *priv = get_priv(device);

    if (device->suspended || !priv->driver->intr_event ||
        device->connection != EGC_CONNECTION_USB || !(priv->endpoint_in & EGC_USB_ENDPOINT_IN))
        return;

    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, priv->endpoint_in, NULL, 0, interrupt_read_cb);
    if (!transfer) {
        EGC_DEBUG("Could not get a in transfer!");
    }
}

/* API exposed to USB device drivers */
egc_device_description_t *egc_device_driver_alloc_desc(egc_input_device_t *device)
{
    return _egc_platform_backend.alloc_desc(device);
}

void egc_device_driver_set_endpoints(egc_input_device_t *device, u8 endpoint_in, u8 interval_in,
                                     u8 endpoint_out, u8 interval_out)
{
    egc_device_priv_t *priv = get_priv(device);
    priv->endpoint_in = endpoint_in;
    priv->endpoint_out = endpoint_out;
    priv->interval_in = interval_in;
    priv->interval_out = interval_out;
}

int egc_device_driver_send_output_report(egc_input_device_t *device, void *data, u16 length)
{
    egc_device_priv_t *priv = get_priv(device);

    const egc_usb_transfer_t *transfer =
        egc_device_driver_issue_intr_transfer_async(device, priv->endpoint_out, data, length, NULL);
    if (device->connection != EGC_CONNECTION_BT && !transfer) {
        EGC_DEBUG("Could not get a transfer for out %02x!", ((u8 *)data)[0]);
        return -1;
    }
    return 0;
}

void _egc_input_device_intr_data_received(egc_input_device_t *device, const void *data, u16 length)
{
    egc_device_priv_t *priv = get_priv(device);
    priv->driver->intr_event(device, data, length);
}

bool _egc_can_submit_transfer(egc_usb_transfer_t *t)
{
    egc_input_device_t *device = t->device;
    egc_device_priv_t *priv = get_priv(device);

    /* We only throttle interrupt transfers */
    if (t->transfer_type != EGC_USB_TRANSFER_INTERRUPT)
        return true;

    bool can_submit = true;
    if (t->endpoint == priv->endpoint_in) {
        can_submit = priv->wait_time_in == 0;
        if (can_submit)
            priv->wait_time_in = priv->interval_in;
    } else if (t->endpoint == priv->endpoint_out) {
        can_submit = priv->wait_time_out == 0;
        if (can_submit)
            priv->wait_time_out = priv->interval_out;
    }

    return can_submit;
}

const egc_usb_transfer_t *egc_device_driver_issue_ctrl_transfer_async(egc_input_device_t *device,
                                                                      u8 requesttype, u8 request,
                                                                      u16 value, u16 index,
                                                                      void *data, u16 length,
                                                                      egc_transfer_cb callback)
{
    if (device->connection == EGC_CONNECTION_USB) {
        return _egc_platform_backend.usb.ctrl_transfer_async(device, requesttype, request, value,
                                                             index, data, length, callback);
    } else if (device->connection == EGC_CONNECTION_BT) {
        return _egc_bt_ctrl_transfer(device, requesttype, request, value, index, data, length,
                                     callback);
    }
    return NULL;
}

const egc_usb_transfer_t *egc_device_driver_issue_intr_transfer_async(egc_input_device_t *device,
                                                                      u8 endpoint, void *data,
                                                                      u16 length,
                                                                      egc_transfer_cb callback)
{
    if (device->connection == EGC_CONNECTION_USB) {
        return _egc_platform_backend.usb.intr_transfer_async(device, endpoint, data, length,
                                                             callback);
    } else if (device->connection == EGC_CONNECTION_BT) {
        /* Only perform the operation if this is an output transfer;
         * inputs are received over the HID interrupt L2CAP channel. */
        if (endpoint & EGC_USB_ENDPOINT_IN)
            return NULL;
        _egc_bt_intr_transfer(device, data, length);
    }
    return NULL;
}

static bool timer_cb_wrapper(egc_input_device_t *device)
{
    egc_device_priv_t *priv = get_priv(device);
    bool keep = (device->connection != EGC_CONNECTION_DISCONNECTED && priv->driver->timer)
                    ? priv->driver->timer(device)
                    : false;
    return keep;
}

int egc_device_driver_set_timer(egc_input_device_t *device, int time_us, int repeat_time_us)
{
    return _egc_platform_backend.set_timer(device, time_us, repeat_time_us, timer_cb_wrapper);
}

int egc_device_driver_report_input(egc_input_device_t *device, const egc_input_state_t *state)
{
    return _egc_platform_backend.report_input(device, state);
}

u32 egc_device_driver_map_buttons(u32 buttons, int count, const egc_gamepad_button_e *map)
{
    u32 ret = 0;
    for (int i = 0; i < count; i++) {
        if (buttons & (1 << i)) {
            ret |= 1 << map[i];
        }
    }
    return ret;
}

int egc_input_device_resume(egc_input_device_t *device)
{
    egc_device_priv_t *priv = get_priv(device);

    EGC_DEBUG("");

    if (!device->suspended)
        return 0;

        /* FIXME: Doesn't work properly with DS3.
         * It doesn't report any data after suspend+resume... */
#if 0
    if (usb_hid_v5_suspend_resume(device->host_fd, device->dev_id, 1, 0) != IOS_OK)
        return IOS_ENOENT;
#endif
    device->suspended = false;

    if (priv->driver->init) {
        const egc_usb_devdesc_t *desc = _egc_platform_backend.usb.get_device_descriptor(device);
        return priv->driver->init(device, desc->idVendor, desc->idProduct);
    }

    read_interrupts(device);
    return 0;
}

int egc_input_device_suspend(egc_input_device_t *device)
{
    egc_device_priv_t *priv = get_priv(device);
    int ret = 0;

    EGC_DEBUG("");

    if (priv->driver->disconnect)
        ret = priv->driver->disconnect(device);

        /* Suspend the device */
#if 0
	usb_hid_v5_suspend_resume(device->host_fd, device->dev_id, 0, 0);
#endif
    device->suspended = true;

    return ret;
}

int egc_input_device_set_leds(egc_input_device_t *device, u32 led_state)
{
    egc_device_priv_t *priv = get_priv(device);

    EGC_DEBUG("");

    if (priv->driver->set_leds)
        return priv->driver->set_leds(device, led_state);

    return 0;
}

int egc_input_device_set_rumble(egc_input_device_t *device, u32 intensity)
{
    egc_device_priv_t *priv = get_priv(device);

    EGC_DEBUG("");

    if (priv->driver->set_rumble)
        return priv->driver->set_rumble(device, intensity > 0);

    return 0;
}

static int on_device_added(egc_input_device_t *device, u16 vid, u16 pid)
{
    egc_device_priv_t *priv = get_priv(device);
    const egc_device_driver_t *driver;

    /* Find if we have a driver for that VID/PID */
    driver = get_usb_device_driver_for(vid, pid);
    if (!driver)
        return -1;

    /* We have ownership, populate the device info */
    priv->driver = driver;
    if (driver->init) {
        int rc = driver->init(device, vid, pid);
        if (rc < 0)
            return rc;
    }

    read_interrupts(device);

    /* Inform the client */
    if (s_device_added_cb)
        s_device_added_cb(device, s_callbacks_userdata);
    return 0;
}

static int on_device_removed(egc_input_device_t *device)
{
    egc_device_priv_t *priv = get_priv(device);
    int rc = 0;

    /* Inform the client */
    if (s_device_removed_cb)
        s_device_removed_cb(device, s_callbacks_userdata);

    if (priv->driver && priv->driver->disconnect)
        rc = priv->driver->disconnect(device);

    return rc;
}

static void on_device_input(egc_input_device_t *device, void *data, u16 length)
{
    egc_device_priv_t *priv = get_priv(device);
    if (priv->driver->intr_event) {
        priv->driver->intr_event(device, data, length);
    }
}

static int event_handler(egc_input_device_t *device, egc_event_e event, ...)
{
    va_list args;
    va_start(args, event);
    int rc = -1;

    if (event == EGC_EVENT_DEVICE_INPUT) {
        void *buffer = va_arg(args, void *);
        u16 length = va_arg(args, int);
        on_device_input(device, buffer, length);
        rc = 0;
    } else if (event == EGC_EVENT_DEVICE_ADDED) {
        u16 vid = va_arg(args, int);
        u16 pid = va_arg(args, int);
        rc = on_device_added(device, vid, pid);
    } else if (event == EGC_EVENT_DEVICE_REMOVED) {
        rc = on_device_removed(device);
    }
    va_end(args);
    return rc;
}

int egc_initialize(egc_input_device_cb added_cb, egc_input_device_cb removed_cb, void *userdata)
{
    s_device_added_cb = added_cb;
    s_device_removed_cb = removed_cb;
    s_callbacks_userdata = userdata;
    int rc = _egc_platform_backend.init(event_handler);
    if (rc < 0)
        return rc;

#if WITH_BLUETOOTH
    rc = _egc_bt_initialize();
#endif

    return rc;
}

int egc_input_device_set_suspended(egc_input_device_t *device, bool suspended)
{
    if (device->connection == EGC_CONNECTION_USB) {
        return _egc_platform_backend.usb.set_suspended
                   ? _egc_platform_backend.usb.set_suspended(device, suspended)
                   : -1;
    }
    return -1;
}

int egc_handle_events()
{
    return _egc_platform_backend.wait_events(0);
}

int egc_wait_events(u32 timeout_us)
{
    return _egc_platform_backend.wait_events(timeout_us);
}

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
#ifdef WITH_DRIVER_DS3
    &ds3_usb_device_driver,
#endif
#ifdef WITH_DRIVER_DS4
    &ds4_usb_device_driver,
#endif
#ifdef WITH_DRIVER_GENERIC
    &dr_usb_device_driver,
#endif
#ifdef WITH_DRIVER_NINTENDO_SWITCH
    &ns_usb_device_driver,
#endif
#ifdef WITH_DRIVER_WIIMOTE
    &wm_device_driver,
#endif
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
        device, priv->endpoint_in, NULL, priv->endpoint_in_size, interrupt_read_cb);
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

void egc_device_driver_set_read_size(egc_input_device_t *device, u8 size)
{
    egc_device_priv_t *priv = get_priv(device);
    priv->endpoint_in_size = size;
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
        egc_gamepad_button_e button_index = map[i];
        if (button_index < EGC_GAMEPAD_BUTTON_COUNT && buttons & (1 << i)) {
            ret |= 1 << button_index;
        }
    }
    return ret;
}

u32 egc_device_driver_extract_bits(const u8 *data, int offset, int n)
{
    int i = 0;
    int bit_nr = 0;
    int bit_shift = offset;
    int bits_to_copy = 8 - bit_shift;
    u32 value = 0;
    u32 mask = (1 << n) - 1;

    while (n > 0) {
        value |= ((u32)data[i] >> bit_shift) << bit_nr;
        n -= bits_to_copy;
        bit_nr += bits_to_copy;
        bits_to_copy = 8;
        bit_shift = 0;
        i++;
    }

    return value & mask;
}

static inline u32 parse_dpad(u8 dpad)
{
    switch (dpad) {
    case 0:
        return 1 << EGC_GAMEPAD_BUTTON_DPAD_UP;
    case 1:
        return (1 << EGC_GAMEPAD_BUTTON_DPAD_UP) | (1 << EGC_GAMEPAD_BUTTON_DPAD_RIGHT);
    case 2:
        return 1 << EGC_GAMEPAD_BUTTON_DPAD_RIGHT;
    case 3:
        return (1 << EGC_GAMEPAD_BUTTON_DPAD_RIGHT) | (1 << EGC_GAMEPAD_BUTTON_DPAD_DOWN);
    case 4:
        return 1 << EGC_GAMEPAD_BUTTON_DPAD_DOWN;
    case 5:
        return (1 << EGC_GAMEPAD_BUTTON_DPAD_DOWN) | (1 << EGC_GAMEPAD_BUTTON_DPAD_LEFT);
    case 6:
        return 1 << EGC_GAMEPAD_BUTTON_DPAD_LEFT;
    case 7:
        return (1 << EGC_GAMEPAD_BUTTON_DPAD_LEFT) | (1 << EGC_GAMEPAD_BUTTON_DPAD_UP);
    default:
        return 0;
    }
}

u16 egc_device_driver_parse_report(const void *raw_report, const u8 *elements,
                                   struct egc_input_state_t *state)
{
    const u8 *data = raw_report;
    int offset = 0;
    u8 type;
    do {
        type = *(elements++);
        if (type == EGC_INPUT_REPORT_TYPE_SKIP) {
            int skip_bits = *(elements++);
            offset += skip_bits;
        } else if (type == EGC_INPUT_REPORT_TYPE_BUTTON4 ||
                   type == EGC_INPUT_REPORT_TYPE_BUTTON4_INVERTED) {
            /* This type is always aligned to a nibble, so we never
             * increment the data pointer within this loop */
            for (int i = 0; i < 4; i++) {
                egc_gamepad_button_e code = *(elements++);
                if (code == EGC_GAMEPAD_BUTTON_INVALID)
                    continue;

                u8 b = data[offset / 8];
                if (type == EGC_INPUT_REPORT_TYPE_BUTTON4_INVERTED) {
                    b = ~b;
                }
                int bit_offset = offset % 8;
                bool down = (b & (1 << (7 - (bit_offset + i)))) != 0;
                if (code == EGC_GAMEPAD_BUTTON_LEFT_TRIGGER) {
                    egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFT_TRIGGER,
                                               down * INT16_MAX);
                } else if (code == EGC_GAMEPAD_BUTTON_RIGHT_TRIGGER) {
                    egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_RIGHT_TRIGGER,
                                               down * INT16_MAX);
                } else if (down) {
                    egc_device_driver_set_button(state, code);
                }
            }
            offset += 4;
        } else if (type == EGC_INPUT_REPORT_TYPE_DPAD) {
            u8 b = data[offset / 8];
            if (offset % 8 == 0) {
                b >>= 4;
            } else {
                b &= 0xf;
            }
            egc_device_driver_set_buttons(state, parse_dpad(b));
            offset += 4;
        } else if (type >= EGC_INPUT_REPORT_TYPE_AXIS_FIRST &&
                   type <= EGC_INPUT_REPORT_TYPE_AXIS_LAST) {
#define EGC_TRIGGER_VALUE 2048
            s16 value = 0;
            EgcInputReportElementType axis_type = type & ~EGC_INPUT_REPORT_TYPE_AXIS_INVERTED;
            if (axis_type == EGC_INPUT_REPORT_TYPE_AXIS_S8) {
                u8 b = data[offset / 8];
                value = (b << 8) | (b << 1) | ((b >> 6) & 0x1);
                offset += 8;
            } else if (axis_type == EGC_INPUT_REPORT_TYPE_AXIS_U8) {
                value = egc_u8_to_s16(data[offset / 8]);
                offset += 8;
            } else if (axis_type == EGC_INPUT_REPORT_TYPE_AXIS_U8_MONO) {
                u8 b = data[offset / 8];
                value = (b << 7) | (b >> 1);
                offset += 8;
            } else if (axis_type == EGC_INPUT_REPORT_TYPE_AXIS_S16LE) {
                value = (s16)((data[offset / 8 + 1] << 8) | data[offset / 8]);
                offset += 16;
            } else {
                // TODO
            }
            egc_gamepad_axis_e axis = *(elements++);
            if (type & EGC_INPUT_REPORT_TYPE_AXIS_INVERTED) {
                value = -value - 1;
            }

            if (axis == EGC_GAMEPAD_AXIS_DPADX) {
                if (value < -EGC_TRIGGER_VALUE) {
                    egc_device_driver_set_button(state, EGC_GAMEPAD_BUTTON_DPAD_LEFT);
                } else if (value > EGC_TRIGGER_VALUE) {
                    egc_device_driver_set_button(state, EGC_GAMEPAD_BUTTON_DPAD_RIGHT);
                }
            } else if (axis == EGC_GAMEPAD_AXIS_DPADY) {
                if (value < -EGC_TRIGGER_VALUE) {
                    egc_device_driver_set_button(state, EGC_GAMEPAD_BUTTON_DPAD_UP);
                } else if (value > EGC_TRIGGER_VALUE) {
                    egc_device_driver_set_button(state, EGC_GAMEPAD_BUTTON_DPAD_DOWN);
                }
            } else {
                egc_device_driver_set_axis(state, axis, value);
            }
#undef EGC_TRIGGER_VALUE
        }
    } while (type != EGC_INPUT_REPORT_TYPE_END);
    return (offset + 7) / 8;
}

void egc_device_driver_fill_desc(egc_device_description_t *desc, const u8 *elements)
{
    u8 type;
    do {
        type = *(elements++);
        if (type == EGC_INPUT_REPORT_TYPE_SKIP) {
            elements++;
        } else if (type == EGC_INPUT_REPORT_TYPE_BUTTON4) {
            for (int i = 0; i < 4; i++) {
                egc_gamepad_button_e code = *(elements++);
                if (code == EGC_GAMEPAD_BUTTON_INVALID)
                    continue;

                if (code == EGC_GAMEPAD_BUTTON_LEFT_TRIGGER) {
                    desc->available_axes |= (1 << EGC_GAMEPAD_AXIS_LEFT_TRIGGER);
                } else if (code == EGC_GAMEPAD_BUTTON_RIGHT_TRIGGER) {
                    desc->available_axes |= (1 << EGC_GAMEPAD_AXIS_RIGHT_TRIGGER);
                } else {
                    desc->available_buttons |= (1 << code);
                }
            }
        } else if (type == EGC_INPUT_REPORT_TYPE_DPAD) {
            desc->available_buttons |= EGC_GAMEPAD_BUTTON_DPAD_UP | EGC_GAMEPAD_BUTTON_DPAD_RIGHT |
                                       EGC_GAMEPAD_BUTTON_DPAD_DOWN | EGC_GAMEPAD_BUTTON_DPAD_LEFT;
        } else if (type >= EGC_INPUT_REPORT_TYPE_AXIS_FIRST &&
                   type <= EGC_INPUT_REPORT_TYPE_AXIS_LAST) {
            egc_gamepad_axis_e axis = *(elements++);
            if (axis == EGC_GAMEPAD_AXIS_DPADX) {
                desc->available_buttons |=
                    EGC_GAMEPAD_BUTTON_DPAD_RIGHT | EGC_GAMEPAD_BUTTON_DPAD_LEFT;
            } else if (axis == EGC_GAMEPAD_AXIS_DPADY) {
                desc->available_buttons |=
                    EGC_GAMEPAD_BUTTON_DPAD_UP | EGC_GAMEPAD_BUTTON_DPAD_DOWN;
            } else {
                desc->available_axes |= (1 << axis);
            }
        }
    } while (type != EGC_INPUT_REPORT_TYPE_END);
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

int egc_input_device_set_rumble(egc_input_device_t *device, u16 low_frequency, u16 high_frequency)
{
    egc_device_priv_t *priv = get_priv(device);

    EGC_DEBUG("");

    if (priv->driver->set_rumble)
        return priv->driver->set_rumble(device, low_frequency, high_frequency);

    return 0;
}

static int on_device_added(egc_input_device_t *device, u16 vid, u16 pid)
{
    egc_device_priv_t *priv = get_priv(device);
    if (!priv->driver) {
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
    }

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
    if (rc < 0) {
        EGC_WARN("Bluetooth initialization failed (%d), continuing without BT support", rc);
        rc = 0;
    }
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

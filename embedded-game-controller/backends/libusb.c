#include <assert.h>
#include <libusb.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WITH_BLUETOOTH
#include <bt-embedded/backends/libusb.h>
#include <bt-embedded/services/hid.h>

#include "bluetooth.h"
#endif

#include "platform.h"
#include "utils.h"

/* Maximum number of connected USB controllers. Increase this if needed. */
#define MAX_ACTIVE_DEVICES 8
#define INPUT_BUFFER_LEN   128

typedef struct {
    /* This must be the first member, since we use it for casting */
    egc_device_priv_t priv;
    libusb_device_handle *handle;
    egc_device_description_t desc;
    egc_usb_devdesc_t usbdesc;
    /* Timer ID (-1 if unset), its address used as timer cookie */
    int64_t timer_us;
    int64_t repeat_timer_us;
    egc_timer_cb timer_callback;
} lu_device_t;

#define PUB(d) (&(d)->priv.pub)

typedef struct {
    egc_usb_transfer_t t;
    struct libusb_transfer *usb;
    egc_transfer_cb callback;
    u8 buffer[INPUT_BUFFER_LEN];
} lu_transfer_t;

static libusb_context *s_libusb_ctx = NULL;
static lu_device_t s_devices[MAX_ACTIVE_DEVICES];
static egc_event_cb s_event_handler;

static inline lu_device_t *lu_device_from_input_device(egc_input_device_t *input_device)
{
    return (lu_device_t *)input_device;
}

static inline lu_transfer_t *lu_transfer_from_libusb(struct libusb_transfer *t)
{
    return t->user_data;
}

static inline lu_transfer_t *lu_transfer_from_egc(egc_usb_transfer_t *t)
{
    return (lu_transfer_t *)((u8 *)t - offsetof(lu_transfer_t, t));
}

static int64_t timespec_to_us(const struct timespec *ts)
{
    return ts->tv_sec * 1000000 + ts->tv_nsec / 1000;
}

static void device_free(lu_device_t *device)
{
    if (device->handle) {
        libusb_close(device->handle);
    }
    memset(device, 0, sizeof(*device));
}

static inline lu_device_t *get_free_device_slot(void)
{
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        if (PUB(&s_devices[i])->connection == EGC_CONNECTION_DISCONNECTED)
            return &s_devices[i];
    }

    return NULL;
}

#if WITH_BLUETOOTH
static egc_input_device_t *lu_bt_device_alloc(const egc_bt_device_desc_t *desc)
{
    lu_device_t *device = get_free_device_slot();
    if (!device)
        return NULL;

    memset(device, 0, sizeof(*device));
    device->desc.vendor_id = desc->vendor_id;
    device->desc.product_id = desc->product_id;
    return PUB(device);
}

static int lu_bt_device_add(egc_input_device_t *input_device)
{
    lu_device_t *device = lu_device_from_input_device(input_device);
    return s_event_handler(input_device, EGC_EVENT_DEVICE_ADDED, device->desc.vendor_id,
                           device->desc.product_id);
}

static void lu_bt_device_free(egc_input_device_t *input_device)
{
    lu_device_t *device = lu_device_from_input_device(input_device);

    s_event_handler(PUB(device), EGC_EVENT_DEVICE_REMOVED);
    device_free(device);
}
#endif

static void transfer_cb(struct libusb_transfer *transfer)
{
    lu_transfer_t *t = lu_transfer_from_libusb(transfer);
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        t->t.status = EGC_USB_TRANSFER_STATUS_COMPLETED;
    } else {
        t->t.status = EGC_USB_TRANSFER_STATUS_ERROR;
    }
    if (t->callback)
        t->callback(&t->t);
    free(t);
}

static const egc_usb_transfer_t *lu_ctrl_transfer_async(egc_input_device_t *input_device,
                                                        u8 requesttype, u8 request, u16 value,
                                                        u16 index, void *data, u16 length,
                                                        egc_transfer_cb callback)
{
    lu_device_t *device = lu_device_from_input_device(input_device);
    lu_transfer_t *t;
    u8 *buffer;

    t = malloc(sizeof(lu_transfer_t));
    if (!t)
        return NULL;

    buffer = t->buffer;
    t->usb = libusb_alloc_transfer(0);

    libusb_fill_control_setup(buffer, requesttype, request, value, index, length);
    memcpy(buffer + 8, data, length);

    libusb_fill_control_transfer(t->usb, device->handle, buffer, transfer_cb, t, 3000);
    t->t.device = input_device;
    t->t.transfer_type = EGC_USB_TRANSFER_CONTROL;
    t->t.status = EGC_USB_TRANSFER_STATUS_UNSET;
    t->t.endpoint = requesttype;
    t->t.length = length;
    t->t.data = buffer + 8;
    t->callback = callback;
    int rc = libusb_submit_transfer(t->usb);
    if (rc != LIBUSB_SUCCESS) {
        EGC_WARN("Transfer failed: %d", rc);
        libusb_free_transfer(t->usb);
        free(t);
        return NULL;
    }

    return &t->t;
}

static const egc_usb_transfer_t *lu_intr_transfer_async(egc_input_device_t *input_device,
                                                        u8 endpoint, void *data, u16 length,
                                                        egc_transfer_cb callback)
{
    lu_device_t *device = lu_device_from_input_device(input_device);
    lu_transfer_t *t;
    u8 *buffer;

    t = malloc(sizeof(lu_transfer_t) + 8 + length);
    if (!t)
        return NULL;

    buffer = t->buffer;
    t->usb = libusb_alloc_transfer(0);

    if (data)
        memcpy(buffer, data, length);
    if (endpoint & LIBUSB_ENDPOINT_IN) {
        length = sizeof(t->buffer);
    }

    libusb_fill_interrupt_transfer(t->usb, device->handle, endpoint, buffer, length, transfer_cb, t,
                                   3000);

    t->t.device = input_device;
    t->t.transfer_type = EGC_USB_TRANSFER_INTERRUPT;
    t->t.endpoint = endpoint;
    t->t.length = length;
    t->t.data = buffer;
    t->callback = callback;
    int rc = libusb_submit_transfer(t->usb);
    if (rc != LIBUSB_SUCCESS) {
        EGC_WARN("Transfer failed: %d", rc);
        libusb_free_transfer(t->usb);
        free(t);
        return NULL;
    }
    return &t->t;
}

static int lu_set_timer(egc_input_device_t *input_device, int time_us, int repeat_time_us,
                        egc_timer_cb callback)
{
    lu_device_t *device = lu_device_from_input_device(input_device);

    if (time_us > 0) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        int64_t now = timespec_to_us(&ts);
        device->timer_us = now + time_us;
    } else {
        device->timer_us = 0;
    }
    device->repeat_timer_us = repeat_time_us;
    device->timer_callback = callback;
    return 0;
}

static int lu_report_input(egc_input_device_t *device, const egc_input_state_t *state)
{
    memcpy(&device->state, state, sizeof(*state));
    return 0;
}

static int on_device_added(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event,
                           void *user_data)
{
    struct libusb_device_descriptor desc;
    lu_device_t *device;

    int rc = libusb_get_device_descriptor(dev, &desc);
    if (rc != LIBUSB_SUCCESS)
        return 0;

    EGC_DEBUG("Device attached: %04x:%04x", desc.idVendor, desc.idProduct);

    /* Get an empty device slot */
    device = get_free_device_slot();
    if (!device)
        return 0;

    rc = libusb_open(dev, &device->handle);
    if (rc != LIBUSB_SUCCESS)
        return 0;

    libusb_set_auto_detach_kernel_driver(device->handle, 1);
    rc = libusb_claim_interface(device->handle, 0);
    if (rc != LIBUSB_SUCCESS) {
        EGC_WARN("Failed to claim USB interface (%d)", rc);
        return 0;
    }
    memcpy(&device->usbdesc, &desc, sizeof(device->usbdesc));
    device->desc.vendor_id = desc.idVendor;
    device->desc.product_id = desc.idProduct;
    device->timer_us = 0;
    device->repeat_timer_us = 0;
    device->timer_callback = NULL;
    PUB(device)->connection = EGC_CONNECTION_USB;

    rc = s_event_handler(PUB(device), EGC_EVENT_DEVICE_ADDED, desc.idVendor, desc.idProduct);
    if (rc < 0) {
        libusb_release_interface(device->handle, 0);
    }
    return 0;
}

static int on_device_removed(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event,
                             void *user_data)
{
    lu_device_t *device;

    EGC_DEBUG("Device removed");
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        device = &s_devices[i];
        if (PUB(device)->connection == EGC_CONNECTION_USB &&
            libusb_get_device(device->handle) == dev) {
            s_event_handler(PUB(device), EGC_EVENT_DEVICE_REMOVED);
            device_free(device);
            break;
        }
    }
    return 0;
}

static int lu_init(egc_event_cb event_handler)
{
    int rc;

    s_event_handler = event_handler;

    rc = libusb_init_context(&s_libusb_ctx, NULL, 0);
    if (rc != LIBUSB_SUCCESS)
        return rc;

    rc = libusb_hotplug_register_callback(s_libusb_ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED,
                                          LIBUSB_HOTPLUG_ENUMERATE, LIBUSB_HOTPLUG_MATCH_ANY,
                                          LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
                                          on_device_added, NULL, NULL);
    if (LIBUSB_SUCCESS != rc) {
        libusb_exit(NULL);
        return rc;
    }

    rc = libusb_hotplug_register_callback(s_libusb_ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
                                          LIBUSB_HOTPLUG_ENUMERATE, LIBUSB_HOTPLUG_MATCH_ANY,
                                          LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
                                          on_device_removed, NULL, NULL);
    if (LIBUSB_SUCCESS != rc) {
        libusb_exit(NULL);
        return rc;
    }

#if WITH_BLUETOOTH
    bte_backend_libusb_set_context(s_libusb_ctx);
#endif
    return 0;
}

static egc_device_description_t *lu_alloc_desc(egc_input_device_t *input_device)
{
    lu_device_t *device = lu_device_from_input_device(input_device);
    input_device->desc = &device->desc;
    return &device->desc;
}

static const egc_usb_devdesc_t *lu_get_device_descriptor(egc_input_device_t *input_device)
{
    lu_device_t *device = lu_device_from_input_device(input_device);
    return &device->usbdesc;
}

/* Invoke any expired timers and return the number of microseconds until the next trigger */
static int64_t invoke_timers()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t now = timespec_to_us(&ts);
    int64_t min_timeout = INT64_MAX;

    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        lu_device_t *device = &s_devices[i];
        if (PUB(device)->connection == EGC_CONNECTION_DISCONNECTED)
            continue;
        if (device->timer_us <= 0)
            continue;

        int64_t remaining_us = device->timer_us - now;
        if (remaining_us > 0) {
            if (remaining_us < min_timeout) {
                min_timeout = remaining_us;
            }
            continue;
        }

        /* timer has expired, invoke the callback */
        bool keep = device->timer_callback(PUB(device));
        if (keep) {
            /* Set the repeat timer */
            device->timer_us = now + device->repeat_timer_us;
        } else {
            device->timer_us = 0;
            device->repeat_timer_us = 0;
        }
    }

    return min_timeout;
}

static int lu_wait_events(u32 timeout_us)
{
    int64_t next_timer_us = invoke_timers();
    if (next_timer_us < timeout_us)
        timeout_us = next_timer_us;

    int completed = 0;
    struct timeval tv = { timeout_us / 1000000, timeout_us % 1000000 };
    libusb_handle_events_timeout_completed(s_libusb_ctx, &tv, &completed);

    invoke_timers();
    return completed;
}

const egc_platform_backend_t _egc_platform_backend = {
    .usb = {
        .get_device_descriptor = lu_get_device_descriptor,
        .ctrl_transfer_async = lu_ctrl_transfer_async,
        .intr_transfer_async = lu_intr_transfer_async,
    },
#if WITH_BLUETOOTH
    .bt = {
        .device_alloc = lu_bt_device_alloc,
        .device_add = lu_bt_device_add,
        .device_free = lu_bt_device_free,
    },
#endif
    .init = lu_init,
    .alloc_desc = lu_alloc_desc,
    .set_timer = lu_set_timer,
    .report_input = lu_report_input,
    .wait_events = lu_wait_events,
};

#include <assert.h>
#include <ogc/lwp.h>
#include <ogc/machine/processor.h>
#include <ogc/message.h>
#include <ogc/mutex.h>
#include <ogc/system.h>
#include <ogc/usb.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"
#include "utils.h"

/* Maximum number of connected USB controllers. Increase this if needed. */
#define MAX_ACTIVE_DEVICES 2

/* Constants */
#define USB_MAX_DEVICES      32
#define MAX_ACTIVE_TRANSFERS (USB_MAX_DEVICES + 2)

typedef struct {
    void *priv;
    /* VID and PID */
    usb_device_entry dev;
    s32 fd;
    union {
        /* This is essentially the same structure, but the libogc one has a
         * configuration pointer at the end. */
        egc_usb_devdesc_t desc;
        usb_devdesc ogcdesc;
    };
} egc_usb_device_t;

typedef struct {
    /* This must be the first member, since we use it for casting */
    egc_input_device_t pub;
    union {
        egc_usb_device_t usb;
        // TODO: add bluetooth data here
    };
    egc_input_state_t next_input;
    /* This mutex ensures that the input report is always updated in full */
    mutex_t next_input_mutex;
    syswd_t timer_id;
    egc_timer_cb timer_callback;
} wii_device_t;

typedef struct {
    u8 buffer[128] ATTRIBUTE_ALIGN(32);
    struct egc_usb_transfer_t t;
    egc_transfer_cb callback;
} wii_transfer_t;

static egc_device_description_t s_device_descriptions[MAX_ACTIVE_DEVICES];

/* Maximum number of events in the queue. If the handle_events() function is
 * not called often enough, the queue might fill and events will be lost. */
#define WII_MAX_EVENTS 32

typedef enum {
    WII_EVENT_DEVICE_ADDED,
    WII_EVENT_DEVICE_REMOVED,
    WII_EVENT_GOT_INPUT,
} wii_event_e;

typedef struct wii_event_queue_t {
    struct wii_event_t {
        wii_event_e type;
        u8 device_index;
    } ATTRIBUTE_PACKED events[WII_MAX_EVENTS];
    u8 current_index;
} ATTRIBUTE_PACKED wii_event_queue_t;
static wii_event_queue_t s_event_queue;
static mutex_t s_event_queue_mutex;

static wii_device_t s_devices[MAX_ACTIVE_DEVICES] ATTRIBUTE_ALIGN(32);
static wii_transfer_t s_transfers[MAX_ACTIVE_TRANSFERS] ATTRIBUTE_ALIGN(32);
static mqbox_t s_worker_queue;
static lwp_t s_worker_thread;
static egc_event_cb s_event_handler;

static int update_device_list(void);
#define MSG_UPDATE_DEVICES_LIST ((mqmsg_t)update_device_list)
#define MSG_FROM_TRANSFER(t)    ((mqmsg_t)t)

static inline wii_device_t *wii_device_from_input_device(egc_input_device_t *input_device)
{
    return (wii_device_t *)input_device;
}

static inline wii_device_t *get_usb_device_for_dev_id(s32 dev_id)
{
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        if (s_devices[i].pub.connection == EGC_CONNECTION_USB &&
            s_devices[i].usb.dev.device_id == dev_id)
            return &s_devices[i];
    }

    return NULL;
}

static inline wii_device_t *get_free_device_slot(void)
{
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        if (s_devices[i].pub.connection == EGC_CONNECTION_DISCONNECTED)
            return &s_devices[i];
    }

    return NULL;
}

static wii_transfer_t *get_free_transfer(void)
{
    for (int i = 0; i < ARRAY_SIZE(s_transfers); i++) {
        if (!s_transfers[i].t.device)
            return &s_transfers[i];
    }

    return NULL;
}

static inline bool is_usb_device_connected(s32 dev_id)
{
    return get_usb_device_for_dev_id(dev_id) != NULL;
}

/* This is called from an interrupt: do nothing in here, just send the event to
 * the worker thread */
static s32 wii_transfer_cb(s32 result, void *userdata)
{
    wii_transfer_t *transfer = userdata;
    if (result > 0) {
        transfer->t.status = EGC_USB_TRANSFER_STATUS_COMPLETED;
        /* TODO: verify whether this is correct, or figure out how to get the length */
        transfer->t.length = result;
    } else {
        transfer->t.status = EGC_USB_TRANSFER_STATUS_ERROR;
        transfer->t.length = 0;
    }
    MQ_Send(s_worker_queue, MSG_FROM_TRANSFER(transfer), MQ_MSG_NOBLOCK);
    return result;
}

static const egc_usb_transfer_t *wii_ctrl_transfer_async(egc_input_device_t *input_device,
                                                         u8 requesttype, u8 request, u16 value,
                                                         u16 index, void *data, u16 length,
                                                         egc_transfer_cb callback)
{
    wii_device_t *device = (wii_device_t *)input_device;
    wii_transfer_t *t = get_free_transfer();
    if (!t)
        return NULL;

    assert(length <= sizeof(t->buffer));
    if (length > 0) {
        memcpy(t->buffer, data, length);
    } else if (requesttype & EGC_USB_ENDPOINT_IN) {
        length = sizeof(t->buffer);
    }
    t->t.device = input_device;
    t->t.transfer_type = EGC_USB_TRANSFER_CONTROL;
    t->t.endpoint = requesttype;
    t->t.length = length;
    t->t.data = t->buffer;
    t->callback = callback;
    int rc = USB_WriteCtrlMsgAsync(device->usb.fd, requesttype, request, value, index, length,
                                   t->buffer, wii_transfer_cb, t);
    if (rc < 0) {
        t->t.device = NULL; /* Mark as unused */
        t = NULL;
    }
    return &t->t;
}

static const egc_usb_transfer_t *wii_intr_transfer_async(egc_input_device_t *input_device,
                                                         u8 endpoint, void *data, u16 length,
                                                         egc_transfer_cb callback)
{
    wii_device_t *device = (wii_device_t *)input_device;
    wii_transfer_t *t = get_free_transfer();
    if (!t)
        return NULL;

    assert(length <= sizeof(t->buffer));
    if (length > 0) {
        memcpy(t->buffer, data, length);
    } else if (endpoint & EGC_USB_ENDPOINT_IN) {
        length = sizeof(t->buffer);
    }
    t->t.device = input_device;
    t->t.transfer_type = EGC_USB_TRANSFER_INTERRUPT;
    t->t.endpoint = endpoint;
    t->t.length = length;
    t->t.data = t->buffer;
    t->callback = callback;
    int rc = USB_WriteIntrMsgAsync(device->usb.fd, endpoint, length, t->buffer, wii_transfer_cb, t);
    if (rc < 0) {
        t->t.device = NULL; /* Mark as unused */
        return NULL;
    }
    return &t->t;
}

/* This is called from an interrupt: do nothing in here, just send the event to
 * the worker thread */
static void timer_cb(syswd_t timer_id, void *userdata)
{
    wii_device_t *device = userdata;
    MQ_Send(s_worker_queue, &device->timer_id, MQ_MSG_NOBLOCK);
}

static void us_to_timespec(int time_us, struct timespec *ts)
{
    ts->tv_sec = time_us / 1000000;
    ts->tv_nsec = (time_us % 1000000) * 1000;
}

static int wii_set_timer(egc_input_device_t *input_device, int time_us, int repeat_time_us,
                         egc_timer_cb callback)
{
    wii_device_t *device = (wii_device_t *)input_device;
    struct timespec time0, time1;
    int rc;

    if (device->timer_id != 0) {
        SYS_CancelAlarm(device->timer_id);
    } else {
        SYS_CreateAlarm(&device->timer_id);
    }

    us_to_timespec(time_us, &time0);
    if (repeat_time_us > 0) {
        us_to_timespec(repeat_time_us, &time1);
        rc = SYS_SetPeriodicAlarm(device->timer_id, &time0, &time1, timer_cb, device);
    } else {
        rc = SYS_SetAlarm(device->timer_id, &time0, timer_cb, device);
    }
    device->timer_callback = callback;
    return rc == 0 ? 0 : -1;
}

static bool queue_event(egc_event_e type, wii_device_t *device)
{
    if (s_event_queue.current_index >= WII_MAX_EVENTS) {
        LOG_INFO("WII event queue is full!");
        return false;
    }

    u8 device_index = device - s_devices;

    LWP_MutexLock(s_event_queue_mutex);
    struct wii_event_t *e = &s_event_queue.events[s_event_queue.current_index++];
    e->type = type;
    e->device_index = device_index;
    LWP_MutexUnlock(s_event_queue_mutex);
    return true;
}

static void wii_device_free(wii_device_t *device)
{
    if (device->usb.fd >= 0) {
        USB_CloseDevice(&device->usb.fd);
    }

    if (device->next_input_mutex != 0) {
        LWP_MutexDestroy(device->next_input_mutex);
        device->next_input_mutex = 0;
    }

    device->pub.connection = EGC_CONNECTION_DISCONNECTED;
    /* If the desc structure was allocated by us, free it */
    for (int i = 0; i < ARRAY_SIZE(s_device_descriptions); i++) {
        if (device->pub.desc == &s_device_descriptions[i]) {
            memset(&s_device_descriptions[i], 0, sizeof(egc_device_description_t));
            break;
        }
    }
}

/* This is called from an interrupt: do nothing in here, just send the event to
 * the worker thread */
int change_notify_cb(int result, void *userdata)
{
    MQ_Send(s_worker_queue, MSG_UPDATE_DEVICES_LIST, MQ_MSG_NOBLOCK);
    return result;
}

/* This function is called once on initialization, then at every
 * USB_DeviceChangeNotifyAsync reply */
static int update_device_list(void)
{
    wii_device_t *device;
    int ret;

    usb_device_entry devlist[USB_MAX_DEVICES];
    u8 count = 0;
    USB_GetDeviceList(devlist, USB_MAX_DEVICES, USB_CLASS_HID, &count);
    printf("Found %d USB device(s)\n", count);

    /* First look for disconnections */
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        device = &s_devices[i];
        if (device->pub.connection == EGC_CONNECTION_DISCONNECTED)
            continue;

        bool found = false;
        for (int j = 0; j < count; j++) {
            if (device->pub.connection == EGC_CONNECTION_USB &&
                device->usb.dev.device_id == devlist[j].device_id) {
                found = true;
                break;
            }
        }

        /* Oops, it got disconnected */
        if (!found) {
            LOG_INFO("Device with VID: 0x%04" PRIx16 ", PID: 0x%04" PRIx16 ", dev_id: 0x%" PRIx32
                     " got disconnected\n",
                     device->usb.dev.vid, device->usb.dev.pid, device->usb.dev.device_id);

            if (!queue_event(EGC_EVENT_DEVICE_REMOVED, device)) {
                /* We normally mark the device as not valid only after the
                 * client has retrieved the event, but since we couldn't add
                 * the event into the queue, let's mark the device as available
                 * here. */
                wii_device_free(device);
            }
        }
    }

    /* Now look for new connections */
    for (int i = 0; i < count; i++) {
        u16 vid = devlist[i].vid;
        u16 pid = devlist[i].pid;
        s32 dev_id = devlist[i].device_id;
        LOG_INFO("[%d] VID: 0x%04" PRIx16 ", PID: 0x%04" PRIx16 ", dev_id: 0x%" PRIx32 "\n", i, vid,
                 pid, dev_id);

        /* Check if we already have that device (same dev_id) connected */
        if (is_usb_device_connected(dev_id))
            continue;

        /* Get an empty device slot */
        device = get_free_device_slot();
        if (!device)
            break;

        memcpy(&device->usb.dev, &devlist[i], sizeof(device->usb.dev));

        ret = USB_OpenDevice(dev_id, vid, pid, &device->usb.fd);
        if (ret != USB_OK) {
            continue;
        }

        /* We must read the USB device descriptor before interacting with the device */
        ret = USB_GetDeviceDescription(device->usb.fd, &device->usb.ogcdesc);
        if (ret < USB_OK) {
            USB_CloseDevice(&device->usb.fd);
            continue;
        }

        /* We have ownership, populate the device info */
        LWP_MutexInit(&device->next_input_mutex, false);
        device->timer_id = 0;
        device->timer_callback = NULL;
        device->pub.connection = EGC_CONNECTION_USB;

        queue_event(EGC_EVENT_DEVICE_ADDED, device);
    }

    /* Continue watching for device changes */
    USB_DeviceChangeNotifyAsync(USB_CLASS_HID, change_notify_cb, NULL);

    return 0;
}

static void *worker_thread(void *unused)
{
    bool done = false;

    update_device_list();

    while (!done) {
        mqmsg_t message;

        bool ok = MQ_Receive(s_worker_queue, &message, MQ_MSG_BLOCK);
        if (!ok) {
            /* Try again? */
            continue;
        }

        if (message == MSG_UPDATE_DEVICES_LIST) {
            update_device_list();
        } else if (message >= MSG_FROM_TRANSFER(&s_transfers[0]) &&
                   message <= MSG_FROM_TRANSFER(&s_transfers[ARRAY_SIZE(s_transfers) - 1])) {
            /* It's a transfer reply */
            for (int i = 0; i < ARRAY_SIZE(s_transfers); i++) {
                wii_transfer_t *transfer = &s_transfers[i];
                if (message == MSG_FROM_TRANSFER(transfer)) {
                    wii_device_t *device = wii_device_from_input_device(transfer->t.device);
                    if (device->usb.dev.device_id != 0 && transfer->callback) {
                        transfer->callback(&transfer->t);
                    }
                    /* Mark the transfer as unused */
                    transfer->t.status = EGC_USB_TRANSFER_STATUS_UNSET;
                    transfer->t.device = NULL;
                    break;
                }
            }
        } else {
            LOG_INFO("%s:%d\n", __func__, __LINE__);
            /* Find if this is a timer */
            for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
                wii_device_t *device = &s_devices[i];
                if (device->pub.connection == EGC_CONNECTION_DISCONNECTED)
                    continue;
                if (message == &device->timer_id) {
                    LOG_INFO("%s:%d timer on %p\n", __func__, __LINE__, device);
                    bool keep = device->timer_callback(&device->pub);
                    if (!keep) {
                        SYS_RemoveAlarm(device->timer_id);
                        device->timer_id = 0;
                    }
                }
            }
        }
    }
    return NULL;
}

static int wii_init(egc_event_cb event_handler)
{
    int rc;

    s_event_handler = event_handler;

    rc = USB_Initialize();
    if (rc != USB_OK)
        return -1;

    rc = MQ_Init(&s_worker_queue, 10);
    if (rc != MQ_ERROR_SUCCESSFUL)
        return -1;

    rc = LWP_MutexInit(&s_event_queue_mutex, false);
    if (rc != 0)
        return -1;

    for (int i = 0; i < ARRAY_SIZE(s_devices); i++)
        s_devices[i].pub.connection = EGC_CONNECTION_DISCONNECTED;

    /* Give our thread a little more priority than the default */
    u8 priority = (LWP_PRIO_IDLE + LWP_PRIO_HIGHEST) / 4;
    rc = LWP_CreateThread(&s_worker_thread, worker_thread, NULL, NULL, 0, priority);
    return rc == 0 ? 0 : -1;
}

static egc_device_description_t *wii_alloc_desc(egc_input_device_t *device)
{
    for (int i = 0; i < ARRAY_SIZE(s_device_descriptions); i++) {
        if (s_device_descriptions[i].vendor_id == 0) {
            device->desc = &s_device_descriptions[i];
            return &s_device_descriptions[i];
        }
    }

    return NULL;
}

static const egc_usb_devdesc_t *wii_get_device_descriptor(egc_input_device_t *device)
{
    wii_device_t *dev = wii_device_from_input_device(device);
    return &dev->usb.desc;
}

static int wii_set_suspended(egc_input_device_t *input_device, bool suspended)
{
    wii_device_t *device = wii_device_from_input_device(input_device);
    int rc;
    if (suspended) {
        rc = USB_SuspendDevice(device->usb.fd);
    } else {
        rc = USB_ResumeDevice(device->usb.fd);
    }
    return rc == USB_OK ? 0 : -1;
}

static int wii_report_input(egc_input_device_t *input_device, const egc_input_state_t *state)
{
    wii_device_t *device = (wii_device_t *)input_device;
    LWP_MutexLock(device->next_input_mutex);
    memcpy(&device->next_input, state, sizeof(*state));
    LWP_MutexUnlock(device->next_input_mutex);
    queue_event(WII_EVENT_GOT_INPUT, device);
    return 0;
}

static int wii_handle_events()
{
    wii_event_queue_t queue;
    /* Create a copy of the queue to ensure it doesn't get modified while we
     * process it */
    LWP_MutexLock(s_event_queue_mutex);
    memcpy(&queue, &s_event_queue, sizeof(queue));
    s_event_queue.current_index = 0; /* empty the queue */
    LWP_MutexUnlock(s_event_queue_mutex);

    for (int i = 0; i < queue.current_index; i++) {
        struct wii_event_t *event = &queue.events[i];
        wii_device_t *device = &s_devices[event->device_index];

        if (event->type == WII_EVENT_DEVICE_ADDED) {
            int ret = s_event_handler(&device->pub, EGC_EVENT_DEVICE_ADDED, device->usb.dev.vid,
                                      device->usb.dev.pid);
            if (ret != 0) {
                wii_device_free(device);
            }
        } else if (event->type == WII_EVENT_DEVICE_REMOVED) {
            s_event_handler(&device->pub, EGC_EVENT_DEVICE_REMOVED);
            /* Mark this device as not valid */
            wii_device_free(device);
        } else if (event->type == WII_EVENT_GOT_INPUT) {
            LWP_MutexLock(device->next_input_mutex);
            memcpy(&device->pub.state, &device->next_input, sizeof(device->next_input));
            LWP_MutexUnlock(device->next_input_mutex);
        }
    }
    return queue.current_index;
}

const egc_platform_backend_t _egc_platform_backend = {
    .usb = {
        .get_device_descriptor = wii_get_device_descriptor,
        .set_suspended = wii_set_suspended,
        .ctrl_transfer_async = wii_ctrl_transfer_async,
        .intr_transfer_async = wii_intr_transfer_async,
    },
    .init = wii_init,
    .alloc_desc = wii_alloc_desc,
    .set_timer = wii_set_timer,
    .report_input = wii_report_input,
    .handle_events = wii_handle_events,
};

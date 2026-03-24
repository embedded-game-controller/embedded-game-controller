#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ipc.h"
#include "ogc_arm.h"
#include "platform.h"
#include "syscalls.h"
#include "usb.h"
#include "utils.h"

/* Maximum number of connected USB controllers. Increase this if needed. */
#define MAX_ACTIVE_DEVICES 2

/* USBv5 USB_HID IOCTL commands */
#define USBV5_IOCTL_GETVERSION      0
#define USBV5_IOCTL_GETDEVICECHANGE 1
#define USBV5_IOCTL_SHUTDOWN        2
#define USBV5_IOCTL_GETDEVPARAMS    3
#define USBV5_IOCTL_ATTACH          4
#define USBV5_IOCTL_RELEASE         5
#define USBV5_IOCTL_ATTACHFINISH    6
#define USBV5_IOCTL_SETALTERNATE    7
#define USBV5_IOCTL_SUSPEND_RESUME  16
#define USBV5_IOCTL_CANCELENDPOINT  17
#define USBV5_IOCTL_CTRLMSG         18
#define USBV5_IOCTL_INTRMSG         19
#define USBV5_IOCTL_ISOMSG          20 /* Not available in USB_HID */
#define USBV5_IOCTL_BULKMSG         21 /* Not available in USB_HID */

/* Constants */
#define USB_MAX_DEVICES      32
#define MAX_ACTIVE_TRANSFERS 2

/* USBv5 HID message structure */
struct usb_hid_v5_transfer {
    u32 dev_id;
    u32 zero;

    union {
        struct {
            u8 bmRequestType;
            u8 bmRequest;
            u16 wValue;
            u16 wIndex;
        } ctrl;

        struct {
            u32 out;
        } intr;

        u32 data[14];
    };
} ATTRIBUTE_PACKED;
static_assert(sizeof(struct usb_hid_v5_transfer) == 64);

typedef struct {
    void *priv;
    /* VID and PID */
    /* TODO: remove, read them from "desc" */
    u16 vid;
    u16 pid;
    /* Used to communicate with Wii's USB module */
    int host_fd;
    u32 dev_id;
    egc_usb_devdesc_t desc;
} egc_usb_device_t;

typedef struct {
    /* This must be the first member, since we use it for casting */
    egc_device_priv_t priv;
    union {
        egc_usb_device_t usb;
        // TODO: add bluetooth data here
    };
    /* Timer ID (-1 if unset), its address used as timer cookie */
    int timer_id;
    egc_timer_cb timer_callback;
} ogc_device_t;

#define PUB(d) (&(d)->priv.pub)

typedef struct egc_usb_device_entry_t {
    s32 device_id;
    u16 vid;
    u16 pid;
    u32 token;
} egc_usb_device_entry_t;

typedef struct {
    u8 buffer[128] ATTRIBUTE_ALIGN(32);
    areply reply;
    struct egc_usb_transfer_t t;
    egc_transfer_cb callback;
} ogc_transfer_t;

static egc_device_description_t s_device_descriptions[MAX_ACTIVE_DEVICES];

/* Maximum number of events in the queue. If the handle_events() function is
 * not called often enough, the queue might fill and events will be lost. */
#define OGC_MAX_EVENTS 32

static ogc_device_t s_devices[MAX_ACTIVE_DEVICES] ATTRIBUTE_ALIGN(32);
static egc_usb_device_entry_t device_change_devices[USB_MAX_DEVICES] ATTRIBUTE_ALIGN(32);
static ogc_transfer_t s_transfers[MAX_ACTIVE_TRANSFERS] ATTRIBUTE_ALIGN(32);
static int host_fd = -1;
static u32 queue_data[OGC_MAX_EVENTS] ATTRIBUTE_ALIGN(32);
static int queue_id = -1;
static egc_event_cb s_event_handler;

/* Async notification messages */
static areply notification_messages[2] = { 0 };
#define MESSAGE_DEVCHANGE    &notification_messages[0]
#define MESSAGE_ATTACHFINISH &notification_messages[1]

#ifdef __arm__
static u32 saved_cprs;
#endif

static inline void enter_critical_section()
{
#ifdef __arm__
    /* There are no thread synchronization instructions on ARMv5, so we just
     * disable interrupts, which will prevent (among other things) thread
     * scheduling.
     * Code inspired by libn3ds and fastboot3DS */
    saved_cprs = get_cpsr();
    set_cpsr_c(saved_cprs | PSR_I);
#else
#error "enter_critical_section not implemented"
#endif
}

static inline void leave_critical_section()
{
#ifdef __arm__
    set_cpsr_c(saved_cprs);
#else
#error "leave_critical_section not implemented"
#endif
}

static inline ogc_device_t *ogc_device_from_input_device(egc_input_device_t *input_device)
{
    return (ogc_device_t *)input_device;
}

static inline ogc_device_t *get_usb_device_for_dev_id(u32 dev_id)
{
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        if (PUB(&s_devices[i])->connection == EGC_CONNECTION_USB &&
            s_devices[i].usb.dev_id == dev_id)
            return &s_devices[i];
    }

    return NULL;
}

static inline ogc_device_t *get_free_device_slot(void)
{
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        if (PUB(&s_devices[i])->connection == EGC_CONNECTION_DISCONNECTED)
            return &s_devices[i];
    }

    return NULL;
}

static ogc_transfer_t *get_free_transfer(void)
{
    for (int i = 0; i < ARRAY_SIZE(s_transfers); i++) {
        if (!s_transfers[i].t.device)
            return &s_transfers[i];
    }

    return NULL;
}

static inline bool is_usb_device_connected(u32 dev_id)
{
    return get_usb_device_for_dev_id(dev_id) != NULL;
}

static int usb_hid_v5_get_descriptors(int host_fd, u32 dev_id, egc_usb_devdesc_t *udd)
{
    u32 inbuf[8] ATTRIBUTE_ALIGN(32);
    u8 outbuf[96] ATTRIBUTE_ALIGN(32);

    /* Setup buffer */
    memset(inbuf, 0, sizeof(inbuf));
    inbuf[0] = dev_id;
    inbuf[2] = 0;

    /* Get device parameters */
    int rc =
        os_ioctl(host_fd, USBV5_IOCTL_GETDEVPARAMS, inbuf, sizeof(inbuf), outbuf, sizeof(outbuf));
    if (udd)
        memcpy(udd, outbuf, sizeof(*udd));
    return rc;
}

static inline void build_ctrl_transfer(struct usb_hid_v5_transfer *transfer, ogc_transfer_t *t,
                                       u8 bmRequest, u16 wValue, u16 wIndex)
{
    ogc_device_t *device = ogc_device_from_input_device(t->t.device);
    memset(transfer, 0, sizeof(*transfer));
    transfer->dev_id = device->usb.dev_id;
    transfer->ctrl.bmRequestType = t->t.endpoint;
    transfer->ctrl.bmRequest = bmRequest;
    transfer->ctrl.wValue = wValue;
    transfer->ctrl.wIndex = wIndex;
}

static inline int usb_hid_v5_ctrl_transfer_async(ogc_transfer_t *t, u8 bmRequest, u16 wValue,
                                                 u16 wIndex, int queue_id)
{
    struct usb_hid_v5_transfer transfer ATTRIBUTE_ALIGN(32);
    ogc_device_t *device = ogc_device_from_input_device(t->t.device);
    ioctlv vectors[2];
    int out = !(t->t.endpoint & EGC_USB_ENDPOINT_IN);

    build_ctrl_transfer(&transfer, t, bmRequest, wValue, wIndex);

    vectors[0].data = &transfer;
    vectors[0].len = sizeof(transfer);
    vectors[1].data = t->t.data;
    vectors[1].len = t->t.length;

    return os_ioctlv_async(device->usb.host_fd, USBV5_IOCTL_CTRLMSG, 1 + out, 1 - out, vectors,
                           queue_id, &t->reply);
}

static inline int usb_hid_v5_intr_transfer_async(ogc_transfer_t *t, int queue_id)
{
    struct usb_hid_v5_transfer transfer ATTRIBUTE_ALIGN(32);
    ogc_device_t *device = ogc_device_from_input_device(t->t.device);
    ioctlv vectors[2];

    int out = !(t->t.endpoint & EGC_USB_ENDPOINT_IN);
    memset(&transfer, 0, sizeof(transfer));
    transfer.dev_id = device->usb.dev_id;
    transfer.intr.out = out;

    vectors[0].data = &transfer;
    vectors[0].len = sizeof(transfer);
    vectors[1].data = t->t.data;
    vectors[1].len = t->t.length;

    return os_ioctlv_async(device->usb.host_fd, USBV5_IOCTL_INTRMSG, 1 + out, 1 - out, vectors,
                           queue_id, &t->reply);
}

static int usb_hid_v5_attach(int host_fd, u32 dev_id)
{
    u32 buf[8] ATTRIBUTE_ALIGN(32);

    memset(buf, 0, sizeof(buf));
    buf[0] = dev_id;

    return os_ioctl(host_fd, USBV5_IOCTL_ATTACH, buf, sizeof(buf), NULL, 0);
}

static int usb_hid_v5_release(int host_fd, u32 dev_id)
{
    u32 buf[8] ATTRIBUTE_ALIGN(32);

    memset(buf, 0, sizeof(buf));
    buf[0] = dev_id;

    return os_ioctl(host_fd, USBV5_IOCTL_RELEASE, buf, sizeof(buf), NULL, 0);
}

static int usb_hid_v5_suspend_resume(int host_fd, int dev_id, int resumed)
{
    u32 buf[8] ATTRIBUTE_ALIGN(32);

    memset(buf, 0, sizeof(buf));
    buf[0] = dev_id;
    buf[2] = resumed;

    return os_ioctl(host_fd, USBV5_IOCTL_SUSPEND_RESUME, buf, sizeof(buf), NULL, 0);
}

static const egc_usb_transfer_t *ogc_ctrl_transfer_async(egc_input_device_t *device, u8 requesttype,
                                                         u8 request, u16 value, u16 index,
                                                         void *data, u16 length,
                                                         egc_transfer_cb callback)
{
    ogc_transfer_t *t = get_free_transfer();
    if (!t)
        return NULL;

    assert(length <= sizeof(t->buffer));
    if (length > 0) {
        memcpy(t->buffer, data, length);
    } else if (requesttype & EGC_USB_ENDPOINT_IN) {
        length = sizeof(t->buffer);
    }
    t->t.device = device;
    t->t.transfer_type = EGC_USB_TRANSFER_CONTROL;
    t->t.endpoint = requesttype;
    t->t.length = length;
    t->t.data = t->buffer;
    t->callback = callback;
    int rc = usb_hid_v5_ctrl_transfer_async(t, request, value, index, queue_id);
    if (rc < 0) {
        t->t.device = NULL; /* Mark as unused */
        t = NULL;
    }
    return &t->t;
}

static const egc_usb_transfer_t *ogc_intr_transfer_async(egc_input_device_t *device, u8 endpoint,
                                                         void *data, u16 length,
                                                         egc_transfer_cb callback)
{
    ogc_transfer_t *t = get_free_transfer();
    if (!t)
        return NULL;

    assert(length <= sizeof(t->buffer));
    if (length > 0) {
        memcpy(t->buffer, data, length);
    } else if (endpoint & EGC_USB_ENDPOINT_IN) {
        length = sizeof(t->buffer);
    }
    t->t.device = device;
    t->t.transfer_type = EGC_USB_TRANSFER_INTERRUPT;
    t->t.endpoint = endpoint;
    t->t.length = length;
    t->t.data = t->buffer;
    t->callback = callback;
    int rc = usb_hid_v5_intr_transfer_async(t, queue_id);
    if (rc < 0) {
        t->t.device = NULL; /* Mark as unused */
        return NULL;
    }
    return &t->t;
}

static int ogc_set_timer(egc_input_device_t *input_device, int time_us, int repeat_time_us,
                         egc_timer_cb callback)
{
    ogc_device_t *device = (ogc_device_t *)input_device;
    int rc;

    if (device->timer_id > 0) {
        os_stop_timer(device->timer_id);
        rc = os_restart_timer(device->timer_id, time_us, repeat_time_us);
    } else {
        device->timer_id =
            os_create_timer(time_us, repeat_time_us, queue_id, (s32)&device->timer_id);
        rc = device->timer_id;
    }
    device->timer_callback = callback;
    return rc;
}

static int ogc_report_input(egc_input_device_t *device, const egc_input_state_t *state)
{
    enter_critical_section();
    memcpy(&device->state, state, sizeof(*state));
    leave_critical_section();
    return 0;
}

static bool report_event(egc_event_e type, ogc_device_t *device)
{
    bool ok = false;
    if (type == EGC_EVENT_DEVICE_ADDED) {
        int ret = s_event_handler(PUB(device), type, device->usb.vid, device->usb.pid);
        ok = (ret == 0);
    } else if (type == EGC_EVENT_DEVICE_REMOVED) {
        s_event_handler(PUB(device), type);
        ok = true;
    }
    return ok;
}

static void ogc_device_free(ogc_device_t *device)
{
    PUB(device)->connection = EGC_CONNECTION_DISCONNECTED;
    if (device->timer_id >= 0) {
        os_destroy_timer(device->timer_id);
        device->timer_id = -1;
    }
    /* If the desc structure was allocated by us, free it */
    for (int i = 0; i < ARRAY_SIZE(s_device_descriptions); i++) {
        if (PUB(device)->desc == &s_device_descriptions[i]) {
            memset(&s_device_descriptions[i], 0, sizeof(egc_device_description_t));
            break;
        }
    }
}

static void handle_device_change_reply(int host_fd, areply *reply)
{
    ogc_device_t *device;
    u16 vid, pid;
    u32 dev_id;
    int ret;
    bool found;

    LOG_DEBUG("Device change, #Attached devices: %" PRId32 "\n", reply->result);

    if (reply->result < 0)
        return;

    /* First look for disconnections */
    for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
        device = &s_devices[i];
        if (PUB(device)->connection == EGC_CONNECTION_DISCONNECTED)
            continue;

        found = false;
        for (int j = 0; j < reply->result; j++) {
            if (PUB(device)->connection == EGC_CONNECTION_USB &&
                device->usb.dev_id == device_change_devices[j].device_id) {
                found = true;
                break;
            }
        }

        /* Oops, it got disconnected */
        if (!found) {
            LOG_DEBUG("Device with VID: 0x%04" PRIx16 ", PID: 0x%04" PRIx16 ", dev_id: 0x%" PRIx32
                      " got disconnected\n",
                      device->usb.vid, device->usb.pid, device->usb.dev_id);

            report_event(EGC_EVENT_DEVICE_REMOVED, device);
            ogc_device_free(device);
        }
    }

    /* Now look for new connections */
    for (int i = 0; i < reply->result; i++) {
        vid = device_change_devices[i].vid;
        pid = device_change_devices[i].pid;
        dev_id = device_change_devices[i].device_id;
        LOG_DEBUG("[%d] VID: 0x%04" PRIx16 ", PID: 0x%04" PRIx16 ", dev_id: 0x%" PRIx32 "\n", i,
                  vid, pid, dev_id);

        /* Check if we already have that device (same dev_id) connected */
        if (is_usb_device_connected(dev_id))
            continue;

        /* Get an empty device slot */
        device = get_free_device_slot();
        if (!device)
            break;

        /* Now we can attach it to take ownership! */
        ret = usb_hid_v5_attach(host_fd, dev_id);
        if (ret != IOS_OK)
            continue;

        /* We must resume the USB device before interacting with it */
        ret = usb_hid_v5_suspend_resume(host_fd, dev_id, 1);
        if (ret != IOS_OK) {
            usb_hid_v5_release(host_fd, dev_id);
            continue;
        }

        /* We must read the USB device descriptor before interacting with the device */
        ret = usb_hid_v5_get_descriptors(host_fd, dev_id, &device->usb.desc);
        if (ret != IOS_OK) {
            usb_hid_v5_release(host_fd, dev_id);
            continue;
        }

        /* We have ownership, populate the device info */
        device->usb.vid = vid;
        device->usb.pid = pid;
        device->usb.host_fd = host_fd;
        device->usb.dev_id = dev_id;
        device->timer_id = -1;
        device->timer_callback = NULL;
        PUB(device)->connection = EGC_CONNECTION_USB;

        if (!report_event(EGC_EVENT_DEVICE_ADDED, device)) {
            usb_hid_v5_release(host_fd, device->usb.dev_id);
            PUB(device)->connection = EGC_CONNECTION_DISCONNECTED;
        }
    }

    ret = os_ioctl_async(host_fd, USBV5_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0, queue_id,
                         MESSAGE_ATTACHFINISH);
    LOG_DEBUG("ioctl(ATTACHFINISH): %d\n", ret);
}

static int usb_hid_init()
{
    u32 ver[8] ATTRIBUTE_ALIGN(32);
    int ret;

    LOG_DEBUG("usb_hid_init\n");

    for (int i = 0; i < ARRAY_SIZE(s_devices); i++)
        PUB(&s_devices[i])->connection = EGC_CONNECTION_DISCONNECTED;

    /* USB_HID supports 16 handles, libogc uses handle 0, so we use handle 15...*/
    ret = os_open("/dev/usb/hid", 15);
    if (ret < 0)
        return ret;
    host_fd = ret;

    ret = os_ioctl(host_fd, USBV5_IOCTL_GETVERSION, NULL, 0, ver, sizeof(ver));
    if (ret < 0)
        return ret;

    /* We only support USBv5 for now */
    if (ver[0] != 0x50001)
        return IOS_EINVAL;

    ret = os_ioctl_async(host_fd, USBV5_IOCTL_GETDEVICECHANGE, NULL, 0, device_change_devices,
                         sizeof(device_change_devices), queue_id, MESSAGE_DEVCHANGE);

    return ret;
}

static int process_queue(u32 timeout_us)
{
    ogc_device_t *device;
    int ret;

    s32 timer_id = -1;
    u32 queue_flags;
    if (timeout_us) {
        timer_id = os_create_timer(timeout_us, 0, queue_id, (s32)&timer_id);
        queue_flags = IOS_MESSAGE_BLOCK;
    } else {
        queue_flags = IOS_MESSAGE_NOBLOCK;
    }

    while (1) {
        void *message;

        /* Wait for a message from USB devices */
        ret = os_message_queue_receive(queue_id, &message, queue_flags);
        if (ret != IOS_OK)
            break;

        if (message == MESSAGE_DEVCHANGE) {
            handle_device_change_reply(host_fd, message);
        } else if (message == MESSAGE_ATTACHFINISH) {
            ret =
                os_ioctl_async(host_fd, USBV5_IOCTL_GETDEVICECHANGE, NULL, 0, device_change_devices,
                               sizeof(device_change_devices), queue_id, MESSAGE_DEVCHANGE);
        } else if (message == &timer_id) {
            /* Timer triggered, give up */
            break;
        } else if ((areply *)message >= &s_transfers[0].reply &&
                   (areply *)message <= &s_transfers[ARRAY_SIZE(s_transfers) - 1].reply) {
            /* It's a transfer reply */
            for (int i = 0; i < ARRAY_SIZE(s_transfers); i++) {
                ogc_transfer_t *transfer = &s_transfers[i];
                if ((areply *)message == &transfer->reply) {
                    ogc_device_t *device = ogc_device_from_input_device(transfer->t.device);
                    if (device->usb.dev_id != 0 && transfer->callback) {
                        if (transfer->reply.result >= 0) {
                            transfer->t.length = transfer->reply.result;
                            transfer->t.status = EGC_USB_TRANSFER_STATUS_COMPLETED;
                        } else {
                            transfer->t.length = 0;
                            transfer->t.status = EGC_USB_TRANSFER_STATUS_ERROR;
                        }
                        transfer->callback(&transfer->t);
                    }
                    /* Mark the transfer as unused */
                    transfer->t.status = EGC_USB_TRANSFER_STATUS_UNSET;
                    transfer->t.device = NULL;
                    break;
                }
            }
        } else {
            /* Find if this is a timer */
            for (int i = 0; i < ARRAY_SIZE(s_devices); i++) {
                device = &s_devices[i];
                if (PUB(device)->connection == EGC_CONNECTION_DISCONNECTED)
                    continue;
                if (message == &device->timer_id) {
                    bool keep = device->timer_callback(PUB(device));
                    if (!keep) {
                        os_destroy_timer(device->timer_id);
                        device->timer_id = -1;
                    }
                }
            }
        }

        /* If we got a message and have the timer set, don't iterate the loop again */
        if (timer_id != -1)
            break;
    }

    if (timer_id != -1) {
        os_destroy_timer(timer_id);
    }
    return 0;
}

static int ogc_init(egc_event_cb event_handler)
{
    int ret;

    s_event_handler = event_handler;

    ret = os_message_queue_create(queue_data, ARRAY_SIZE(queue_data));
    if (ret < 0)
        return ret;
    queue_id = ret;

    ret = usb_hid_init();
    if (ret < 0)
        return ret;

    return 0;
}

static egc_device_description_t *ogc_alloc_desc(egc_input_device_t *device)
{
    for (int i = 0; i < ARRAY_SIZE(s_device_descriptions); i++) {
        if (s_device_descriptions[i].vendor_id == 0) {
            device->desc = &s_device_descriptions[i];
            return &s_device_descriptions[i];
        }
    }

    return NULL;
}

static const egc_usb_devdesc_t *ogc_get_device_descriptor(egc_input_device_t *device)
{
    ogc_device_t *dev = ogc_device_from_input_device(device);
    return &dev->usb.desc;
}

static int ogc_set_suspended(egc_input_device_t *input_device, bool suspended)
{
    ogc_device_t *device = ogc_device_from_input_device(input_device);
    return usb_hid_v5_suspend_resume(device->usb.host_fd, device->usb.dev_id, !suspended);
}

static int ogc_wait_events(u32 timeout_us)
{
    return process_queue(timeout_us);
}

const egc_platform_backend_t _egc_platform_backend = {
    .usb = {
        .get_device_descriptor = ogc_get_device_descriptor,
        .set_suspended = ogc_set_suspended,
        .ctrl_transfer_async = ogc_ctrl_transfer_async,
        .intr_transfer_async = ogc_intr_transfer_async,
    },
    .init = ogc_init,
    .alloc_desc = ogc_alloc_desc,
    .set_timer = ogc_set_timer,
    .report_input = ogc_report_input,
    .wait_events = ogc_wait_events,
};

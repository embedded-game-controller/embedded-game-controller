#if WITH_BLUETOOTH

#include <bt-embedded/client.h>
#include <bt-embedded/hci.h>
#include <bt-embedded/l2cap.h>
#include <bt-embedded/services/hid.h>
#include <bt-embedded/services/sdp.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "bluetooth.h"
#include "bt_backend.h"
#include "egc.h"
#include "platform.h"
#include "utils.h"

#define MAX_INQUIRY_RESPONSES 4

#ifndef EGC_BT_MAX_DEVICES
#define EGC_BT_MAX_DEVICES 4
#endif

#define BD_ADDR_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define BD_ADDR_DATA(b)                                                                            \
    (b)->bytes[5], (b)->bytes[4], (b)->bytes[3], (b)->bytes[2], (b)->bytes[1], (b)->bytes[0]

enum {
    EGC_BT_STATE_UNUSED = 0,
    EGC_BT_STATE_INQUIRY,
    EGC_BT_STATE_PROBING,
    EGC_BT_STATE_CONNECTING,
    EGC_BT_STATE_CONNECTED,
};

typedef struct {
    egc_input_device_t *input_device;
    uint8_t state;
    union {
        /* Contents depend on the value of "state" */
        struct {
            BteL2cap *hid_ctrl;
            BteL2cap *hid_intr;
        } connected;

        struct {
            BteSdpClient *sdp;
        } probing;

        struct {
            BteBdAddr address;
        } inquiry;
    } s;
} egc_bt_device_t;

static egc_bt_device_t s_bt_devices[EGC_BT_MAX_DEVICES];
static BteClient *s_client;
static bool s_inquiry_requested = false;
static bool s_hci_ready = false;
static BtePacketType s_packet_types;

static const BteBdAddr *device_get_address(const egc_bt_device_t *device)
{
    if (device->state == EGC_BT_STATE_INQUIRY) {
        return &device->s.inquiry.address;
    } else if (device->state == EGC_BT_STATE_PROBING) {
        BteL2cap *l2cap = bte_sdp_client_get_l2cap(device->s.probing.sdp);
        return bte_l2cap_get_address(l2cap);
    } else if (device->state == EGC_BT_STATE_CONNECTING ||
               device->state == EGC_BT_STATE_CONNECTED) {
        return bte_l2cap_get_address(device->s.connected.hid_ctrl);
    }

    return NULL;
}

static egc_bt_device_t *device_by_address(const BteBdAddr *address)
{
    for (int i = 0; i < ARRAY_SIZE(s_bt_devices); i++) {
        egc_bt_device_t *d = &s_bt_devices[i];
        const BteBdAddr *dev_address = device_get_address(d);
        if (dev_address && memcmp(address, dev_address, 6) == 0) {
            return d;
        }
    }

    return NULL;
}

static egc_bt_device_t *egc_bt_device_from_input(egc_input_device_t *input_device)
{
    for (int i = 0; i < ARRAY_SIZE(s_bt_devices); i++) {
        egc_bt_device_t *d = &s_bt_devices[i];
        if (d->input_device == input_device)
            return d;
    }
    return NULL;
}

static egc_bt_device_t *bt_device_alloc(const BteBdAddr *address)
{
    for (int i = 0; i < ARRAY_SIZE(s_bt_devices); i++) {
        egc_bt_device_t *d = &s_bt_devices[i];
        if (d->state == EGC_BT_STATE_UNUSED) {
            d->state = EGC_BT_STATE_INQUIRY;
            d->s.inquiry.address = *address;
            return d;
        }
    }
    return NULL;
}

static void bt_device_free(egc_bt_device_t *device)
{
    if (device->state == EGC_BT_STATE_PROBING) {
        bte_sdp_client_unref(device->s.probing.sdp);
    } else if (device->state == EGC_BT_STATE_CONNECTING ||
               device->state == EGC_BT_STATE_CONNECTED) {
        bte_l2cap_unref(device->s.connected.hid_ctrl);
        if (device->s.connected.hid_intr) {
            bte_l2cap_unref(device->s.connected.hid_intr);
        }
    }

    if (device->input_device) {
        _egc_platform_backend.bt.device_free(device->input_device);
    }

    memset(device, 0, sizeof(*device));
}

static void hid_intr_message_received_cb(BteL2cap *l2cap, BteBufferReader *reader, void *userdata)
{
    egc_bt_device_t *device = userdata;
    u16 len = 0;
    u8 *data = bte_buffer_reader_read_max(reader, &len);
    if (len == 0)
        return;

    u8 transfer_type = data[0] & BTE_HID_HDR_TRANS_MASK;
    if (transfer_type == BTE_HID_TRANS_DATA) {
        _egc_input_device_intr_data_received(device->input_device, data + 1, len - 1);
    }
}

static void hid_intr_disconnected_cb(BteL2cap *l2cap, uint8_t reason, void *userdata)
{
    egc_bt_device_t *device = userdata;
    EGC_DEBUG("");
    bt_device_free(device);
}

static void hid_intr_connect_cb(BteL2cap *l2cap, const BteL2capNewConfiguredReply *reply,
                                void *userdata)
{
    egc_bt_device_t *device = userdata;

    if (reply->result != BTE_L2CAP_INFO_RESP_RES_OK) {
        EGC_DEBUG("Failed %d", reply->result);
        bt_device_free(device);
        return;
    }

    device->state = EGC_BT_STATE_CONNECTED;
    device->s.connected.hid_intr = bte_l2cap_ref(l2cap);

    /* The device is ready to be used, hand it over to the platform backend */
    int rc = _egc_platform_backend.bt.device_add(device->input_device);
    if (rc < 0) {
        EGC_DEBUG("Device addition failed, rc = %d", rc);
        bt_device_free(device);
        return;
    }

    bte_l2cap_set_userdata(l2cap, device);
    bte_l2cap_on_message_received(device->s.connected.hid_intr, hid_intr_message_received_cb);
    bte_l2cap_on_acl_disconnected(l2cap, hid_intr_disconnected_cb);
}

static void hid_ctrl_connect_cb(BteL2cap *l2cap, const BteL2capNewConfiguredReply *reply,
                                void *userdata)
{
    egc_bt_device_t *device = userdata;

    if (reply->result != BTE_L2CAP_INFO_RESP_RES_OK) {
        EGC_DEBUG("Failed %d", reply->result);
        bt_device_free(device);
        return;
    }

    /* Save the sdp handle, since we are overwriting the union */
    BteSdpClient *sdp = device->s.probing.sdp;
    device->state = EGC_BT_STATE_CONNECTING;
    device->s.connected.hid_ctrl = bte_l2cap_ref(l2cap);
    device->s.connected.hid_intr = NULL;
    bte_sdp_client_unref(sdp);

    bte_l2cap_set_userdata(l2cap, device);

    const BteBdAddr *address = device_get_address(device);
    bte_l2cap_new_configured(s_client, address, BTE_L2CAP_PSM_HID_INTR, NULL,
                             BTE_L2CAP_CONNECT_FLAG_NONE, NULL, hid_intr_connect_cb, device);
}

static bool parse_did_attribute(egc_bt_device_desc_t *desc, u16 attr_id, BteSdpDeReader *reader)
{
    switch (attr_id) {
    case BTE_SDP_ATTR_ID_DID_VENDOR_ID:
        desc->vendor_id = bte_sdp_de_reader_read_uint16(reader);
        break;
    case BTE_SDP_ATTR_ID_DID_PRODUCT_ID:
        desc->product_id = bte_sdp_de_reader_read_uint16(reader);
        break;
    }
    return true;
}

static bool parse_sdp_reply(egc_bt_device_desc_t *bt_device_desc, const uint8_t *de)
{
    BteSdpDeReader reader;
    bte_sdp_de_reader_init(&reader, de);

    if (!bte_sdp_de_reader_enter(&reader))
        return false;

    /* Iterate the list of services */
    while (bte_sdp_de_reader_next(&reader)) {
        if (!bte_sdp_de_reader_enter(&reader))
            continue;

        bool is_did_service = false;

        /* Iterate the list of attributes within a service */
        while (bte_sdp_de_reader_next(&reader)) {
            u16 attr_id = bte_sdp_de_reader_read_uuid16(&reader);
            /* Position the reader on the attribute value */
            if (!bte_sdp_de_reader_next(&reader))
                return false;

            if (attr_id == BTE_SDP_ATTR_ID_SRV_CLS_ID_LIST) {
                if (!bte_sdp_de_reader_enter(&reader))
                    return false;
                while (bte_sdp_de_reader_next(&reader)) {
                    u16 service_id = bte_sdp_de_reader_read_uuid16(&reader);
                    if (service_id == BTE_SDP_SRV_CLASS_PNP_INFO) {
                        is_did_service = true;
                    }
                }
                if (!bte_sdp_de_reader_leave(&reader))
                    return false;
            } else if (is_did_service) {
                if (!parse_did_attribute(bt_device_desc, attr_id, &reader))
                    return false;
            }
        }
        if (!bte_sdp_de_reader_leave(&reader))
            return false;
    }
    return true;
}

static void sdp_service_search_attr_cb(BteSdpClient *sdp, const BteSdpServiceAttrReply *reply,
                                       void *userdata)
{
    egc_bt_device_t *device = userdata;
    if (reply->error_code != 0) {
        EGC_DEBUG("Failed %d", reply->error_code);
        bt_device_free(device);
        return;
    }

    egc_bt_device_desc_t desc = {};
    bool ok = parse_sdp_reply(&desc, reply->attr_list_de);
    if (!ok) {
        EGC_DEBUG("Invalid SDP data");
        bt_device_free(device);
        return;
    }

    EGC_DEBUG("VID %04x, PID %04x", desc.vendor_id, desc.product_id);
    /* Allocate the device and initialize it, but don't invoke the driver yet. */
    egc_input_device_t *input_device = device->input_device =
        _egc_platform_backend.bt.device_alloc(&desc);
    if (!input_device) {
        EGC_DEBUG("Couldn't allocate device");
        bt_device_free(device);
        return;
    }

    input_device->connection = EGC_CONNECTION_BT;

    const BteBdAddr *address = device_get_address(device);
    bte_l2cap_new_configured(s_client, address, BTE_L2CAP_PSM_HID_CTRL, NULL,
                             BTE_L2CAP_CONNECT_FLAG_NONE, NULL, hid_ctrl_connect_cb, device);
}

static void sdp_connect_cb(BteL2cap *l2cap, const BteL2capNewConfiguredReply *reply, void *userdata)
{
    egc_bt_device_t *device = userdata;
    if (reply->result != BTE_L2CAP_INFO_RESP_RES_OK) {
        EGC_DEBUG("Failed %d", reply->result);
        bt_device_free(device);
        return;
    }

    device->state = EGC_BT_STATE_PROBING;
    BteSdpClient *sdp = device->s.probing.sdp = bte_sdp_client_new(l2cap);
    /* clang-format off */
    u8 pattern[32];
    bte_sdp_de_write(pattern, sizeof(pattern),
                     BTE_SDP_DE_TYPE_SEQUENCE,
                     BTE_SDP_DE_TYPE_UUID16, BTE_SDP_PROTO_L2CAP,
                     BTE_SDP_DE_END);
    u8 id_list[20];
    bte_sdp_de_write(id_list, sizeof(id_list),
                     BTE_SDP_DE_TYPE_SEQUENCE,
                     BTE_SDP_DE_TYPE_UINT32, 0x0000ffff,
                     BTE_SDP_DE_END);
    /* clang-format on */

    bool ok = bte_sdp_service_search_attr_req(sdp, pattern, 1000, id_list,
                                              sdp_service_search_attr_cb, device);
    if (!ok) {
        EGC_DEBUG("Could not issue SDP request");
        bt_device_free(device);
        return;
    }
}

static void inquiry_cb(BteHci *hci, const BteHciInquiryReply *reply, void *)
{
    for (int i = 0; i < reply->num_responses; i++) {
        const BteHciInquiryResponse *r = &reply->responses[i];
        EGC_DEBUG("Device " BD_ADDR_FMT ", service class %d, major %d, minor %d",
                  BD_ADDR_DATA(&r->address), bte_cod_get_service_class(r->class_of_device),
                  bte_cod_get_major_dev_class(r->class_of_device),
                  bte_cod_get_minor_dev_class(r->class_of_device));
        if (bte_cod_get_major_dev_class(r->class_of_device) != BTE_COD_MAJOR_DEV_CLASS_PERIPH) {
            continue;
        }

        if (device_by_address(&r->address)) {
            /* We are already handling this device */
            continue;
        }

        egc_bt_device_t *device = bt_device_alloc(&r->address);
        if (!device) {
            EGC_DEBUG("No more BT slots available");
            continue;
        }

        BteHciConnectParams params;
        params.packet_type = s_packet_types;
        params.clock_offset = r->clock_offset;
        params.page_scan_rep_mode = r->page_scan_rep_mode;
        params.allow_role_switch = true;
        bte_l2cap_new_configured(s_client, &r->address, BTE_L2CAP_PSM_SDP, &params,
                                 BTE_L2CAP_CONNECT_FLAG_NONE, NULL, sdp_connect_cb, device);
    }
}

static void initialized_cb(BteHci *hci, bool success, void *)
{
    s_hci_ready = success;
    EGC_DEBUG("success %d, requested %d", success, s_inquiry_requested);
    s_packet_types = bte_hci_packet_types_from_features(bte_hci_get_supported_features(hci));
    if (s_inquiry_requested) {
        egc_bt_start_scan();
    }
}

int _egc_bt_initialize()
{
    s_client = bte_client_new();
    if (!s_client)
        return -ENOENT;

    BteHci *hci = bte_hci_get(s_client);
    bte_hci_on_initialized(hci, initialized_cb, NULL);
    return 0;
}

const egc_usb_transfer_t *_egc_bt_ctrl_transfer(egc_input_device_t *input_device, u8 requesttype,
                                                u8 request, u16 value, u16 index, void *data,
                                                u16 len, egc_transfer_cb callback)
{
    return NULL; /* TODO */
}

int _egc_bt_intr_transfer(egc_input_device_t *input_device, void *data, u16 len)
{
    egc_bt_device_t *device = egc_bt_device_from_input(input_device);
    if (!device || device->state != EGC_BT_STATE_CONNECTED)
        return -1;

    BteBufferWriter writer;
    bool ok = bte_l2cap_create_message(device->s.connected.hid_intr, &writer, len + 1);
    if (!ok)
        return -1;

    uint8_t *buf = bte_buffer_writer_ptr_n(&writer, len + 1);
    buf[0] = BTE_HID_TRANS_DATA | BTE_HID_REP_TYPE_OUTPUT;
    memcpy(buf + 1, data, len);
    int rc = bte_l2cap_send_message(device->s.connected.hid_intr, bte_buffer_writer_end(&writer));
    return rc;
}

int egc_bt_start_scan()
{
    if (!s_hci_ready) {
        s_inquiry_requested = true;
        return 0;
    }

    BteHci *hci = bte_hci_get(s_client);
    bte_hci_periodic_inquiry(hci, 4, 5, BTE_LAP_GIAC, 3, 0, NULL, inquiry_cb, NULL);
    return 0;
}

int egc_bt_stop_scan()
{
    if (!s_hci_ready) {
        s_inquiry_requested = false;
        return 0;
    }

    bte_hci_exit_periodic_inquiry(bte_hci_get(s_client), NULL, NULL);
    return 0;
}

#else /* !WITH_BLUETOOTH */

#include <errno.h>

#include "egc.h"

int egc_bt_start_scan()
{
    return -ENOSYS;
}

int egc_bt_stop_scan()
{
    return -ENOSYS;
}

#endif /* WITH_BLUETOOTH */

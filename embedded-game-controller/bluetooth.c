#include <errno.h>
#include <stddef.h>

#include "bluetooth.h"

#if WITH_BLUETOOTH

#include <bt-embedded/client.h>
#include <bt-embedded/hci.h>
#include <bt-embedded/l2cap.h>
#include <bt-embedded/l2cap_server.h>
#include <bt-embedded/services/hid.h>
#include <bt-embedded/services/sdp.h>
#include <stdio.h>
#include <string.h>

#include "bt_backend.h"
#include "egc.h"
#include "platform.h"
#include "utils.h"

#define MAX_INQUIRY_RESPONSES 4

#ifndef EGC_BT_MAX_DEVICES
#define EGC_BT_MAX_DEVICES 7
#endif

enum {
    EGC_BT_STATE_UNUSED = 0,
    EGC_BT_STATE_INQUIRY,
    EGC_BT_STATE_INCOMING,
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

/* We can have at most these initialization callbacks
 * - Read stored link keys
 * - Starting the inquiry
 * - Setting up the L2CAP server
 * - Platform backend registering a vendor callback (Wii)
 */
#define MAX_READY_CB 4

static egc_bt_device_t s_bt_devices[EGC_BT_MAX_DEVICES];
static BteClient *s_client;
static bool s_hci_ready = false;
static BtePacketType s_packet_types;
static BteL2capServer *s_l2cap_server_hid_ctrl;
static BteL2capServer *s_l2cap_server_hid_intr;

static egc_bt_stored_link_key_t *s_stored_link_keys;
static u8 s_stored_link_keys_max;
static u8 s_stored_link_keys_oldest;         /* Assume it's the one in slot 0, initially */
static u8 s_stored_link_keys_max_ctrl = 255; /* Controller limit, read at init */
static_assert(sizeof(egc_bt_stored_link_key_t) == sizeof(BteHciStoredLinkKey));

static EgcBtConnectionCb s_connection_cb;
static EgcBtAuthDataRequestedCb s_link_key_requested_cb;
static EgcBtAuthDataRequestedCb s_pin_code_requested_cb;
static EgcBtLinkKeyReceivedCb s_link_key_received_cb;

static egc_bt_initialized_cb s_ready_callbacks[MAX_READY_CB];
static u8 s_ready_callbacks_count = 0;

static bool mem_is_zero(const void *data, size_t size)
{
    const u8 *bytes = data;
    bool is_zero = true;
    for (int i = 0; i < size; i++) {
        if (bytes[i] != 0) {
            is_zero = false;
            break;
        }
    }
    return is_zero;
}

static const BteBdAddr *device_get_address(const egc_bt_device_t *device)
{
    if (device->state == EGC_BT_STATE_INQUIRY) {
        return &device->s.inquiry.address;
    } else if (device->state == EGC_BT_STATE_PROBING) {
        BteL2cap *l2cap = bte_sdp_client_get_l2cap(device->s.probing.sdp);
        return bte_l2cap_get_address(l2cap);
    } else if (device->state == EGC_BT_STATE_CONNECTING ||
               device->state == EGC_BT_STATE_CONNECTED || device->state == EGC_BT_STATE_INCOMING) {
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
               device->state == EGC_BT_STATE_CONNECTED || device->state == EGC_BT_STATE_INCOMING) {
        if (device->state == EGC_BT_STATE_INCOMING) {
            /* The SDP channel is stored in the userdata of the HID ctrl channel */
            BteSdpClient *sdp = bte_l2cap_get_userdata(device->s.connected.hid_ctrl);
            if (sdp) {
                bte_sdp_client_unref(sdp);
            }
        }
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

static void hid_disconnected_cb(BteL2cap *l2cap, uint8_t reason, void *userdata)
{
    egc_bt_device_t *device = userdata;
    EGC_DEBUG("");
    bt_device_free(device);
}

static void watch_connection_status(egc_bt_device_t *device, BteL2cap *l2cap)
{
    bte_l2cap_set_userdata(l2cap, device);
    bte_l2cap_on_disconnected(l2cap, hid_disconnected_cb);
    bte_l2cap_on_acl_disconnected(l2cap, hid_disconnected_cb);
}

static void device_set_connected(egc_bt_device_t *device)
{
    device->state = EGC_BT_STATE_CONNECTED;
    /* The device is ready to be used, hand it over to the platform backend */
    int rc = _egc_platform_backend.bt.device_add(device->input_device);
    if (rc < 0) {
        EGC_DEBUG("Device addition failed, rc = %d", rc);
        bt_device_free(device);
        return;
    }

    bte_l2cap_set_userdata(device->s.connected.hid_intr, device);
    bte_l2cap_on_message_received(device->s.connected.hid_intr, hid_intr_message_received_cb);
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

    device->s.connected.hid_intr = bte_l2cap_ref(l2cap);
    device_set_connected(device);
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

    watch_connection_status(device, l2cap);

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

static void connect_to_device(egc_bt_device_t *device, const egc_bt_device_desc_t *desc)
{
    EGC_DEBUG("VID %04x, PID %04x", desc->vendor_id, desc->product_id);
    /* Allocate the device and initialize it, but don't invoke the driver yet. */
    egc_input_device_t *input_device = device->input_device =
        _egc_platform_backend.bt.device_alloc(desc);
    if (!input_device) {
        EGC_DEBUG("Couldn't allocate device");
        bt_device_free(device);
        return;
    }

    input_device->connection = EGC_CONNECTION_BT;

    if (device->state == EGC_BT_STATE_PROBING) {
        const BteBdAddr *address = device_get_address(device);
        bte_l2cap_new_configured(s_client, address, BTE_L2CAP_PSM_HID_CTRL, NULL,
                                 BTE_L2CAP_CONNECT_FLAG_NONE, NULL, hid_ctrl_connect_cb, device);
    } else if (device->state == EGC_BT_STATE_INCOMING) {
        /* The SDP channel is stored in the userdata of the HID ctrl channel */
        BteSdpClient *sdp = bte_l2cap_get_userdata(device->s.connected.hid_ctrl);
        bte_sdp_client_unref(sdp);
        /* HID channels are already connected, we can proceed with the identification */
        bte_l2cap_set_userdata(device->s.connected.hid_ctrl, device);
        device_set_connected(device);
    }
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

    connect_to_device(device, &desc);
}

static void sdp_connect_cb(BteL2cap *l2cap, const BteL2capNewConfiguredReply *reply, void *userdata)
{
    egc_bt_device_t *device = userdata;
    if (reply->result != BTE_L2CAP_INFO_RESP_RES_OK) {
        EGC_DEBUG("Failed %d", reply->result);
        bt_device_free(device);
        return;
    }

    BteSdpClient *sdp = bte_sdp_client_new(l2cap);
    if (device->state == EGC_BT_STATE_INQUIRY) {
        device->s.probing.sdp = sdp;
        device->state = EGC_BT_STATE_PROBING;
    } else if (device->state == EGC_BT_STATE_INCOMING) {
        bte_l2cap_set_userdata(device->s.connected.hid_ctrl, sdp);
    }

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

#ifdef __wii__
    /* Hack to support wiimotes emulated by Dolphin */
    const BteBdAddr *address = bte_l2cap_get_address(l2cap);
    if (address->bytes[0] == 0x11 && address->bytes[1] == 0x02 && address->bytes[2] == 0x19 &&
        address->bytes[3] == 0x79 && address->bytes[4] == 0x00) {
        EGC_DEBUG("Dolphin wiimote, skipping SDP query");
        egc_bt_device_desc_t desc = {
            0,
        };
        desc.vendor_id = 0x057e;
        desc.product_id = 0x0306;
        connect_to_device(device, &desc);
        return;
    }
#endif /* __wii__ */

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
        EGC_DEBUG("Device " EGC_BT_ADDRESS_FMT ", service class %d, major %d, minor %d",
                  EGC_BT_ADDRESS_DATA(&r->address), bte_cod_get_service_class(r->class_of_device),
                  bte_cod_get_major_dev_class(r->class_of_device),
                  bte_cod_get_minor_dev_class(r->class_of_device));
        if (bte_cod_get_major_dev_class(r->class_of_device) != BTE_COD_MAJOR_DEV_CLASS_PERIPH) {
            continue;
        }

        if (device_by_address(&r->address)) {
            /* We are already handling this device */
            continue;
        }

        BteL2CapConnectFlags flags = BTE_L2CAP_CONNECT_FLAG_NONE;
        if (s_connection_cb) {
            bool is_incoming = false;
            egc_bt_connection_reply_e reply =
                s_connection_cb((const egc_bt_address_t *)&r->address,
                                bte_cod_get_service_class(r->class_of_device),
                                bte_cod_get_major_dev_class(r->class_of_device),
                                bte_cod_get_minor_dev_class(r->class_of_device), is_incoming,
                                _egc_callbacks_userdata);
            if (reply == EGC_BT_CONNECTION_REPLY_REFUSE)
                continue;
            /* TODO: once
             * https://github.com/embedded-game-controller/bt-embedded/issues/10
             * is fixed, it's probably better to request authentication after
             * the SDP connection has been established. And maybe the decision
             * on whether authentication is needed should be taken by the HID
             * driver. */
            if (reply == EGC_BT_CONNECTION_REPLY_REQ_AUTH)
                flags |= BTE_L2CAP_CONNECT_FLAG_AUTH;
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
        bte_l2cap_new_configured(s_client, &r->address, BTE_L2CAP_PSM_SDP, &params, flags, NULL,
                                 sdp_connect_cb, device);
    }
}

static void add_ready_callback(egc_bt_initialized_cb callback)
{
    if (s_hci_ready) {
        BteHci *hci = bte_hci_get(s_client);
        callback(hci);
    } else {
        int i;
        /* Check if it's already there */
        for (i = 0; i < s_ready_callbacks_count; i++) {
            if (s_ready_callbacks[i] == callback) {
                return;
            }
        }
        if (s_ready_callbacks_count < MAX_READY_CB) {
            s_ready_callbacks[s_ready_callbacks_count++] = callback;
        }
    }
}

static void remove_ready_callback(egc_bt_initialized_cb callback)
{
    int dest_index = -1;
    for (int i = 0; i < s_ready_callbacks_count; i++) {
        if (dest_index >= 0) {
            s_ready_callbacks[dest_index++] = s_ready_callbacks[i];
        }
        if (s_ready_callbacks[i] == callback) {
            dest_index = i;
        }
    }
    if (dest_index >= 0) {
        s_ready_callbacks_count--;
    }
}

static void read_stored_link_key_cb(BteHci *hci, const BteHciReadStoredLinkKeyReply *reply,
                                    void *userdata)
{
    EGC_DEBUG("Stored keys: %d, max %d (status %d)", reply->num_keys, reply->max_keys,
              reply->status);
    if (reply->status != 0)
        return;

    s_stored_link_keys_max_ctrl = reply->max_keys;
    int count = reply->num_keys;
    if (count > s_stored_link_keys_max)
        count = s_stored_link_keys_max;
    memcpy(s_stored_link_keys, reply->stored_keys, sizeof(egc_bt_stored_link_key_t) * count);
}

static void read_stored_link_keys(BteHci *hci)
{
    if (s_stored_link_keys) {
        bte_hci_read_stored_link_key(hci, NULL, read_stored_link_key_cb, NULL);
    }
}

static void initialized_cb(BteHci *hci, bool success, void *)
{
    s_hci_ready = success;
    EGC_DEBUG("success %d", success);
    s_packet_types = bte_hci_packet_types_from_features(bte_hci_get_supported_features(hci));
    for (int i = 0; i < s_ready_callbacks_count; i++) {
        s_ready_callbacks[i](hci);
    }
}

static void start_inquiry(BteHci *hci)
{
    bte_hci_periodic_inquiry(hci, 4, 5, BTE_LAP_GIAC, 3, 0, NULL, inquiry_cb, NULL);
}

static void hid_configure_cb(BteL2cap *l2cap, const BteL2capConfigureReply *reply, void *userdata)
{
    EGC_DEBUG("rejected mask: %08x", reply->rejected_mask);
}

static void hid_state_changed_cb(BteL2cap *l2cap, BteL2capState state, void *userdata)
{
    const BteBdAddr *address = bte_l2cap_get_address(l2cap);
    egc_bt_device_t *device = device_by_address(address);
    if (!device)
        return;

    if (device->s.connected.hid_ctrl && device->s.connected.hid_intr &&
        bte_l2cap_get_state(device->s.connected.hid_ctrl) == BTE_L2CAP_OPEN &&
        bte_l2cap_get_state(device->s.connected.hid_intr) == BTE_L2CAP_OPEN) {
        bte_l2cap_new_configured(s_client, address, BTE_L2CAP_PSM_SDP, NULL,
                                 BTE_L2CAP_CONNECT_FLAG_NONE, NULL, sdp_connect_cb, device);
    }
}

static void incoming_ctrl_connected_cb(BteL2capServer *l2cap_server, BteL2cap *l2cap,
                                       void *userdata)
{
    const BteBdAddr *address = bte_l2cap_get_address(l2cap);
    EGC_DEBUG("from " EGC_BT_ADDRESS_FMT, EGC_BT_ADDRESS_DATA(address));
    egc_bt_device_t *device = bt_device_alloc(address);
    if (!device) {
        return;
    }
    device->state = EGC_BT_STATE_INCOMING;
    device->s.connected.hid_ctrl = bte_l2cap_ref(l2cap);
    device->s.connected.hid_intr = NULL;

    bte_l2cap_configure(l2cap, NULL, hid_configure_cb, device);
    bte_l2cap_on_state_changed(l2cap, hid_state_changed_cb);
    watch_connection_status(device, l2cap);
}

static void incoming_intr_connected_cb(BteL2capServer *l2cap_server, BteL2cap *l2cap,
                                       void *userdata)
{
    const BteBdAddr *address = bte_l2cap_get_address(l2cap);
    egc_bt_device_t *device = device_by_address(address);
    if (!device) {
        return;
    }

    device->s.connected.hid_intr = bte_l2cap_ref(l2cap);
    bte_l2cap_configure(l2cap, NULL, hid_configure_cb, device);
    bte_l2cap_on_state_changed(l2cap, hid_state_changed_cb);
}

static bool connection_request_cb(BteL2capServer *l2cap_server, const BteBdAddr *address,
                                  const BteClassOfDevice *cod, void *userdata)
{
    if (s_connection_cb) {
        bool is_incoming = true;
        egc_bt_connection_reply_e reply =
            s_connection_cb((const egc_bt_address_t *)address, bte_cod_get_service_class(*cod),
                            bte_cod_get_major_dev_class(*cod), bte_cod_get_minor_dev_class(*cod),
                            is_incoming, _egc_callbacks_userdata);
        if (reply == EGC_BT_CONNECTION_REPLY_REFUSE)
            return false;

        bte_l2cap_server_set_needs_auth(l2cap_server, reply == EGC_BT_CONNECTION_REPLY_REQ_AUTH);
    }
    return true;
}

static bool decline_connection(BteL2capServer *l2cap_server, const BteBdAddr *address,
                               const BteClassOfDevice *cod, void *userdata)
{
    return false;
}

static void enter_page_mode(BteHci *hci)
{
    s_l2cap_server_hid_ctrl = bte_l2cap_server_new(s_client, BTE_L2CAP_PSM_HID_CTRL);
    s_l2cap_server_hid_intr = bte_l2cap_server_new(s_client, BTE_L2CAP_PSM_HID_INTR);
    bte_l2cap_server_set_role(s_l2cap_server_hid_ctrl, BTE_HCI_ROLE_MASTER);
    bte_l2cap_server_on_connected(s_l2cap_server_hid_ctrl, incoming_ctrl_connected_cb, NULL);
    bte_l2cap_server_on_connected(s_l2cap_server_hid_intr, incoming_intr_connected_cb, NULL);
    bte_l2cap_server_on_connection_request(s_l2cap_server_hid_ctrl, connection_request_cb, NULL);
    /* Since HID clients are required to connect to the control PSM first, the
     * ACL connection is always received on the BteL2capServer handling the
     * control connection. */
    bte_l2cap_server_on_connection_request(s_l2cap_server_hid_intr, decline_connection, NULL);
}

static bool on_link_key_requested(BteHci *hci, const BteBdAddr *address, void *userdata)
{
    EGC_DEBUG("address: " EGC_BT_ADDRESS_FMT, EGC_BT_ADDRESS_DATA((egc_bt_address_t *)address));
    egc_bt_device_t *device = device_by_address(address);
    if (!device) {
        /* Not one of our devices: ignore */
        return false;
    }

    if (s_stored_link_keys) {
        /* If we have the key, use it. This code seems to be triggered only on
         * the Wii (and probably on other machines where the host BT version is
         * ancient); on modern BT controllers, the keys are handed out
         * automatically. However I've not being able to find out in which BT
         * version this changed (probably 2.1, since that's the version which
         * makes it impossible to read the link keys out of a controller). */
        for (int i = 0; i < s_stored_link_keys_max; i++) {
            egc_bt_stored_link_key_t *r = &s_stored_link_keys[i];
            if (egc_bt_address_cmp((egc_bt_address_t *)address, &r->address) == 0 &&
                !mem_is_zero(&r->key, sizeof(r->key))) {
                bte_hci_link_key_req_reply(hci, address, (BteLinkKey *)&r->key, NULL, NULL);
                return true;
            }
        }
    }

    if (s_link_key_requested_cb) {
        s_link_key_requested_cb((egc_bt_address_t *)address, _egc_callbacks_userdata);
        return true;
    } else {
        return false;
    }
}

static bool on_link_key_received(BteHci *hci, const BteHciLinkKeyNotificationData *data,
                                 void *userdata)
{
    EGC_DEBUG("address: " EGC_BT_ADDRESS_FMT ", type %d",
              EGC_BT_ADDRESS_DATA((egc_bt_address_t *)&data->address), data->key_type);
    egc_bt_device_t *device = device_by_address(&data->address);
    if (!device) {
        /* Not one of our devices: ignore */
        return false;
    }

    if (s_link_key_received_cb) {
        s_link_key_received_cb((egc_bt_address_t *)&data->address, (egc_bt_link_key_t *)&data->key,
                               _egc_callbacks_userdata);
    }
    return true;
}

static bool on_pin_code_requested(BteHci *hci, const BteBdAddr *address, void *userdata)
{
    EGC_DEBUG("address: " EGC_BT_ADDRESS_FMT, EGC_BT_ADDRESS_DATA((egc_bt_address_t *)address));
    egc_bt_device_t *device = device_by_address(address);
    if (!device) {
        /* Not one of our devices: ignore */
        return false;
    }

    if (s_pin_code_requested_cb) {
        s_pin_code_requested_cb((egc_bt_address_t *)address, _egc_callbacks_userdata);
        return true;
    } else {
        return false;
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

int _egc_bt_disconnect(egc_input_device_t *input_device)
{
    egc_bt_device_t *device = egc_bt_device_from_input(input_device);
    if (!device || device->state != EGC_BT_STATE_CONNECTED)
        return -1;

    bte_l2cap_disconnect(device->s.connected.hid_intr);
    bte_l2cap_disconnect(device->s.connected.hid_ctrl);
    return 0;
}

void _egc_bt_on_initialized(egc_bt_initialized_cb callback)
{
    add_ready_callback(callback);
}

void _egc_bt_run_inquiry()
{
    BteHci *hci = bte_hci_get(s_client);
    bte_hci_inquiry(hci, BTE_LAP_GIAC, 3, 0, NULL, inquiry_cb, NULL);
}

int egc_bt_start_scan()
{
    add_ready_callback(start_inquiry);
    return 0;
}

int egc_bt_stop_scan()
{
    if (!s_hci_ready) {
        remove_ready_callback(start_inquiry);
        return 0;
    }

    bte_hci_exit_periodic_inquiry(bte_hci_get(s_client), NULL, NULL);
    return 0;
}

int egc_bt_enter_page_mode()
{
    add_ready_callback(enter_page_mode);
    return 0;
}

int egc_bt_leave_page_mode()
{
    if (!s_hci_ready) {
        remove_ready_callback(enter_page_mode);
        return 0;
    }

    if (s_l2cap_server_hid_ctrl) {
        bte_l2cap_server_unref(s_l2cap_server_hid_ctrl);
        s_l2cap_server_hid_ctrl = NULL;
    }
    if (s_l2cap_server_hid_intr) {
        bte_l2cap_server_unref(s_l2cap_server_hid_intr);
        s_l2cap_server_hid_intr = NULL;
    }
    return 0;
}

int egc_bt_device_get_address(egc_input_device_t *input_device, egc_bt_address_t *address)
{
    egc_bt_device_t *device = egc_bt_device_from_input(input_device);
    if (!device || device->state != EGC_BT_STATE_CONNECTED)
        return -EINVAL;
    memcpy(address, device_get_address(device), sizeof(*address));
    return 0;
}

int egc_bt_get_local_address(egc_bt_address_t *address)
{
    BteHci *hci = bte_hci_get(s_client);
    bool ok = bte_hci_get_bd_address(hci, (BteBdAddr *)address);
    return ok ? 0 : -1;
}

void egc_bt_set_connection_filter(EgcBtConnectionCb callback)
{
    s_connection_cb = callback;
}

void egc_bt_on_link_key_requested(EgcBtAuthDataRequestedCb callback)
{
    s_link_key_requested_cb = callback;
    BteHci *hci = bte_hci_get(s_client);
    bte_hci_on_link_key_request(hci, on_link_key_requested);
}

void egc_bt_send_link_key(const egc_bt_address_t *address, const u8 *link_key)
{
    BteHci *hci = bte_hci_get(s_client);
    if (link_key) {
        bte_hci_link_key_req_reply(hci, (BteBdAddr *)address, (BteLinkKey *)link_key, NULL, NULL);
    } else {
        bte_hci_link_key_req_neg_reply(hci, (BteBdAddr *)address, NULL, NULL);
    }
}

void egc_bt_on_link_key_received(EgcBtLinkKeyReceivedCb callback)
{
    s_link_key_received_cb = callback;
    BteHci *hci = bte_hci_get(s_client);
    bte_hci_on_link_key_notification(hci, on_link_key_received);
}

int egc_bt_store_link_key(const egc_bt_address_t *address, const egc_bt_link_key_t *key)
{
    BteHci *hci = bte_hci_get(s_client);
    BteHciStoredLinkKey stored_key;
    memcpy(&stored_key.address, address, sizeof(egc_bt_address_t));
    memcpy(&stored_key.key, key, sizeof(egc_bt_link_key_t));
    if (s_stored_link_keys) {
        int max_keys = s_stored_link_keys_max_ctrl;
        if (max_keys > s_stored_link_keys_max)
            max_keys = s_stored_link_keys_max;

        /* Try to find a free slot */
        int dst_slot = -1;
        for (int i = 0; i < max_keys; i++) {
            egc_bt_stored_link_key_t *r = &s_stored_link_keys[i];
            if (egc_bt_address_cmp(&r->address, address) == 0 ||
                mem_is_zero(&r->address, sizeof(r->address))) {
                dst_slot = i;
                break;
            }
        }

        bool needs_deleting = false;
        if (dst_slot < 0) {
            needs_deleting = true;
            if (s_stored_link_keys_oldest >= max_keys) {
                s_stored_link_keys_oldest = 0;
            }
            dst_slot = s_stored_link_keys_oldest++;
        }

        if (needs_deleting) {
            egc_bt_address_t *dst_address = &s_stored_link_keys[dst_slot].address;
            EGC_DEBUG("Deleting key at slot %d (Address " EGC_BT_ADDRESS_FMT ")", dst_slot,
                      EGC_BT_ADDRESS_DATA(dst_address));
            bte_hci_delete_stored_link_key(hci, (BteBdAddr *)dst_address, NULL, NULL);
        }
        memcpy(&s_stored_link_keys[dst_slot], &stored_key, sizeof(stored_key));
    }
    bte_hci_write_stored_link_key(hci, 1, &stored_key, NULL, NULL);
    return 0;
}

int egc_bt_delete_link_key(const egc_bt_address_t *address)
{
    BteHci *hci = bte_hci_get(s_client);
    bte_hci_delete_stored_link_key(hci, (BteBdAddr *)address, NULL, NULL);
    return 0;
}

void egc_bt_on_pin_requested(EgcBtAuthDataRequestedCb callback)
{
    s_pin_code_requested_cb = callback;
    BteHci *hci = bte_hci_get(s_client);
    bte_hci_on_pin_code_request(hci, on_pin_code_requested);
}

void egc_bt_send_pin(const egc_bt_address_t *address, const u8 *pin, u8 length)
{
    BteHci *hci = bte_hci_get(s_client);
    if (pin && length > 0) {
        bte_hci_pin_code_req_reply(hci, (BteBdAddr *)address, pin, length, NULL, NULL);
    } else {
        bte_hci_pin_code_req_neg_reply(hci, (BteBdAddr *)address, NULL, NULL);
    }
}

void egc_bt_enable_link_keys_storage(egc_bt_stored_link_key_t *storage, u8 max_keys)
{
    s_stored_link_keys = storage;
    s_stored_link_keys_max = max_keys;
    memset(storage, 0, sizeof(egc_bt_stored_link_key_t) * max_keys);
    add_ready_callback(read_stored_link_keys);
}

#else /* !WITH_BLUETOOTH */

#include <errno.h>

#include "egc.h"

const egc_usb_transfer_t *_egc_bt_ctrl_transfer(egc_input_device_t *device, u8 requesttype,
                                                u8 request, u16 value, u16 index, void *data,
                                                u16 length, egc_transfer_cb callback)
{
    return NULL;
}

int _egc_bt_intr_transfer(egc_input_device_t *device, void *data, u16 length)
{
    return -ENOSYS;
}

void _egc_bt_disconnect(egc_input_device_t *device)
{
    return -ENOSYS;
}

int egc_bt_start_scan()
{
    return -ENOSYS;
}

int egc_bt_stop_scan()
{
    return -ENOSYS;
}

int egc_bt_enter_page_mode()
{
    return -ENOSYS;
}

int egc_bt_leave_page_mode()
{
    return -ENOSYS;
}

int egc_bt_device_get_address(egc_input_device_t *device, egc_bt_address_t *address)
{
    return -ENOSYS;
}

int egc_bt_get_local_address(egc_bt_address_t *address)
{
    return -ENOSYS;
}

void egc_bt_set_connection_filter(EgcBtConnectionCb callback, void *userdata)
{
}

void egc_bt_on_link_key_requested(EgcBtAuthDataRequestedCb callback)
{
}

void egc_bt_send_link_key(const egc_bt_address_t *address, const u8 *link_key)
{
}

void egc_bt_on_link_key_received(EgcBtLinkKeyReceivedCb callback)
{
}

int egc_bt_store_link_key(const egc_bt_address_t *address, const egc_bt_link_key_t *key)
{
    return -ENOSYS;
}

int egc_bt_delete_link_key(const egc_bt_address_t *address)
{
    return -ENOSYS;
}

void egc_bt_on_pin_requested(EgcBtAuthDataRequestedCb callback)
{
}

void egc_bt_send_pin(const egc_bt_address_t *address, const u8 *pin)
{
}

void egc_bt_enable_link_keys_storage(egc_bt_stored_link_key_t *storage, u8 max_keys)
{
}

#endif /* WITH_BLUETOOTH */

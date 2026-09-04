#include "driver_api.h"
#include "utils.h"

/* Resources:
 * - https://github.com/xerpi/libsicksaxis
 * -
 * https://web.archive.org/web/20150227021757/http://www.circuitsathome.com/mcu/programming/ps3-and-wiimote-game-controllers-on-the-arduino-host-shield-part-2
 */

#define SONY_VID 0x054c

#define DS3_ACC_RES_PER_G 125

struct ds3_input_report {
    u8 report_id;
    u8 unk0;

    u8 left : 1;
    u8 down : 1;
    u8 right : 1;
    u8 up : 1;
    u8 start : 1;
    u8 r3 : 1;
    u8 l3 : 1;
    u8 select : 1;

    u8 square : 1;
    u8 cross : 1;
    u8 circle : 1;
    u8 triangle : 1;
    u8 r1 : 1;
    u8 l1 : 1;
    u8 r2 : 1;
    u8 l2 : 1;

    u8 not_used : 7;
    u8 ps : 1;

    u8 unk1;

    u8 left_x;
    u8 left_y;
    u8 right_x;
    u8 right_y;

    u32 unk2;

    u8 dpad_sens_up;
    u8 dpad_sens_right;
    u8 dpad_sens_down;
    u8 dpad_sens_left;

    u8 shoulder_sens_l2;
    u8 shoulder_sens_r2;
    u8 shoulder_sens_l1;
    u8 shoulder_sens_r1;

    u8 button_sens_triangle;
    u8 button_sens_circle;
    u8 button_sens_cross;
    u8 button_sens_square;

    u16 unk3;
    u8 unk4;

    u8 status;
    u8 power_rating;
    u8 comm_status;

    u32 unk5;
    u32 unk6;
    u8 unk7;

    u16 acc_x;
    u16 acc_y;
    u16 acc_z;
    u16 z_gyro;
} ATTRIBUTE_PACKED;

struct ds3_rumble {
    u8 duration_right;
    u8 power_right;
    u8 duration_left;
    u8 power_left;
};

struct ds3_private_data_t {
    u8 leds;
    u8 rumble_low;
    u8 rumble_high;
    u8 step;
    bool report_received;
};
static_assert(sizeof(struct ds3_private_data_t) <= EGC_INPUT_DEVICE_DRIVER_DATA_SIZE);
#define PRIV(input_device) ((struct ds3_private_data_t *)get_priv(input_device)->private_data)

static const u8 s_elements_ds3[] = {
    /* clang-format off */
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_DPAD_LEFT,
        EGC_GAMEPAD_BUTTON_DPAD_DOWN,
        EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
        EGC_GAMEPAD_BUTTON_DPAD_UP,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_START,
        EGC_GAMEPAD_BUTTON_RIGHT_STICK,
        EGC_GAMEPAD_BUTTON_LEFT_STICK,
        EGC_GAMEPAD_BUTTON_BACK,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_WEST,
        EGC_GAMEPAD_BUTTON_SOUTH,
        EGC_GAMEPAD_BUTTON_EAST,
        EGC_GAMEPAD_BUTTON_NORTH,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
        EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
        EGC_GAMEPAD_BUTTON_RIGHT_TRIGGER,
        EGC_GAMEPAD_BUTTON_LEFT_TRIGGER,
    EGC_INPUT_REPORT_TYPE_SKIP, 4,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_GUIDE,
    EGC_INPUT_REPORT_TYPE_SKIP, 8,
    EGC_INPUT_REPORT_TYPE_AXIS_U8,
        EGC_GAMEPAD_AXIS_LEFTX,
    EGC_INPUT_REPORT_TYPE_AXIS_U8,
        EGC_GAMEPAD_AXIS_LEFTY,
    EGC_INPUT_REPORT_TYPE_AXIS_U8,
        EGC_GAMEPAD_AXIS_RIGHTX,
    EGC_INPUT_REPORT_TYPE_AXIS_U8,
        EGC_GAMEPAD_AXIS_RIGHTY,
    EGC_INPUT_REPORT_TYPE_END
    /* clang-format on */
};

static const egc_device_description_t s_device_description = {
    .vendor_id = SONY_VID,
    .product_id = 0x0268,
    /* clang-format off */
    .available_buttons =
        BIT(EGC_GAMEPAD_BUTTON_DPAD_UP) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_DOWN) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_LEFT) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_RIGHT) |
        BIT(EGC_GAMEPAD_BUTTON_NORTH) |
        BIT(EGC_GAMEPAD_BUTTON_EAST) |
        BIT(EGC_GAMEPAD_BUTTON_SOUTH) |
        BIT(EGC_GAMEPAD_BUTTON_WEST) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_TRIGGER) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_TRIGGER) |
        BIT(EGC_GAMEPAD_BUTTON_BACK) |
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_GUIDE) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_STICK),
    .available_axes =
        BIT(EGC_GAMEPAD_AXIS_LEFTX) |
        BIT(EGC_GAMEPAD_AXIS_LEFTY) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTX) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTY) |
        BIT(EGC_GAMEPAD_AXIS_LEFT_TRIGGER) |
        BIT(EGC_GAMEPAD_AXIS_RIGHT_TRIGGER),
    /* clang-format on */
    .type = EGC_DEVICE_TYPE_GAMEPAD,
    .num_touch_points = 0,
    .num_leds = 4,
    .num_accelerometers = 1,
    .has_rumble = true,
};

static int ds3_request_data(egc_input_device_t *device);

static void ds3_parse_report(egc_input_device_t *device, const struct ds3_input_report *report)
{
    struct egc_input_state_t state = { 0 };

    egc_device_driver_parse_report((u8 *)report + 2, s_elements_ds3, &state);
    egc_accelerometer_t *accel = egc_device_driver_get_accelerometer(device, &state, 0);
#define MAP_ACCEL(v) ((v) * EGC_ACCELEROMETER_RES_PER_G / DS3_ACC_RES_PER_G)
    accel->x = MAP_ACCEL(511 - (s16)be16toh(report->acc_x));
    accel->y = MAP_ACCEL(511 - (s16)be16toh(report->acc_z));
    accel->z = MAP_ACCEL((s16)be16toh(report->acc_y) - 511);
#undef MAP_ACCEL

    egc_device_driver_report_input(device, &state);
}

static int ds3_request_data(egc_input_device_t *device)
{
    egc_device_driver_enable_intr_events(device, true);
    return 0;
}

static int ds3_step(egc_input_device_t *device);

static void ds3_generic_step_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED) {
        EGC_DEBUG_DATA(transfer->data, transfer->length);
    } else {
        EGC_DEBUG("status %d", transfer->status);
        /* Retry */
        struct ds3_private_data_t *priv = PRIV(device);
        priv->step--;
    }
    ds3_step(device);
}

static int ds3_request_report(egc_input_device_t *device)
{
    char buf[49] = { 0x01, 0 };
    return egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_GET, EGC_USB_REQ_GETREPORT,
        (EGC_USB_REPTYPE_INPUT << 8) | 0x01, 0, buf, sizeof(buf), ds3_generic_step_cb);
}

static inline int ds3_write_host_address(egc_input_device_t *device,
                                         const egc_bt_address_t *address)
{
    char buf[] = { 0x01, 0x00, EGC_BT_ADDRESS_DATA(address) };
    return egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_SET, EGC_USB_REQ_SETREPORT,
        (EGC_USB_REPTYPE_FEATURE << 8) | 0xf5, 0, buf, sizeof(buf), ds3_generic_step_cb);
}

static void ds3_read_stored_host_address_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED) {
        EGC_DEBUG_DATA(transfer->data, transfer->length);
#if WITH_BLUETOOTH
        /* Compare the host address, and update it if different */
        egc_bt_address_t local_mac;
        if (device->connection == EGC_CONNECTION_USB && egc_bt_get_local_address(&local_mac) == 0) {
            const u8 *b = transfer->data + 2;
            egc_bt_address_t stored_address = {
                { b[5], b[4], b[3], b[2], b[1], b[0] }
            };
            if (egc_bt_address_cmp(&local_mac, &stored_address) != 0) {
                EGC_DEBUG("Stored address doesn't match, updating " EGC_BT_ADDRESS_FMT
                          " -> " EGC_BT_ADDRESS_FMT,
                          EGC_BT_ADDRESS_DATA(&stored_address), EGC_BT_ADDRESS_DATA(&local_mac));
                ds3_write_host_address(device, &local_mac);
                /* ds3_step will be invoked by the callback */
                return;
            }
        }
        // TODO
#endif
    } else {
        EGC_DEBUG("status %d", transfer->status);
    }
    ds3_step(device);
}

static int ds3_read_stored_host_address(egc_input_device_t *device)
{
    char buf[8];
    return egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_GET, EGC_USB_REQ_GETREPORT,
        (EGC_USB_REPTYPE_FEATURE << 8) | 0xf5, 0, buf, sizeof(buf),
        ds3_read_stored_host_address_cb);
}

static int ds3_set_operational_usb(egc_input_device_t *device)
{
    char buf[] = { 0x42, 0x0c, 0x00, 0x00 };
    return egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_SET, EGC_USB_REQ_SETREPORT,
        (EGC_USB_REPTYPE_FEATURE << 8) | 0xf4, 0, buf, sizeof(buf), ds3_generic_step_cb);
}

static int ds3_read_bd_addr(egc_input_device_t *device)
{
    char buf[17] = { 0 };
    return egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_GET, EGC_USB_REQ_GETREPORT,
        (EGC_USB_REPTYPE_FEATURE << 8) | 0xf2, 0, buf, sizeof(buf), ds3_generic_step_cb);
}

typedef int (*ds3_step_function)(egc_input_device_t *device);

static const ds3_step_function s_usb_step_functions[] = {
    /* clang-format off */
    ds3_read_bd_addr,
    ds3_set_operational_usb,
    ds3_read_stored_host_address,
    ds3_request_report,
    ds3_request_data,
    NULL,
    /* clang-format on */
};

#if WITH_BLUETOOTH
static int ds3_set_operational_bt(egc_input_device_t *device)
{
    char buf[] = { 0x42, 0x03, 0x00, 0x00 };
    return egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_SET, EGC_USB_REQ_SETREPORT,
        (EGC_USB_REPTYPE_FEATURE << 8) | 0xf4, 0, buf, sizeof(buf), ds3_generic_step_cb);
}

static const ds3_step_function s_bt_step_functions[] = {
    /* clang-format off */
    ds3_set_operational_bt,
    ds3_request_report,
    ds3_request_data,
    NULL,
    /* clang-format on */
};
#endif

static int ds3_step(egc_input_device_t *device)
{
    struct ds3_private_data_t *priv = PRIV(device);
    EGC_DEBUG("step %d", priv->step);

    const ds3_step_function *functions = NULL;
    if (device->connection == EGC_CONNECTION_USB) {
        functions = s_usb_step_functions;
#if WITH_BLUETOOTH
    } else if (device->connection == EGC_CONNECTION_BT) {
        functions = s_bt_step_functions;
#endif
    }
    ds3_step_function func = functions ? functions[priv->step] : NULL;
    if (!func) {
        EGC_DEBUG("Initialization complete");
        return 0;
    }
    priv->step++;
    if (device->connection == EGC_CONNECTION_BT) {
        func(device);
        return 0;
    } else {
        return func(device);
    }
}

static int ds3_set_leds_rumble(egc_input_device_t *device, u8 leds, const struct ds3_rumble *rumble)
{
    u8 buf[] = {
        0x01,                         /* Padding */
        0x00, 0x00, 0x00, 0x00,       /* Rumble (r, r, l, l) */
        0x00, 0x00, 0x00, 0x00,       /* Padding */
        0x00,                         /* LED_1 = 0x02, LED_2 = 0x04, ... */
        0xff, 0x27, 0x10, 0x00, 0x32, /* LED_4 */
        0xff, 0x27, 0x10, 0x00, 0x32, /* LED_3 */
        0xff, 0x27, 0x10, 0x00, 0x32, /* LED_2 */
        0xff, 0x27, 0x10, 0x00, 0x32, /* LED_1 */
        0x00, 0x00, 0x00, 0x00, 0x00  /* LED_5 (not soldered) */
    };

    buf[1] = rumble->duration_right;
    buf[2] = rumble->power_right;
    buf[3] = rumble->duration_left;
    buf[4] = rumble->power_left;
    buf[9] = leds;

    return egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_SET, EGC_USB_REQ_SETREPORT,
        (EGC_USB_REPTYPE_OUTPUT << 8) | 0x01, 0, buf, sizeof(buf), NULL);
}

static int ds3_driver_update_leds_rumble(egc_input_device_t *device)
{
    struct ds3_private_data_t *priv = PRIV(device);
    struct ds3_rumble rumble;
    u8 leds;

    leds = priv->leds << 1;

    /* Do like SDL2 does: left is low frequency, right is high */
    rumble.duration_right = 0xff;
    rumble.power_right = priv->rumble_high ? 1 : 0; /* right only supports 0 or 1 */
    rumble.duration_left = 0xff;
    rumble.power_left = priv->rumble_low;

    return ds3_set_leds_rumble(device, leds, &rumble);
}

bool ds3_driver_ops_probe(const egc_device_description_t *desc)
{
    /* The DS3 does not respond correctly to SDP queries over bluetooth, so in
     * that case we have only the name. */
    static const char name[] = "PLAYSTATION(R)3";
    if (strncmp(desc->name, name, sizeof(name) - 1) == 0) {
        egc_device_description_t *wdesc = (egc_device_description_t *)desc;
        wdesc->vendor_id = SONY_VID;
        wdesc->product_id = 0x0268;
    }

    static const egc_device_id_t compatible[] = {
        { SONY_VID, 0x0268 },
    };

    return egc_device_driver_is_compatible(desc, compatible, ARRAY_SIZE(compatible));
}

int ds3_driver_ops_init(egc_input_device_t *device)
{
    int ret;
    struct ds3_private_data_t *priv = PRIV(device);

    EGC_DEBUG("");
    device->desc = &s_device_description;

    /* Init private state */
    priv->leds = 1;
    priv->rumble_low = priv->rumble_high = 0;

    egc_device_driver_set_endpoints(device, EGC_USB_ENDPOINT_IN | 1, 15, EGC_USB_ENDPOINT_OUT | 2,
                                    15);
    egc_device_driver_set_read_size(device, sizeof(struct ds3_input_report));
    if (device->connection == EGC_CONNECTION_USB) {
        egc_device_driver_enable_intr_events(device, false);
    }
    priv->step = 0;
    ret = ds3_step(device);
    if (ret < 0)
        return ret;

    return 0;
}

int ds3_driver_ops_set_leds(egc_input_device_t *device, u32 led_state)
{
    struct ds3_private_data_t *priv = PRIV(device);

    priv->leds = led_state;

    return ds3_driver_update_leds_rumble(device);
}

int ds3_driver_ops_set_rumble(egc_input_device_t *device, u16 low_frequency, u16 high_frequency)
{
    struct ds3_private_data_t *priv = PRIV(device);

    priv->rumble_low = low_frequency >> 8;
    priv->rumble_high = high_frequency >> 8;

    return ds3_driver_update_leds_rumble(device);
}

static void ds3_driver_ops_intr_event(egc_input_device_t *device, const void *data, u16 length)
{
    struct ds3_private_data_t *priv = PRIV(device);
    EGC_DEBUG_DATA(data, length);
    if (length >= sizeof(struct ds3_input_report)) {
        ds3_parse_report(device, data);
    }

    if (!priv->report_received) {
        ds3_driver_update_leds_rumble(device);
        priv->report_received = true;
    }
}

const egc_device_driver_t ds3_usb_device_driver = {
    .probe = ds3_driver_ops_probe,
    .init = ds3_driver_ops_init,
    .set_leds = ds3_driver_ops_set_leds,
    .set_rumble = ds3_driver_ops_set_rumble,
    .intr_event = ds3_driver_ops_intr_event,
};

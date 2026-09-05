#include "driver_api.h"
#include "utils.h"

#define SONY_VID 0x054c

#define DS3_ACC_RES_PER_G 113

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

enum ds3_analog_axis_e {
    DS3_ANALOG_AXIS_LEFT_X,
    DS3_ANALOG_AXIS_LEFT_Y,
    DS3_ANALOG_AXIS_RIGHT_X,
    DS3_ANALOG_AXIS_RIGHT_Y,
    /* TODO: L2 and R2 are also axes */
    DS3_ANALOG_AXIS_COUNT
};

struct ds3_private_data_t {
    u8 leds;
    u8 rumble_low;
    u8 rumble_high;
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
    EGC_INPUT_REPORT_TYPE_AXIS_U8 | EGC_INPUT_REPORT_TYPE_AXIS_INVERTED,
        EGC_GAMEPAD_AXIS_LEFTY,
    EGC_INPUT_REPORT_TYPE_AXIS_U8,
        EGC_GAMEPAD_AXIS_RIGHTX,
    EGC_INPUT_REPORT_TYPE_AXIS_U8 | EGC_INPUT_REPORT_TYPE_AXIS_INVERTED,
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
        BIT(EGC_GAMEPAD_BUTTON_LEFT_PADDLE1) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1) |
        BIT(EGC_GAMEPAD_BUTTON_BACK) |
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_GUIDE) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_STICK),
    .available_axes =
        BIT(EGC_GAMEPAD_AXIS_LEFTX) |
        BIT(EGC_GAMEPAD_AXIS_LEFTY) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTX) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTY),
    /* clang-format on */
    .type = EGC_DEVICE_TYPE_GAMEPAD,
    .num_touch_points = 0,
    .num_leds = 4,
    .num_accelerometers = 0, /* TODO temp */
    .has_rumble = true,
};

static u8 s_output_report[] = {
    0x01, /* Report ID */
    0x00, 0xFF, 0x00, 0xFF, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0x27, 0x10, 0x00, 0x32,
    0xFF, 0x27, 0x10, 0x00, 0x32,
    0xFF, 0x27, 0x10, 0x00, 0x32,
    0xFF, 0x27, 0x10, 0x00, 0x32,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00
};

static int ds3_request_data(egc_input_device_t *device);

static void ds3_parse_report(egc_input_device_t *device, struct ds3_input_report *report)
{
    struct egc_input_state_t state = { 0 };

    egc_device_driver_parse_report((u8*)report + 2, s_elements_ds3, &state);
    egc_accelerometer_t *accel = egc_device_driver_get_accelerometer(device, &state, 0);
#define MAP_ACCEL(v) ((v) * EGC_ACCELEROMETER_RES_PER_G / DS3_ACC_RES_PER_G)
    accel->x = MAP_ACCEL((s16)report->acc_x - 511);
    accel->y = MAP_ACCEL(511 - (s16)report->acc_y);
    accel->z = MAP_ACCEL(511 - (s16)report->acc_z);
#undef MAP_ACCEL

    egc_device_driver_report_input(device, &state);
}

static void ds3_get_report_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    struct ds3_input_report *report = (void *)transfer->data;

    EGC_DEBUG("status %d, length %d", transfer->status, transfer->length);
    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED) {
        EGC_DEBUG_DATA(transfer->data, transfer->length);
        if (transfer->length >= sizeof(struct ds3_input_report) && report->report_id == 0x01) {
            ds3_parse_report(device, report);
        }
    }

    ds3_request_data(device);
}

static int ds3_request_data(egc_input_device_t *device)
{
    /*
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_GET, EGC_USB_REQ_GETREPORT,
        (EGC_USB_REPTYPE_INPUT << 8) | 0x01, 0, NULL, 0, ds3_get_report_cb);
        */
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, EGC_USB_ENDPOINT_IN | 1, NULL, sizeof(struct ds3_input_report), ds3_get_report_cb);
    EGC_DEBUG("Got transfer %p", transfer);
    return transfer != NULL ? 0 : -1;
}

static void ds3_set_operational3_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    EGC_DEBUG("status %d", transfer->status);
    EGC_DEBUG_DATA(transfer->data, transfer->length);
    ds3_request_data(device);
}

static void ds3_set_operational2_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    EGC_DEBUG("status %d", transfer->status);
    EGC_DEBUG_DATA(transfer->data, transfer->length);
    {
        const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
            device, EGC_USB_ENDPOINT_OUT | 2, s_output_report, sizeof(s_output_report),
            ds3_set_operational3_cb);
    }
}

static int ds3_set_operational2(egc_input_device_t *device)
{
    char buf[] = { 0x42, 0x0C, 0x00, 0x00 };
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_GET, EGC_USB_REQ_GETREPORT,
        (EGC_USB_REPTYPE_FEATURE << 8) | 0xf4, 0, buf, sizeof(buf), ds3_set_operational2_cb);
    return transfer != NULL ? 0 : -1;
}

static void ds3_set_operational_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED) {
        /* There's the controller's MAC BT address at offset 4 */
        EGC_DEBUG_DATA(transfer->data, transfer->length);
    } else {
        EGC_DEBUG("status %d", transfer->status);
    }
    ds3_set_operational2(device);
}

static int ds3_set_operational(egc_input_device_t *device)
{
    EGC_DEBUG("");
    char buf[17];
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_GET, EGC_USB_REQ_GETREPORT,
        (EGC_USB_REPTYPE_FEATURE << 8) | 0xf2, 0, buf, sizeof(buf), ds3_set_operational_cb);
    return transfer != NULL ? 0 : -1;
}

static int ds3_set_leds_rumble(egc_input_device_t *device, u8 leds, const struct ds3_rumble *rumble)
{
    u8 buf[] = {
        0x00,                         /* Padding */
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

    const egc_usb_transfer_t *transfer = egc_device_driver_issue_ctrl_transfer_async(
        device, EGC_USB_REQTYPE_INTERFACE_SET, EGC_USB_REQ_SETREPORT,
        (EGC_USB_REPTYPE_OUTPUT << 8) | 0x01, 0, buf, sizeof(buf), NULL);
    return transfer != NULL ? 0 : -1;
}

static int ds3_driver_update_leds_rumble(egc_input_device_t *device)
{
    struct ds3_private_data_t *priv = PRIV(device);
    struct ds3_rumble rumble;
    u8 leds;

    leds = priv->leds << 1;

    /* Do like SDL2 does: left is low frequency, right is high */
    rumble.duration_right = 0xff;
    rumble.power_right = priv->rumble_high ? 0 : 1; /* right only supports 0 or 1 */
    rumble.duration_left = 0xff;
    rumble.power_left = priv->rumble_low;

    return ds3_set_leds_rumble(device, leds, &rumble);
}

bool ds3_driver_ops_probe(u16 vid, u16 pid)
{
    static const egc_device_id_t compatible[] = {
        { SONY_VID, 0x0268 },
    };

    return egc_device_driver_is_compatible(vid, pid, compatible, ARRAY_SIZE(compatible));
}

int ds3_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    int ret;
    struct ds3_private_data_t *priv = PRIV(device);

    EGC_DEBUG("");
    device->desc = &s_device_description;

    /* Init private state */
    priv->leds = 0;
    priv->rumble_low = priv->rumble_high = 0;

    egc_device_driver_set_endpoints(device, EGC_USB_ENDPOINT_IN | 1, 5, EGC_USB_ENDPOINT_OUT | 2,
                                    5);
    ret = ds3_set_operational(device);
    if (ret < 0)
        return ret;

    return 0;
}

int ds3_driver_ops_disconnect(egc_input_device_t *device)
{
    struct ds3_private_data_t *priv = PRIV(device);

    priv->leds = 0;
    priv->rumble_low = priv->rumble_high = 0;

    return ds3_driver_update_leds_rumble(device);
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

const egc_device_driver_t ds3_usb_device_driver = {
    .probe = ds3_driver_ops_probe,
    .init = ds3_driver_ops_init,
    .disconnect = ds3_driver_ops_disconnect,
    .set_leds = ds3_driver_ops_set_leds,
    .set_rumble = ds3_driver_ops_set_rumble,
};

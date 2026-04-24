#include "driver_api.h"
#include "utils.h"
#include <string.h>

// uses Xbox 360 input for gamepad
struct iinepm_drum_input_report {
    u8 msg_type;        // always 0x00
    u8 msg_size;        // always 0x14 (20 bytes)
    u16 buttons;        // button bitmask
    u8 left_trigger;    // (0-255) - Side/rim hits
    u8 right_trigger;   // (0-255) - Side/rim hits
    s16 left_thumb_x;
    s16 left_thumb_y;
    s16 right_thumb_x;
    s16 right_thumb_y;
    u8 reserved[6];
} ATTRIBUTE_PACKED;

enum iinepm_button_bits_e {
    IINEPM_BUTTON_BIT_DPAD_UP       = 0x0001,
    IINEPM_BUTTON_BIT_DPAD_DOWN     = 0x0002,
    IINEPM_BUTTON_BIT_DPAD_LEFT     = 0x0004,
    IINEPM_BUTTON_BIT_DPAD_RIGHT    = 0x0008,
    IINEPM_BUTTON_BIT_START         = 0x0010,
    IINEPM_BUTTON_BIT_BACK          = 0x0020,
    IINEPM_BUTTON_BIT_LEFT_THUMB    = 0x0040,  // Also used for don L
    IINEPM_BUTTON_BIT_RIGHT_THUMB   = 0x0080,  // Also used for don R
    IINEPM_BUTTON_BIT_LEFT_SHOULDER = 0x0100,
    IINEPM_BUTTON_BIT_RIGHT_SHOULDER= 0x0200,
    IINEPM_BUTTON_BIT_GUIDE         = 0x0400,
    IINEPM_BUTTON_BIT_A             = 0x1000,
    IINEPM_BUTTON_BIT_B             = 0x2000,
    IINEPM_BUTTON_BIT_X             = 0x4000,
    IINEPM_BUTTON_BIT_Y             = 0x8000,
};

enum iinepm_buttons_e {
    IINEPM_BUTTON_DPAD_UP,
    IINEPM_BUTTON_DPAD_DOWN,
    IINEPM_BUTTON_DPAD_LEFT,
    IINEPM_BUTTON_DPAD_RIGHT,
    IINEPM_BUTTON_START,
    IINEPM_BUTTON_BACK,
    IINEPM_BUTTON_LEFT_THUMB,        // Also Center Drum 1
    IINEPM_BUTTON_RIGHT_THUMB,       // Also Center Drum 2
    IINEPM_BUTTON_LEFT_SHOULDER,
    IINEPM_BUTTON_RIGHT_SHOULDER,
    IINEPM_BUTTON_GUIDE,
    IINEPM_BUTTON_A,
    IINEPM_BUTTON_B,
    IINEPM_BUTTON_X,
    IINEPM_BUTTON_Y,
    IINEPM_BUTTON_COUNT
};

struct iinepm_drum_private_data_t {
    u8 last_side_hit_intensity;
    bool side_hit_active;
};
static_assert(sizeof(struct iinepm_drum_private_data_t) <= EGC_INPUT_DEVICE_PRIVATE_DATA_SIZE);

static const egc_gamepad_button_e s_button_map[IINEPM_BUTTON_COUNT] = {
    [IINEPM_BUTTON_DPAD_UP] = EGC_GAMEPAD_BUTTON_DPAD_UP,
    [IINEPM_BUTTON_DPAD_DOWN] = EGC_GAMEPAD_BUTTON_DPAD_DOWN,
    [IINEPM_BUTTON_DPAD_LEFT] = EGC_GAMEPAD_BUTTON_DPAD_LEFT,
    [IINEPM_BUTTON_DPAD_RIGHT] = EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
    [IINEPM_BUTTON_START] = EGC_GAMEPAD_BUTTON_START,
    [IINEPM_BUTTON_BACK] = EGC_GAMEPAD_BUTTON_BACK,
    [IINEPM_BUTTON_LEFT_THUMB] = EGC_GAMEPAD_BUTTON_SOUTH,      // don L
    [IINEPM_BUTTON_RIGHT_THUMB] = EGC_GAMEPAD_BUTTON_EAST,      // don R
    [IINEPM_BUTTON_LEFT_SHOULDER] = EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
    [IINEPM_BUTTON_RIGHT_SHOULDER] = EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
    [IINEPM_BUTTON_GUIDE] = EGC_GAMEPAD_BUTTON_GUIDE,
    [IINEPM_BUTTON_A] = EGC_GAMEPAD_BUTTON_SOUTH,               // xbox A = south
    [IINEPM_BUTTON_B] = EGC_GAMEPAD_BUTTON_EAST,                // xbox B = east
    [IINEPM_BUTTON_X] = EGC_GAMEPAD_BUTTON_WEST,                // xbox X = west
    [IINEPM_BUTTON_Y] = EGC_GAMEPAD_BUTTON_NORTH,               // xbox Y = north
};

static const egc_device_description_t s_device_description = {
    .vendor_id = 0x056e,
    .product_id = 0x2004,
    .available_buttons =
        BIT(EGC_GAMEPAD_BUTTON_SOUTH) |
        BIT(EGC_GAMEPAD_BUTTON_EAST) |
        BIT(EGC_GAMEPAD_BUTTON_WEST) |
        BIT(EGC_GAMEPAD_BUTTON_NORTH) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_BACK) |
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_GUIDE) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_UP) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_DOWN) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_LEFT) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_RIGHT),
    .available_axes =
        BIT(EGC_GAMEPAD_AXIS_LEFT_TRIGGER) |
        BIT(EGC_GAMEPAD_AXIS_RIGHT_TRIGGER),
    .type = EGC_DEVICE_TYPE_GAMEPAD,
    .num_touch_points = 0,
    .num_leds = 0,
    .num_accelerometers = 0,
    .has_rumble = false,
};

static int iinepm_drum_request_data(egc_input_device_t *device);

static inline u32 iinepm_get_buttons(const struct iinepm_drum_input_report *report)
{
    u32 buttons = 0;
    u16 xbox_buttons = report->buttons;

    if (xbox_buttons & IINEPM_BUTTON_BIT_DPAD_UP)       buttons |= BIT(IINEPM_BUTTON_DPAD_UP);
    if (xbox_buttons & IINEPM_BUTTON_BIT_DPAD_DOWN)     buttons |= BIT(IINEPM_BUTTON_DPAD_DOWN);
    if (xbox_buttons & IINEPM_BUTTON_BIT_DPAD_LEFT)     buttons |= BIT(IINEPM_BUTTON_DPAD_LEFT);
    if (xbox_buttons & IINEPM_BUTTON_BIT_DPAD_RIGHT)    buttons |= BIT(IINEPM_BUTTON_DPAD_RIGHT);
    if (xbox_buttons & IINEPM_BUTTON_BIT_START)         buttons |= BIT(IINEPM_BUTTON_START);
    if (xbox_buttons & IINEPM_BUTTON_BIT_BACK)          buttons |= BIT(IINEPM_BUTTON_BACK);
    if (xbox_buttons & IINEPM_BUTTON_BIT_LEFT_THUMB)    buttons |= BIT(IINEPM_BUTTON_LEFT_THUMB);
    if (xbox_buttons & IINEPM_BUTTON_BIT_RIGHT_THUMB)   buttons |= BIT(IINEPM_BUTTON_RIGHT_THUMB);
    if (xbox_buttons & IINEPM_BUTTON_BIT_LEFT_SHOULDER) buttons |= BIT(IINEPM_BUTTON_LEFT_SHOULDER);
    if (xbox_buttons & IINEPM_BUTTON_BIT_RIGHT_SHOULDER)buttons |= BIT(IINEPM_BUTTON_RIGHT_SHOULDER);
    if (xbox_buttons & IINEPM_BUTTON_BIT_GUIDE)         buttons |= BIT(IINEPM_BUTTON_GUIDE);
    if (xbox_buttons & IINEPM_BUTTON_BIT_A)             buttons |= BIT(IINEPM_BUTTON_A);
    if (xbox_buttons & IINEPM_BUTTON_BIT_B)             buttons |= BIT(IINEPM_BUTTON_B);
    if (xbox_buttons & IINEPM_BUTTON_BIT_X)             buttons |= BIT(IINEPM_BUTTON_X);
    if (xbox_buttons & IINEPM_BUTTON_BIT_Y)             buttons |= BIT(IINEPM_BUTTON_Y);

    return buttons;
}

static inline s16 iinepm_drum_velocity_to_s16(u8 velocity)
{
    // convert 0-255 range to -32768 to 32767 range
    return (velocity == 0) ? -32768 : ((velocity * 65535) / 255) - 32768;
}

static void drum_intr_transfer_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    struct iinepm_drum_input_report *report = (void *)transfer->data;
    struct iinepm_drum_private_data_t *priv = (struct iinepm_drum_private_data_t *)device->private_data;
    struct egc_input_state_t state = {0};

    static u8 prev_data[128];
    static bool first_data = true;

    // debug printing
    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED && transfer->length > 0) {
        if (first_data || memcmp(transfer->data, prev_data, transfer->length) != 0) {
            static const u8 idle_patterns[][20] = {
                {0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                {0x00, 0x14, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            };

            bool is_idle = false;
            for (int i = 0; i < 2; i++) {
                if (memcmp(transfer->data, idle_patterns[i], 20) == 0) {
                    is_idle = true;
                    break;
                }
            }

            if (!is_idle || report->buttons != 0x0000 || report->left_trigger > 0 || report->right_trigger > 0) {
                if (report->buttons != 0x0000) printf("  -> BUTTON INPUT: buttons=0x%04x\n", report->buttons);
                if (report->left_trigger > 0) printf("  -> LEFT SIDE/RIM HIT: intensity=%d (0x%02x)\n", report->left_trigger, report->left_trigger);
                if (report->right_trigger > 0) printf("  -> RIGHT SIDE/RIM HIT: intensity=%d (0x%02x)\n",report->right_trigger, report->right_trigger);
            }

            memcpy(prev_data, transfer->data, transfer->length);
            first_data = false;
        }
    }

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED &&
        transfer->length >= sizeof(struct iinepm_drum_input_report)) {

        u32 buttons = iinepm_get_buttons(report);
        state.gamepad.buttons = egc_device_driver_map_buttons(buttons, IINEPM_BUTTON_COUNT, s_button_map);

        if (report->left_trigger > 0) {
            state.gamepad.buttons |= BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER);
            state.gamepad.axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER] = iinepm_drum_velocity_to_s16(report->left_trigger);

            priv->side_hit_active = true;
            priv->last_side_hit_intensity = report->left_trigger;
        } else if (priv->side_hit_active) {
            priv->side_hit_active = false;
            state.gamepad.axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER] = -32768; //min
        } else {
            state.gamepad.axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER] = -32768; //min
        }

        if (report->right_trigger > 0) {
            state.gamepad.buttons |= BIT(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER);
            state.gamepad.axes[EGC_GAMEPAD_AXIS_RIGHT_TRIGGER] = iinepm_drum_velocity_to_s16(report->right_trigger);

            priv->side_hit_active = true;
            priv->last_side_hit_intensity = report->right_trigger;
        } else if (priv->side_hit_active) {
            priv->side_hit_active = false;
            state.gamepad.axes[EGC_GAMEPAD_AXIS_RIGHT_TRIGGER] = -32768; //min
        } else {
            state.gamepad.axes[EGC_GAMEPAD_AXIS_RIGHT_TRIGGER] = -32768; //min
        }

        egc_device_driver_report_input(device, &state);
    }
    iinepm_drum_request_data(device);
}

static void iinepm_drum_init_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED) {
        printf("initialization completed successfully\n");
    } else {
        printf("initialization failed with status: %d\n", transfer->status);
    }

    // Start requesting data regardless of init result
    iinepm_drum_request_data(device);
}

static int iinepm_drum_request_data(egc_input_device_t *device)
{
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, EGC_USB_ENDPOINT_IN | 1, NULL, 0, drum_intr_transfer_cb);

    if (transfer == NULL) {
        printf("Failed to issue USB interrupt transfer\n");
        return -1;
    }

    return 0;
}

static bool iinepm_drum_driver_ops_probe(u16 vid, u16 pid)
{
    static const egc_device_id_t compatible[] = {
        { 0x056e, 0x2004 },
    };

    return egc_device_driver_is_compatible(vid, pid, compatible, ARRAY_SIZE(compatible));
}

static int iinepm_drum_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    struct iinepm_drum_private_data_t *priv_data = (struct iinepm_drum_private_data_t *)device->private_data;

    device->desc = &s_device_description;
    priv_data->last_side_hit_intensity = 0;
    priv_data->side_hit_active = false;

    egc_device_driver_set_timer(device, 1000 * 500, 0); //500ms delay
    return 0;
}

static bool iinepm_drum_driver_ops_timer(egc_input_device_t *device)
{
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_ctrl_transfer_async(
        device,
        EGC_USB_CTRLTYPE_DIR_DEVICE2HOST | EGC_USB_CTRLTYPE_TYPE_CLASS | EGC_USB_CTRLTYPE_REC_INTERFACE,
        EGC_USB_REQ_GETREPORT,
        (EGC_USB_REPTYPE_INPUT << 8) | 0x00,
        0,
        NULL,
        0,
        iinepm_drum_init_cb);

    if (transfer == NULL) {
        printf("Failed to issue Xbox 360 initialization transfer\n");
        // Still try to start data requests
        return iinepm_drum_request_data(device);
    }
    return false;
}

static int iinepm_drum_driver_ops_disconnect(egc_input_device_t *device)
{
    printf("Device disconnected\n");
    return 0;
}

const egc_device_driver_t iine_pro_max_device_driver = {
    .probe = iinepm_drum_driver_ops_probe,
    .init = iinepm_drum_driver_ops_init,
    .disconnect = iinepm_drum_driver_ops_disconnect,
    .timer = iinepm_drum_driver_ops_timer,
};

#include "driver_api.h"
#include "generic_mappings.h"
#include "utils.h"

struct dr_private_data_t {
    bool process_reports;
    const u8 *report_elements;
};
static_assert(sizeof(struct dr_private_data_t) <= EGC_INPUT_DEVICE_DRIVER_DATA_SIZE);
#define PRIV(input_device) ((struct dr_private_data_t *)get_priv(input_device)->private_data)

static const egc_device_description_t s_device_description = {
    .vendor_id = 0x0079,
    .product_id = 0x0006,
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
        BIT(EGC_GAMEPAD_BUTTON_BACK) |
        BIT(EGC_GAMEPAD_BUTTON_START) |
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
    .num_leds = 0,
    .num_accelerometers = 0,
    .has_rumble = false,
};

static void dr_driver_ops_intr_event(egc_input_device_t *device, const void *data, u16 length)
{
    struct dr_private_data_t *priv = PRIV(device);
    struct egc_input_state_t state = {
        0,
    };

    if (priv->process_reports) {
        egc_device_driver_parse_report(data, priv->report_elements, &state);
        egc_device_driver_report_input(device, &state);
    }
}

static bool dr_driver_ops_probe(u16 vid, u16 pid)
{
    const u8 *elements = egc_device_driver_input_parser_for(vid, pid);
    return elements != NULL;
}

static int dr_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    struct dr_private_data_t *priv = PRIV(device);

    egc_device_description_t *desc = egc_device_driver_alloc_desc(device);
    desc->vendor_id = vid;
    desc->product_id = pid;
    priv->report_elements = egc_device_driver_input_parser_for(vid, pid);
    egc_device_driver_fill_desc(desc, priv->report_elements);

    /* Compute the report size by parsing a fake report */
    {
        u8 null_report[128] = {
            0,
        };
        struct egc_input_state_t state;
        u16 size = egc_device_driver_parse_report(null_report, priv->report_elements, &state);
        egc_device_driver_set_read_size(device, size);
    }

    device->desc = desc;
    egc_device_driver_set_endpoints(device, EGC_USB_ENDPOINT_IN | 1, 5, 0 /* not used */, 5);

    priv->process_reports = false;
    /* Set a half-second timer to let the device stabilize before requesting
     * updates */
    egc_device_driver_set_timer(device, 1000 * 500, 0);
    return 0;
}

static bool dr_driver_ops_timer(egc_input_device_t *device)
{
    struct dr_private_data_t *priv = PRIV(device);
    priv->process_reports = true;

    /* Return false to destroy the timer */
    return false;
}

const egc_device_driver_t dr_usb_device_driver = {
    .probe = dr_driver_ops_probe,
    .init = dr_driver_ops_init,
    .timer = dr_driver_ops_timer,
    .intr_event = dr_driver_ops_intr_event,
};

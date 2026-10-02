#include "driver_api.h"
#include "generic_mappings.h"
#include "utils.h"

struct dr_private_data_t {
    bool process_reports;
    const u8 *report_elements;
};
static_assert(sizeof(struct dr_private_data_t) <= EGC_INPUT_DEVICE_DRIVER_DATA_SIZE);
#define PRIV(input_device) ((struct dr_private_data_t *)get_priv(input_device)->private_data)

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

static bool dr_driver_ops_probe(const egc_device_description_t *desc)
{
    const u8 *elements = egc_device_driver_input_parser_for(desc->vendor_id, desc->product_id);
    return elements != NULL;
}

static int dr_driver_ops_init(egc_input_device_t *device)
{
    struct dr_private_data_t *priv = PRIV(device);

    egc_device_description_t *desc = egc_device_driver_get_desc(device);
    priv->report_elements = egc_device_driver_input_parser_for(desc->vendor_id, desc->product_id);
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

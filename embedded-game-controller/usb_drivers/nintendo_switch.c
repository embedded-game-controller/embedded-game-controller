#include <limits.h>

#include "driver_api.h"
#include "utils.h"

/* This should increase the frequency of updates. On the Wii this makes no
 * difference, we get from in average 54 valid updates per second in any case.
 */
#define NS_INCREASE_BAUDRATE 0

/* Constants taken from the hid-nintendo.c driver of the Linux kernel */

/* Output Reports */
#define JC_OUTPUT_RUMBLE_AND_SUBCMD 0x01
#define JC_OUTPUT_FW_UPDATE_PKT     0x03
#define JC_OUTPUT_RUMBLE_ONLY       0x10
#define JC_OUTPUT_MCU_DATA          0x11
#define JC_OUTPUT_USB_CMD           0x80

/* Subcommand IDs */
#define JC_SUBCMD_STATE                 0x00
#define JC_SUBCMD_MANUAL_BT_PAIRING     0x01
#define JC_SUBCMD_REQ_DEV_INFO          0x02
#define JC_SUBCMD_SET_REPORT_MODE       0x03
#define JC_SUBCMD_TRIGGERS_ELAPSED      0x04
#define JC_SUBCMD_GET_PAGE_LIST_STATE   0x05
#define JC_SUBCMD_SET_HCI_STATE         0x06
#define JC_SUBCMD_RESET_PAIRING_INFO    0x07
#define JC_SUBCMD_LOW_POWER_MODE        0x08
#define JC_SUBCMD_SPI_FLASH_READ        0x10
#define JC_SUBCMD_SPI_FLASH_WRITE       0x11
#define JC_SUBCMD_RESET_MCU             0x20
#define JC_SUBCMD_SET_MCU_CONFIG        0x21
#define JC_SUBCMD_SET_MCU_STATE         0x22
#define JC_SUBCMD_SET_PLAYER_LIGHTS     0x30
#define JC_SUBCMD_GET_PLAYER_LIGHTS     0x31
#define JC_SUBCMD_SET_HOME_LIGHT        0x38
#define JC_SUBCMD_ENABLE_IMU            0x40
#define JC_SUBCMD_SET_IMU_SENSITIVITY   0x41
#define JC_SUBCMD_WRITE_IMU_REG         0x42
#define JC_SUBCMD_READ_IMU_REG          0x43
#define JC_SUBCMD_ENABLE_VIBRATION      0x48
#define JC_SUBCMD_GET_REGULATED_VOLTAGE 0x50

/* USB Commands */
#define JC_USB_CMD_CONN_STATUS 0x01
#define JC_USB_CMD_HANDSHAKE   0x02
#define JC_USB_CMD_BAUDRATE_3M 0x03
#define JC_USB_CMD_NO_TIMEOUT  0x04
#define JC_USB_CMD_EN_TIMEOUT  0x05
#define JC_USB_RESET           0x06
#define JC_USB_PRE_HANDSHAKE   0x91
#define JC_USB_SEND_UART       0x92

/* Input Reports */
#define JC_INPUT_BUTTON_EVENT 0x3F
#define JC_INPUT_SUBCMD_REPLY 0x21
#define JC_INPUT_IMU_DATA     0x30
#define JC_INPUT_MCU_DATA     0x31
#define JC_INPUT_USB_RESPONSE 0x81

/* Magic value denoting presence of user calibration */
#define JC_CAL_USR_MAGIC 0xA1B2

/* SPI FLASH addresses */
#define JC_SPI_ADDR_IMU_CAL_USR_MAGIC 0x8026
#define JC_SPI_ADDR_IMU_CAL_USR       0x8028
#define JC_SPI_ADDR_IMU_CAL_FCT       0x6020

#define JC_DFLT_ACCEL_SCALE 0x4000

static u8 s_rumble_data[8] = {
    0x00, 0x10, 0x40, 0x40, 0x00, 0x10, 0x40, 0x40,
};

typedef struct {
    s16 accel_x;
    s16 accel_y;
    s16 accel_z;
    s16 gyro_x;
    s16 gyro_y;
    s16 gyro_z;
} ATTRIBUTE_PACKED ns_joycon_imu_data_t;

typedef struct {
    u16 addr;
    u16 unknown;
    u8 size;
} ATTRIBUTE_PACKED ns_spi_read_request_t;

typedef struct {
    s16 accel_offset[3];
    s16 accel_scale[3];
    s16 gyro_offset[3];
    s16 gyro_scale[3];
} ATTRIBUTE_PACKED ns_joycon_imu_cal_t;

typedef struct {
    u8 ack; /* MSB 1 for ACK, 0 for NACK */
    u8 id;  /* id of requested subcmd */
    union {
        struct {
            ns_spi_read_request_t req;
            union {
                struct ns_spi_imu_cal_user_t {
                    /* SPI addr 0x8026 */
                    u16 magic;
                    /* SPI addr 0x8028 */
                    ns_joycon_imu_cal_t cal;
                } ATTRIBUTE_PACKED imu_cal_user;

                ns_joycon_imu_cal_t imu_cal_factory;
            };
        } ATTRIBUTE_PACKED spi_reply; /* SPI flash data */
        u8 data[35];                  /* will be at most 35 bytes */
    };
} ATTRIBUTE_PACKED ns_joycon_subcmd_reply_t;

typedef struct {
    u8 id;
    u8 timer;
    u8 bat_con; /* battery and connection info */
    u8 button_status[3];
    u8 left_stick[3];
    u8 right_stick[3];
    u8 vibrator_report;
    union {
        ns_joycon_subcmd_reply_t subcmd_reply;
        ns_joycon_imu_data_t imu_data[3];
    };
} ATTRIBUTE_PACKED ns_input_report_t;

struct ns_subcmd_request {
    u8 output_id;  /* JC_OUTPUT_RUMBLE_AND_SUBCMD or JC_OUTPUT_RUMBLE_ONLY */
    u8 packet_num; /* incremented every send */
    u8 rumble_data[8];
    u8 subcmd_id;
    union {
        u8 data[53]; /* The maximum transfer size for this controller is 64
                        bytes */
        ns_spi_read_request_t spi_read;
    };
} ATTRIBUTE_PACKED;

enum ns_buttons_e {
    /* bitfield1: */
    NS_BUTTON_Y,
    NS_BUTTON_X,
    NS_BUTTON_B,
    NS_BUTTON_A,
    NS_BUTTON_SR_R,
    NS_BUTTON_SL_R,
    NS_BUTTON_R,
    NS_BUTTON_ZR,
    /* bitfield2: */
    NS_BUTTON_MINUS,
    NS_BUTTON_PLUS,
    NS_BUTTON_RSTICK,
    NS_BUTTON_LSTICK,
    NS_BUTTON_HOME,
    NS_BUTTON_CAP,
    NS_BUTTON_UNUSED0,
    NS_BUTTON_UNUSED1,
    /* bitfield3: */
    NS_BUTTON_DOWN,
    NS_BUTTON_UP,
    NS_BUTTON_RIGHT,
    NS_BUTTON_LEFT,
    NS_BUTTON_SR_L,
    NS_BUTTON_SL_L,
    NS_BUTTON_L,
    NS_BUTTON_ZL,
    NS_BUTTON_COUNT
};

static const egc_gamepad_button_e s_button_map[NS_BUTTON_COUNT] = {
    [NS_BUTTON_UP] = EGC_GAMEPAD_BUTTON_DPAD_UP,
    [NS_BUTTON_DOWN] = EGC_GAMEPAD_BUTTON_DPAD_DOWN,
    [NS_BUTTON_LEFT] = EGC_GAMEPAD_BUTTON_DPAD_LEFT,
    [NS_BUTTON_RIGHT] = EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
    [NS_BUTTON_X] = EGC_GAMEPAD_BUTTON_NORTH,
    [NS_BUTTON_A] = EGC_GAMEPAD_BUTTON_EAST,
    [NS_BUTTON_B] = EGC_GAMEPAD_BUTTON_SOUTH,
    [NS_BUTTON_Y] = EGC_GAMEPAD_BUTTON_WEST,
    [NS_BUTTON_L] = EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
    [NS_BUTTON_R] = EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
    [NS_BUTTON_ZL] = EGC_GAMEPAD_BUTTON_LEFT_PADDLE1,
    [NS_BUTTON_ZR] = EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1,
    [NS_BUTTON_MINUS] = EGC_GAMEPAD_BUTTON_BACK,
    [NS_BUTTON_PLUS] = EGC_GAMEPAD_BUTTON_START,
    [NS_BUTTON_LSTICK] = EGC_GAMEPAD_BUTTON_LEFT_STICK,
    [NS_BUTTON_RSTICK] = EGC_GAMEPAD_BUTTON_RIGHT_STICK,
    [NS_BUTTON_HOME] = EGC_GAMEPAD_BUTTON_GUIDE,
    [NS_BUTTON_CAP] = EGC_GAMEPAD_BUTTON_MISC1,
};

enum ns_analog_axis_e {
    NS_ANALOG_AXIS_LEFT_X,
    NS_ANALOG_AXIS_LEFT_Y,
    NS_ANALOG_AXIS_RIGHT_X,
    NS_ANALOG_AXIS_RIGHT_Y,
    NS_ANALOG_AXIS__NUM
};

typedef enum {
    NS_CODED_COMMAND,
    NS_CODED_SUBCOMMAND,
    NS_CODED_GET_DESCRIPTOR,
    NS_CODED_CAL,
    NS_CODED_PAIR,
    NS_CODED_START_READS,
} ns_coded_command_type;

typedef struct {
    ns_coded_command_type type;
    union {
        struct {
            u8 op;
        } cmd;
        struct {
            u8 value;
        } dsc;
        struct {
            u8 op;
            u8 data; /* Only used in subcommands */
            u8 size;
        } subcmd;
        struct {
            u16 addr;
            u8 size;
        } cal;
    };
} ATTRIBUTE_PACKED ns_coded_command_t;

static const ns_coded_command_t s_initialization_commands[] = {
    { NS_CODED_GET_DESCRIPTOR, { .dsc = { 0 } } },
    { NS_CODED_GET_DESCRIPTOR, { .dsc = { 1 } } },
    { NS_CODED_GET_DESCRIPTOR, { .dsc = { 2 } } },
    { NS_CODED_GET_DESCRIPTOR, { .dsc = { 3 } } },
    { NS_CODED_COMMAND, { .cmd = { JC_USB_CMD_HANDSHAKE } } },
#if NS_INCREASE_BAUDRATE
    { NS_CODED_COMMAND, { .cmd = { JC_USB_CMD_BAUDRATE_3M } } },
    { NS_CODED_COMMAND, { .cmd = { JC_USB_CMD_HANDSHAKE } } },
#endif
    { NS_CODED_COMMAND, { .cmd = { JC_USB_CMD_NO_TIMEOUT } } },
    { NS_CODED_SUBCOMMAND, { .subcmd = { JC_SUBCMD_REQ_DEV_INFO, 0, 0 } } },
    { NS_CODED_SUBCOMMAND, { .subcmd = { JC_SUBCMD_LOW_POWER_MODE, 0, 1 } } },
    /* This (as the descriptor requests above) is essential for third party
     * controllers such as the 8bitdo Ultimate in order to avoid them from
     * disconnecting and switching to PC mode */
    {
     NS_CODED_PAIR, },
    /* IMU factory calibration data */
    { NS_CODED_CAL, { .cal = { JC_SPI_ADDR_IMU_CAL_FCT, sizeof(ns_joycon_imu_cal_t) } } },
    /* Get user calibration for imu; if valid, this will overwrite the factory
     * calibration we retrieved before */
    { NS_CODED_CAL,
     { .cal = { JC_SPI_ADDR_IMU_CAL_USR_MAGIC, sizeof(struct ns_spi_imu_cal_user_t) } } },
    { NS_CODED_SUBCOMMAND, { .subcmd = { JC_SUBCMD_SET_REPORT_MODE, JC_INPUT_IMU_DATA, 1 } } },
    { NS_CODED_SUBCOMMAND, { .subcmd = { JC_SUBCMD_ENABLE_IMU, 1, 1 } } },
    { NS_CODED_SUBCOMMAND, { .subcmd = { JC_SUBCMD_ENABLE_VIBRATION, 1, 1 } } },
};
#define NS_INITIALIZATION_COMPLETED (ARRAY_SIZE(s_initialization_commands))

struct ns_private_data_t {
    int update_count;
    ns_joycon_imu_cal_t imu_cal;
    s16 accel_divisor[3];
    s16 gyro_divisor[3];
    s8 init_state;
    u8 next_packet_num;
    u8 bt_mac_addr[6];
    u8 requested_leds;
    bool led_change_queued;
    bool requested_rumble;
};
static_assert(sizeof(struct ns_private_data_t) <= EGC_INPUT_DEVICE_PRIVATE_DATA_SIZE);

static const egc_device_description_t s_device_description = {
    .vendor_id = 0x057e,
    .product_id = 0x2009,
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
        BIT(EGC_GAMEPAD_BUTTON_MISC1) |
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
    .num_accelerometers = 1,
    .has_rumble = true,
};

static void ns_active_step(egc_input_device_t *device);

static inline u32 ns_get_buttons(const u8 *buttons)
{
    return buttons[0] | (buttons[1] << 8) | (buttons[2] << 16);
}

static void ns_get_analog_axis(const u8 *bytes, s16 *axes)
{
    /* TODO: adjust the values according with the calibration data */
    axes[0] = (s16)((bytes[1] << 12) | (bytes[0] << 4) | (bytes[0] & 0x0f)) + SHRT_MIN;
    axes[1] = (s16)((bytes[2] << 8) | (bytes[1] >> 4) | (bytes[1] & 0xf0)) + SHRT_MIN;
}

static inline void ns_get_accel(const struct ns_private_data_t *priv,
                                const ns_input_report_t *report, egc_accelerometer_t *accel)
{
    int x, y, z;

    x = y = z = 0;
    int weight = ARRAY_SIZE(report->imu_data);
    int divisor = 0;
    for (int i = 0; i < ARRAY_SIZE(report->imu_data); i++) {
        x += (s16)le16toh(report->imu_data[i].accel_x) * weight;
        y += (s16)le16toh(report->imu_data[i].accel_y) * weight;
        z += (s16)le16toh(report->imu_data[i].accel_z) * weight;
        divisor += weight;
        weight--;
    }
    x /= divisor;
    y /= divisor;
    z /= divisor;
    accel->x = x * EGC_ACCELEROMETER_RES_PER_G / (priv->accel_divisor[0]);
    accel->y = y * EGC_ACCELEROMETER_RES_PER_G / (priv->accel_divisor[1]);
    accel->z = z * EGC_ACCELEROMETER_RES_PER_G / (priv->accel_divisor[2]);
}

static bool parse_input_report(const struct ns_private_data_t *priv,
                               const ns_input_report_t *report, struct egc_input_state_t *state)
{
    if (report->id != JC_INPUT_IMU_DATA && report->id != 0) {
        LOG_INFO("  report ID: %02x\n", report->id);
        return false;
    }
    u32 buttons = ns_get_buttons(report->button_status);
    state->gamepad.buttons = egc_device_driver_map_buttons(buttons, NS_BUTTON_COUNT, s_button_map);
    ns_get_analog_axis(report->left_stick, &state->gamepad.axes[NS_ANALOG_AXIS_LEFT_X]);
    ns_get_analog_axis(report->right_stick, &state->gamepad.axes[NS_ANALOG_AXIS_RIGHT_X]);
    ns_get_accel(priv, report, &state->gamepad.accelerometer[0]);
    return true;
}

static void on_request_completed(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    ns_input_report_t *report = (void *)transfer->data;
    struct ns_private_data_t *priv = (void *)device->private_data;
    struct egc_input_state_t state = { 0 };

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED && transfer->length > 0) {
        if (parse_input_report(priv, report, &state)) {
            egc_device_driver_report_input(device, &state);
            priv->update_count++;
        }
    } else if (transfer->status == EGC_USB_TRANSFER_STATUS_ERROR) {
        LOG_DEBUG("%s, status = %d, length=%d\n", __func__, transfer->status, transfer->length);
    }

    ns_active_step(device);
}

static int ns_request_data(egc_input_device_t *device)
{
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, EGC_USB_ENDPOINT_IN | 1, NULL, 0, on_request_completed);
    if (!transfer) {
        LOG_INFO("Couldn't get a transfer!\n");
    }
    return transfer != NULL ? 0 : -1;
}

static void on_command_completed(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    ns_request_data(device);
}

static int ns_send_report(egc_input_device_t *device, u8 *data, int size, egc_transfer_cb callback)
{
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, EGC_USB_ENDPOINT_OUT | 2, data, size, callback);
    return transfer != NULL ? 0 : -1;
}

static int ns_send_command_usb(egc_input_device_t *device, u8 command, egc_transfer_cb callback)
{
    u8 data[2] ATTRIBUTE_ALIGN(32) = { JC_OUTPUT_USB_CMD, command };
    return ns_send_report(device, data, sizeof(data), callback);
}

static int ns_send_subcmd(egc_input_device_t *device, struct ns_subcmd_request *req, int size,
                          egc_transfer_cb callback)
{
    struct ns_private_data_t *priv = (void *)device->private_data;

    req->output_id = JC_OUTPUT_RUMBLE_AND_SUBCMD;
    req->packet_num = priv->next_packet_num;
    if (++priv->next_packet_num > 0xf) {
        priv->next_packet_num = 0;
    }
    memcpy(req->rumble_data, s_rumble_data, sizeof(s_rumble_data));
    return ns_send_report(device, (u8 *)req, sizeof(*req) + size, callback);
}

static int ns_rumble(egc_input_device_t *device)
{
    u8 data[10] = { JC_OUTPUT_RUMBLE_ONLY, 0x00 };

    /* Frequency and amplitude can be configured (see the Linux driver for
     * reference), but for the time being let's just hardcode values for a mild
     * vibration. */
    u16 freq_hi = 0x8800;
    u8 freq_lo = 0x42;
    u8 amp_hi = 0xae;
    u16 amp_lo = 0x806b;
    u8 *rumble = data + 2;
    rumble[0] = rumble[4] = freq_hi >> 8;
    rumble[1] = rumble[5] = (freq_hi && 0xff) + amp_hi;
    rumble[2] = rumble[6] = freq_lo + (amp_lo >> 8);
    rumble[3] = rumble[7] = amp_lo & 0xff;

    return ns_send_report(device, data, sizeof(data), on_command_completed);
}

static int ns_set_player_leds(egc_input_device_t *device, u8 flash, u8 on)
{
    struct ns_subcmd_request *req;
    u8 buf[sizeof(*req) + 1] = { 0 };
    req = (struct ns_subcmd_request *)buf;

    req->subcmd_id = JC_SUBCMD_SET_PLAYER_LIGHTS;
    req->data[0] = flash << 4 | on;
    return ns_send_subcmd(device, req, 1, on_command_completed);
}

void ns_active_step(egc_input_device_t *device)
{
    struct ns_private_data_t *priv = (void *)device->private_data;

    if (priv->led_change_queued) {
        ns_set_player_leds(device, 0, priv->requested_leds);
        priv->led_change_queued = false;
    } else if (priv->requested_rumble) {
        ns_rumble(device);
    } else {
        ns_request_data(device);
    }
}

static int ns_init_step(egc_input_device_t *device);

static void ns_copy_u16_from_le(u16 *dst, const u16 *src, size_t count)
{
    for (int i = 0; i < count; i++)
        dst[i] = le16toh(src[i]);
}

static void ns_init_step_read_data_reply(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    struct ns_private_data_t *priv = (void *)device->private_data;

    const ns_coded_command_t *step = &s_initialization_commands[priv->init_state];
    if (step->type == NS_CODED_CAL) {
        ns_input_report_t *report = (void *)transfer->data;

        if (report->subcmd_reply.ack) {
            u16 requested_address = report->subcmd_reply.spi_reply.req.addr;
            if (requested_address == htole16(JC_SPI_ADDR_IMU_CAL_USR_MAGIC)) {
                struct ns_spi_imu_cal_user_t *user_cal =
                    &report->subcmd_reply.spi_reply.imu_cal_user;

                if (user_cal->magic == htole16(JC_CAL_USR_MAGIC)) {
                    ns_copy_u16_from_le((u16 *)&priv->imu_cal, ((u16 *)user_cal) + 1,
                                        sizeof(ns_joycon_imu_cal_t) / sizeof(u16));
                }
            } else if (requested_address == htole16(JC_SPI_ADDR_IMU_CAL_FCT)) {
                ns_joycon_imu_cal_t *factory_cal = &report->subcmd_reply.spi_reply.imu_cal_factory;
                ns_copy_u16_from_le((u16 *)&priv->imu_cal, (u16 *)factory_cal,
                                    sizeof(ns_joycon_imu_cal_t) / sizeof(u16));
            } else {
                LOG_DEBUG("Got SPI address: %04x\n",
                          (int)le16toh(report->subcmd_reply.spi_reply.req.addr));
            }
        }
    }
    ns_init_step(device);
}

static void ns_init_step_reply(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    egc_device_driver_issue_intr_transfer_async(device, EGC_USB_ENDPOINT_IN | 1, NULL, 0,
                                                ns_init_step_read_data_reply);
}

static void ns_string_descriptor_reply(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    ns_init_step(device);
}

static int ns_get_string_descriptor(egc_input_device_t *device, u8 index)
{
    u16 language = 0;
    egc_device_driver_issue_ctrl_transfer_async(
        device,
        EGC_USB_CTRLTYPE_DIR_DEVICE2HOST | EGC_USB_CTRLTYPE_TYPE_STANDARD |
            EGC_USB_CTRLTYPE_REC_DEVICE,
        EGC_USB_REQ_GETDESCRIPTOR, (EGC_USB_REPTYPE_FEATURE << 8) | index, language, NULL, 0,
        ns_string_descriptor_reply);
    return 0;
}

static inline void ns_prepare_calibration(struct ns_private_data_t *priv)
{
    for (int i = 0; i < 3; i++) {
        priv->accel_divisor[i] = (priv->imu_cal.accel_scale[i] - priv->imu_cal.accel_offset[i]) *
                                 EGC_ACCELEROMETER_RES_PER_G / JC_DFLT_ACCEL_SCALE;
        priv->gyro_divisor[i] = priv->imu_cal.gyro_scale[i] - priv->imu_cal.gyro_offset[i];

        /* this should never happen, but make sure we don't crash the driver */
        if (priv->accel_divisor[i] == 0)
            priv->accel_divisor[i] = 1;
        if (priv->gyro_divisor[i] == 0)
            priv->gyro_divisor[i] = 1;
    }
}

static int ns_init_step(egc_input_device_t *device)
{
    struct ns_private_data_t *priv = (void *)device->private_data;
    int rc;

    priv->init_state++;

    if (priv->init_state >= NS_INITIALIZATION_COMPLETED) {
        ns_prepare_calibration(priv);
        /* Start reading data */
        ns_active_step(device);
    } else {
        const ns_coded_command_t *step = &s_initialization_commands[priv->init_state];
        if (step->type == NS_CODED_COMMAND) {
            rc = ns_send_command_usb(device, step->cmd.op, ns_init_step_reply);
        } else if (step->type == NS_CODED_SUBCOMMAND) {
            u8 buf[64] = { 0 };
            struct ns_subcmd_request *req = (struct ns_subcmd_request *)buf;
            req->subcmd_id = step->subcmd.op;
            req->data[0] = step->subcmd.data;
            rc = ns_send_subcmd(device, req, step->subcmd.size, ns_init_step_reply);
        } else if (step->type == NS_CODED_CAL) {
            u8 buf[64] = { 0 };
            struct ns_subcmd_request *req = (struct ns_subcmd_request *)buf;
            req->subcmd_id = JC_SUBCMD_SPI_FLASH_READ;
            req->spi_read.addr = le16toh(step->cal.addr);
            req->spi_read.size = step->cal.size;
            rc = ns_send_subcmd(device, req, 5, ns_init_step_reply);
        } else if (step->type == NS_CODED_PAIR) {
            u8 buf[64] = { 0 };
            struct ns_subcmd_request *req = (struct ns_subcmd_request *)buf;
            req->subcmd_id = JC_SUBCMD_MANUAL_BT_PAIRING;
            req->data[0] = 0x04;
            memcpy(req->data + 1, priv->bt_mac_addr, 6);
            req->data[8] = 0x04;
            strcpy((char *)req->data + 9, "<Nintendo Switch");
            rc = ns_send_subcmd(device, req, 32, ns_init_step_reply);
        } else if (step->type == NS_CODED_GET_DESCRIPTOR) {
            rc = ns_get_string_descriptor(device, step->dsc.value);
        }
    }
    return rc;
}

static bool ns_driver_ops_probe(u16 vid, u16 pid)
{
    static const egc_device_id_t compatible[] = {
        { 0x057e, 0x2009 }, /* Switch Pro Controller */
    };

    return egc_device_driver_is_compatible(vid, pid, compatible, ARRAY_SIZE(compatible));
}

static int ns_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    struct ns_private_data_t *priv = (void *)device->private_data;

    device->desc = &s_device_description;

    /* Init private state */
    priv->next_packet_num = 0;
    priv->init_state = -1;
    priv->update_count = 0;
    priv->led_change_queued = false;
    priv->requested_rumble = false;
    for (int i = 0; i < 3; i++) {
        priv->imu_cal.accel_offset[i] = 0;
        priv->imu_cal.accel_scale[i] = 0x4000;
        priv->imu_cal.gyro_offset[i] = 0;
        /* Value taken from Linux driver */
        priv->imu_cal.gyro_scale[i] = 13371;
    }

    ns_init_step(device);
    return 0;
}

static int ns_driver_ops_set_leds(egc_input_device_t *device, u32 leds)
{
    struct ns_private_data_t *priv = (void *)device->private_data;

    priv->requested_leds = leds;
    priv->led_change_queued = true;
    return 0;
}

static int ns_driver_ops_set_rumble(egc_input_device_t *device, bool rumble_on)
{
    struct ns_private_data_t *priv = (void *)device->private_data;
    if (rumble_on != priv->requested_rumble) {
        priv->requested_rumble = rumble_on;
    }
    return 0;
}

const egc_device_driver_t ns_usb_device_driver = {
    .probe = ns_driver_ops_probe,
    .init = ns_driver_ops_init,
    .set_leds = ns_driver_ops_set_leds,
    .set_rumble = ns_driver_ops_set_rumble,
};

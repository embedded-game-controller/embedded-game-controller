#include <limits.h>
#include <math.h>

#include "driver_api.h"
#include "generic_mappings.h"
#include "utils.h"
#include "wiimote.h"

#define WM_VID_NINTENDO 0x057e
#define WM_PID_WIIMOTE  0x0306

#define WM_CMD_SET_LEDS        0x11
#define WM_CMD_SET_REPORT_TYPE 0x12
#define WM_CMD_IR_ENABLE_CLOCK 0x13
#define WM_CMD_REQ_STATUS      0x15
#define WM_CMD_WRITE_DATA      0x16
#define WM_CMD_READ_DATA       0x17
#define WM_CMD_IR_ENABLE_LOGIC 0x1a

#define WM_CMD_FLAG_RUMBLE 0x01
#define WM_CMD_FLAG_ACK    0x02
#define WM_CMD_FLAG_ENABLE 0x04

/* Report types */
#define WM_REP_STATUS         0x20
#define WM_REP_READ           0x21
#define WM_REP_ACK            0x22
#define WM_REP_BTN            0x30
#define WM_REP_BTN_ACC        0x31
#define WM_REP_BTN_EXP8       0x32
#define WM_REP_BTN_ACC_IR     0x33
#define WM_REP_BTN_EXP        0x34
#define WM_REP_BTN_ACC_EXP    0x35
#define WM_REP_BTN_IR_EXP     0x36
#define WM_REP_BTN_ACC_IR_EXP 0x37
#define WM_REP_EXP            0x3d

#define WM_REP_STATUS_LEN         6
#define WM_REP_READ_LEN           5
#define WM_REP_ACK_LEN            4
#define WM_REP_BTN_LEN            2
#define WM_REP_BTN_ACC_LEN        5
#define WM_REP_IR_BASIC_LEN       10
#define WM_REP_BTN_ACC_IR_EXP_LEN (WM_REP_BTN_ACC_LEN + WM_REP_IR_BASIC_LEN + 6)

#define WM_STATUS_BATTERY_CRITICAL 0x1
#define WM_STATUS_EXP_ATTACHED     0x2
#define WM_STATUS_SPEAKER_ENABLED  0x4
#define WM_STATUS_IR_ENABLED       0x8

#define WM_IR_MODE_BASIC    1
#define WM_IR_MODE_EXTENDED 3
#define WM_IR_MODE_FULL     5

/* Register addresses */
#define WM_REG_EXP_CALIBRATION 0x04a40020
#define WM_REG_EXP_DECRYPT1    0x04a400f0
#define WM_REG_EXP_DECRYPT2    0x04a400fb
#define WM_REG_EXP_TYPE        0x04a400fa
#define WM_REG_MP_CALIBRATION  0x04a60020
#define WM_REG_MP_INIT         0x04a600f0
#define WM_REG_MP_STATUS       0x04a600fe
#define WM_REG_IR_BLOCK1       0x04b00000
#define WM_REG_IR_BLOCK2       0x04b0001a
#define WM_REG_IR_OP           0x04b00030
#define WM_REG_IR_MODE         0x04b00033

#define WM_REG_EXP_CALIBRATION_LEN 16

/* Memory addresses */
#define WM_MEM_CALIBRATION 0x00000016

#define WM_MEM_CALIBRATION_LEN 10

/* Haven't given this number mush thought, we might need to tweak it later */
#define WM_BAR_OFFSET_Y 150

#define WM_NUM_ATTEMPTS_DEFAULT 8

static const u8 s_elements_wiimote_btn[] = {
    /* clang-format off */
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_START,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_DPAD_UP,
        EGC_GAMEPAD_BUTTON_DPAD_DOWN,
        EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
        EGC_GAMEPAD_BUTTON_DPAD_LEFT,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_GUIDE,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_BACK,
    EGC_INPUT_REPORT_TYPE_BUTTON4, /* A - B - 1 - 2 */
        EGC_GAMEPAD_BUTTON_EAST,
        EGC_GAMEPAD_BUTTON_SOUTH,
        EGC_GAMEPAD_BUTTON_WEST,
        EGC_GAMEPAD_BUTTON_NORTH,
    EGC_INPUT_REPORT_TYPE_END
    /* clang-format on */
};

static const u8 s_elements_classic_btn[] = {
    /* clang-format off */
    EGC_INPUT_REPORT_TYPE_BUTTON4_INVERTED,
        EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
        EGC_GAMEPAD_BUTTON_DPAD_DOWN,
        EGC_GAMEPAD_BUTTON_LEFT_TRIGGER,
        EGC_GAMEPAD_BUTTON_BACK,
    EGC_INPUT_REPORT_TYPE_BUTTON4_INVERTED,
        EGC_GAMEPAD_BUTTON_GUIDE,
        EGC_GAMEPAD_BUTTON_START,
        EGC_GAMEPAD_BUTTON_RIGHT_TRIGGER,
        EGC_GAMEPAD_BUTTON_INVALID,
    EGC_INPUT_REPORT_TYPE_BUTTON4_INVERTED,
        EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
        EGC_GAMEPAD_BUTTON_SOUTH,
        EGC_GAMEPAD_BUTTON_WEST,
        EGC_GAMEPAD_BUTTON_EAST,
    EGC_INPUT_REPORT_TYPE_BUTTON4_INVERTED,
        EGC_GAMEPAD_BUTTON_NORTH,
        EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
        EGC_GAMEPAD_BUTTON_DPAD_LEFT,
        EGC_GAMEPAD_BUTTON_DPAD_UP,
    EGC_INPUT_REPORT_TYPE_END
    /* clang-format on */
};

typedef enum ATTRIBUTE_PACKED {
    WM_STATE_IDLE = 0,

    WM_STATE_CALIBRATION,
    WM_STATE_LEDS_REQ,
    WM_STATE_RUMBLE_REQ,
    WM_STATE_REPORT_REQ,
    WM_STATE_HANDSHAKE_READ_CALIBRATION,
    WM_STATE_HANDSHAKE_COMPLETE,

    WM_STATE_EXP_FIRST = 10,
    WM_STATE_EXP_CHECK_ENCRYPTED = WM_STATE_EXP_FIRST,
    /* These next two steps are for disabling encryption */
    WM_STATE_EXP_DECRYPT_1,
    WM_STATE_EXP_DECRYPT_2,
    WM_STATE_EXP_IDENTIFICATION,
    WM_STATE_EXP_READ_CALIBRATION,
    WM_STATE_EXP_LAST = WM_STATE_EXP_READ_CALIBRATION,

    /* IR states */
    WM_STATE_IR_FIRST = 20,
    WM_STATE_IR_ENABLE_CLOCK = WM_STATE_IR_FIRST,
    WM_STATE_IR_ENABLE_LOGIC,
    WM_STATE_IR_SENSITIVITY_1,
    WM_STATE_IR_SENSITIVITY_2,
    WM_STATE_IR_SENSITIVITY_3,
    WM_STATE_IR_SET_MODE,
    WM_STATE_IR_SET_OP2,
    WM_STATE_IR_LAST = WM_STATE_IR_SET_OP2,

    /* Motion Plus states: */
    WM_STATE_MP_FIRST = 30,
    WM_STATE_MP_PROBE = WM_STATE_MP_FIRST,
    WM_STATE_MP_INITIALIZING,
    WM_STATE_MP_ENABLING,
    WM_STATE_MP_LAST = WM_STATE_MP_ENABLING,
    /* TODO: expose the Motion+ disable operation to the client? */

    /* Currently not used, since EGC does not support playing sounds yet */
    WM_STATE_SPEAKER_FIRST = 40,
    WM_STATE_SPEAKER_ENABLING_1,
    WM_STATE_SPEAKER_ENABLING_2,
    WM_STATE_SPEAKER_ENABLING_3,
    WM_STATE_SPEAKER_ENABLING_4,
    WM_STATE_SPEAKER_ENABLING_5,
    WM_STATE_SPEAKER_ENABLING_6,
    WM_STATE_SPEAKER_DISABLING,
    WM_STATE_SPEAKER_LAST = WM_STATE_SPEAKER_DISABLING,
} WmState;

struct wm_calibration_accel_t {
    u16 zero[3];
    u16 g_force[3];
};

struct wm_calibration_gyro_t {
    u16 zero[3];
    /* Scale value when the controller is rotating with a speed of 1 radian per second */
    u16 pi_sec[3];
};

struct wm_calibration_axis_t {
    u8 max;
    u8 min;
    u8 center;
} ATTRIBUTE_PACKED;

struct wm_private_data_t {
    WmState state;
    u8 requested_leds : 4;
    u8 leds : 4;

    bool status_pending : 1;
    bool calibrated : 1;
    bool requested_rumble : 1;
    bool active_rumble : 1;

    bool exp_attached : 1;
    bool exp_ready : 1;
    bool exp_motion_plus : 1;
    bool motion_plus_probed : 1;

    bool motion_plus_enabled : 1;
    bool ir_requested : 1;
    bool ir_enabled : 1;
    bool report_type_requested : 1;

    EgcWiimoteExpType exp_type : 4;
    u8 remaining_attempts : 4;
    /* The highest bit of x tell whether the point is valid; the next 2
     * bits hold the index where the point was found */
    egc_point_t ir_points_prev[2];
    s16 ir_bar_width_prev;
    struct wm_calibration_accel_t cal ATTRIBUTE_ALIGN(2);
    union {
        struct wm_exp_cal_motion_plus_t {
            struct wm_calibration_gyro_t gyro;
        } motion_plus;
        struct wm_exp_cal_nunchuck_t {
            struct wm_calibration_accel_t accel;
            struct wm_calibration_axis_t stick_x;
            struct wm_calibration_axis_t stick_y;
        } nunchuck;
        struct wm_exp_cal_classic_t {
            struct wm_calibration_axis_t l_stick_x;
            struct wm_calibration_axis_t l_stick_y;
            struct wm_calibration_axis_t r_stick_x;
            struct wm_calibration_axis_t r_stick_y;
        } classic;
    } exp_cal;
} ATTRIBUTE_PACKED;
static_assert(sizeof(struct wm_private_data_t) <= EGC_INPUT_DEVICE_DRIVER_DATA_SIZE);
#define PRIV(input_device) ((struct wm_private_data_t *)get_priv(input_device)->private_data)

static const egc_device_description_t s_device_description_wiimote = {
    .vendor_id = WM_VID_NINTENDO,
    .product_id = WM_PID_WIIMOTE,
    /* clang-format off */
    .available_buttons =
        BIT(EGC_GAMEPAD_BUTTON_DPAD_UP) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_DOWN) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_LEFT) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_RIGHT) |
        /* Mapping copied from SDL's SDL_hidapi_wii.c */
        BIT(EGC_GAMEPAD_BUTTON_NORTH) | /* 2 */
        BIT(EGC_GAMEPAD_BUTTON_EAST) | /* A */
        BIT(EGC_GAMEPAD_BUTTON_SOUTH) | /* B */
        BIT(EGC_GAMEPAD_BUTTON_WEST) | /* 1 */
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_GUIDE) |
        BIT(EGC_GAMEPAD_BUTTON_BACK),
    .available_axes = 0,
    /* clang-format on */
    .type = EGC_DEVICE_TYPE_GAMEPAD,
    .num_touch_points = 1,
    .num_leds = 4,
    .num_accelerometers = 1,
    .has_rumble = true,
};

/* We use a union since these operations cannot be done at the same time */
static union {
    EgcDriverWiimoteReadDataCb read_data;
    EgcDriverWiimoteWriteDataCb write_data;
} s_client_cb;
static bool s_calibration_enabled = true;
static s16 s_bar_offset_y = WM_BAR_OFFSET_Y;

static int wm_step(egc_input_device_t *device);

static inline int wm_send(egc_input_device_t *device, u8 *data, int size)
{
    // EGC_DEBUG_DATA(data, size);
    return egc_device_driver_send_output_report(device, data, size);
}

static int wm_send_command(egc_input_device_t *device, u8 *data, int size)
{
    struct wm_private_data_t *priv = PRIV(device);
    if (priv->requested_rumble)
        data[1] |= WM_CMD_FLAG_RUMBLE;
    int rc = wm_send(device, data, size);
    if (rc >= 0) {
        priv->active_rumble = priv->requested_rumble;
    }
    return rc;
}

static int wm_send_report(egc_input_device_t *device, u8 report, u8 *data, int size)
{
    data[0] = report;
    return wm_send_command(device, data, size);
}

static int wm_write_data(egc_input_device_t *device, u32 offset, const void *data, u8 size)
{
    u8 msg[1 + 4 + 1 + 16], *ptr;

    if (size > 16)
        return -1;

    ptr = msg + 1;
    *(u32 *)ptr = htobe32(offset);
    ptr += 4;
    *(ptr++) = size;
    memcpy(ptr, data, size);
    ptr += size;
    memset(ptr, 0, 16 - size);
    return wm_send_report(device, WM_CMD_WRITE_DATA, msg, sizeof(msg));
}

static int wm_read_data(egc_input_device_t *device, u32 offset, u16 size)
{
    u8 msg[1 + 4 + 2], *ptr;

    ptr = msg + 1;
    *(u32 *)ptr = htobe32(offset);
    ptr += 4;
    *(u16 *)ptr = htobe16(size);
    return wm_send_report(device, WM_CMD_READ_DATA, msg, sizeof(msg));
}

static int wm_set_leds(egc_input_device_t *device, u8 leds)
{
    u8 data[2];
    data[1] = (leds << 4) | WM_CMD_FLAG_ACK;
    return wm_send_report(device, WM_CMD_SET_LEDS, data, sizeof(data));
}

static int wm_ir_step(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);
    u8 data[2];
    int rc = -1;

    if (priv->state == WM_STATE_IR_ENABLE_CLOCK) {
        data[1] = WM_CMD_FLAG_ACK | (priv->ir_requested ? WM_CMD_FLAG_ENABLE : 0);
        rc = wm_send_report(device, WM_CMD_IR_ENABLE_CLOCK, data, sizeof(data));
    } else if (priv->state == WM_STATE_IR_ENABLE_LOGIC) {
        data[1] = WM_CMD_FLAG_ACK | (priv->ir_requested ? WM_CMD_FLAG_ENABLE : 0);
        rc = wm_send_report(device, WM_CMD_IR_ENABLE_LOGIC, data, sizeof(data));
    } else if (priv->state == WM_STATE_IR_SENSITIVITY_1) {
        u8 value = 1;
        rc = wm_write_data(device, WM_REG_IR_OP, &value, sizeof(value));
    } else if (priv->state == WM_STATE_IR_SENSITIVITY_2) {
        /* Middle sensitivity, from https://wiibrew.org/wiki/Wiimote#IR_Camera */
        static const u8 block1[] = {
            0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0xaa, 0x00, 0x64,
        };
        rc = wm_write_data(device, WM_REG_IR_BLOCK1, block1, sizeof(block1));
    } else if (priv->state == WM_STATE_IR_SENSITIVITY_3) {
        static const u8 block2[] = {
            0x63,
            0x03,
        };
        rc = wm_write_data(device, WM_REG_IR_BLOCK2, block2, sizeof(block2));
    } else if (priv->state == WM_STATE_IR_SET_MODE) {
        u8 value = WM_IR_MODE_BASIC;
        rc = wm_write_data(device, WM_REG_IR_MODE, &value, sizeof(value));
    } else if (priv->state == WM_STATE_IR_SET_OP2) {
        u8 value = 8;
        rc = wm_write_data(device, WM_REG_IR_OP, &value, sizeof(value));
    }

    EGC_DEBUG("now state %d, rc %d", priv->state, rc);
    return rc;
}

static int wm_request_status(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);

    if (priv->status_pending)
        return 0;

    u8 data[2];
    data[1] = 0;
    int rc = wm_send_report(device, WM_CMD_REQ_STATUS, data, sizeof(data));
    if (rc >= 0) {
        priv->status_pending = true;
    }
    return rc;
}

static int wm_mp_step(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);
    int rc = -1;

    if (priv->state == WM_STATE_MP_PROBE) {
        rc = wm_read_data(device, WM_REG_MP_STATUS, 2);
    } else if (priv->state == WM_STATE_MP_INITIALIZING) {
        u8 value = 0x55;
        rc = wm_write_data(device, WM_REG_MP_INIT, &value, sizeof(value));
    } else if (priv->state == WM_STATE_MP_ENABLING) {
        u8 value = 0x04;
        rc = wm_write_data(device, WM_REG_MP_STATUS, &value, sizeof(value));
    }

    return rc;
}

static int wm_set_report_type(egc_input_device_t *device, u8 report_type)
{
    u8 data[3];
    data[1] = WM_CMD_FLAG_ACK | WM_CMD_FLAG_ENABLE; /* enable continuous reporting */
    data[2] = report_type;
    return wm_send_report(device, WM_CMD_SET_REPORT_TYPE, data, sizeof(data));
}

static void wm_parse_3u16(const u8 *data, u16 *values)
{
    values[0] = (data[0] << 2) | ((data[3] >> 4) & 0x3);
    values[1] = (data[1] << 2) | ((data[3] >> 2) & 0x3);
    values[2] = (data[2] << 2) | ((data[3] >> 0) & 0x3);
}

static s16 wm_axis_value(u8 raw, const struct wm_calibration_axis_t *cal)
{
    int val;
    if (raw > cal->center) {
        val = (raw - cal->center) * INT16_MAX / (cal->max - cal->center);
        if (val > INT16_MAX)
            val = INT16_MAX;
    } else {
        val = (cal->center - raw) * INT16_MIN / (cal->center - cal->min);
        if (val < INT16_MIN)
            val = INT16_MIN;
    }
    return val;
}

static inline s16 wm_accel_value(s16 raw, const struct wm_calibration_accel_t *cal, int index)
{
    return (raw - cal->zero[index]) * EGC_ACCELEROMETER_RES_PER_G /
           (cal->g_force[index] - cal->zero[index]);
}

/* This checksum mechanism is used for the calibration data */
static bool wm_checksum(const u8 *data, int size)
{
    u8 checksum = 0x55;
    for (int i = 0; i < size - 1; i++) {
        checksum += data[i];
    }
    if (checksum != data[size - 1]) {
        EGC_DEBUG("Wrong checksum, %02x vs %02x", checksum, data[size - 1]);
    }
    return checksum == data[size - 1];
}

static void wm_parse_buttons(const u8 *data, egc_input_state_t *state)
{
    egc_device_driver_parse_report(data, s_elements_wiimote_btn, state);
}

static void wm_parse_accel(egc_input_device_t *device, const u8 *data,
                           const struct wm_calibration_accel_t *cal, egc_input_state_t *state)
{
    s16 x = (data[2] << 2) | ((data[0] >> 5) & 0x3);
    s16 y = (data[3] << 2) | ((data[1] >> 4) & 0x2);
    s16 z = (data[4] << 2) | ((data[1] >> 5) & 0x2);

    egc_accelerometer_t *accel = egc_device_driver_get_accelerometer(device, state, 0);
    accel->x = wm_accel_value(x, cal, 0);
    accel->y = wm_accel_value(z, cal, 2);
    accel->z = -wm_accel_value(y, cal, 1);
}

static float compute_angle(egc_point_t a, egc_point_t b, int *distance_squared)
{
    int diff_x = b.x - a.x;
    int diff_y = b.y - a.y;
    float angle = atan2f(diff_y, diff_x);
    if (distance_squared) {
        *distance_squared = diff_x * diff_x + diff_y * diff_y;
    }
    return angle;
}

/* Returns:
 * 0: not found
 * 1: found, tracking just one point
 * 2: found, tracking two points
 */
static int wm_ir_find_bar(egc_input_device_t *device, const egc_point_t *points, float wiimote_roll,
                          egc_point_t *bar_points, float *ir_roll, float *bar_width)
{
    struct wm_private_data_t *priv = PRIV(device);

    /* When we are sure of a point, we'll put its index in here */
    int new_camera_index[2] = { -1, -1 };
    float new_roll = NAN;
    int new_width_squared = -1;

    /* First, prepare the information about the previous valid points */
    int num_points_prev = 0;
    int points_prev_camera_index[2];
    egc_point_t points_prev[2];
    for (int i = 0; i < 2; i++) {
        egc_point_t p = priv->ir_points_prev[i];
        if (p.x & 0x8000) {
            points_prev[i].x = p.x & 0x3ff;
            points_prev[i].y = p.y;
            points_prev_camera_index[i] = (p.x >> 13) & 0x3;
            num_points_prev++;
        } else {
            points_prev[i].x = points_prev[i].y = -1;
            points_prev_camera_index[i] = -1;
        }
    }

    /* Check if they are still valid */
    int num_found_points = 0;
    if (points_prev_camera_index[0] >= 0 && points_prev_camera_index[1] >= 0 &&
        points[points_prev_camera_index[0]].x != 0x3ff &&
        points[points_prev_camera_index[1]].x != 0x3ff) {
        egc_point_t candidates[2];
        for (int i = 0; i < 2; i++) {
            candidates[i] = points[points_prev_camera_index[i]];
            int diff_x = candidates[i].x - points_prev[i].x;
            int diff_y = candidates[i].y - points_prev[i].y;
            int distance_squared = diff_x * diff_x + diff_y * diff_y;
            /* Accept a move up to 100 pixels */
            if (distance_squared < 10000) {
                new_camera_index[num_found_points++] = points_prev_camera_index[i];
            }
        }
    }

    if (num_found_points == 2) {
        goto done;
    }

    int num_points = 0;
    int valid_point_index = -1;
    for (int i = 0; i < 4; i++) {
        if (points[i].x != 0x3ff) {
            num_points++;
            /* this variable is only used when we see a single point, so it
             * doesn't matter if we execute this line more than once */
            valid_point_index = i;
        }
    }
    if (num_points == 0 || (num_points == 1 && num_points_prev == 0))
        goto not_found;

    if (num_points >= 2) {
        /* Try all the combinations and find out the most likely one. Assuming that
         * all points were valid, we'd have 6 possible combinations (since the
         * order does not matter). */
        int best_weight = INT_MAX;
        int fixed_index = num_found_points > 0 ? new_camera_index[0] : -1;
        for (int i = 0; i < num_points; i++) {
            for (int j = i + 1; j < num_points; j++) {
                if (fixed_index >= 0) {
                    /* At least one of the points must be the ne we found */
                    if (i != fixed_index && j != fixed_index)
                        continue;
                }
                egc_point_t a = points[i];
                egc_point_t b = points[j];
                int distance_squared;
                float angle = compute_angle(a, b, &distance_squared);
                float angle_diff = fmodf(angle - wiimote_roll, M_PI);
                if (angle_diff < 0)
                    angle_diff += M_PI;
                if (angle_diff > M_PI / 2) {
                    angle_diff = M_PI - angle_diff;
                }
                /* We prefer that the angle is as close as possible to the
                 * accelerometer, and that the distance between the points is
                 * short.
                 * Transform the angle from float to int, pick 1024 as weight to
                 * keep the math simple (other numbers would work too) */
                int angle_int = angle_diff * 1024;
                int weight = angle_int * angle_int + distance_squared;
                if (weight < best_weight) {
                    new_camera_index[0] = i;
                    new_camera_index[1] = j;
                    new_roll = angle;
                    new_width_squared = distance_squared;
                    best_weight = weight;
                    num_found_points = 2;
                }
            }
        }
    } else { /* num_points == 1 */
        /* Check which of the previous points this is. We always store the
         * priv->ir_points_prev points so that index 0 is the left one of the
         * status bar, and index 1 is the right one. So finding out which one
         * we see is important to understand our current position. */
        int prev_valid_point_index = -1;
        egc_point_t point = points[valid_point_index];
        if (num_found_points == 1) {
            prev_valid_point_index = points_prev_camera_index[0] >= 0 ? 0 : 1;
        } else {
            /* Pick the closest one */
            int min_distance = INT_MAX;
            for (int i = 0; i < 2; i++) {
                if (points_prev_camera_index[i] < 0)
                    continue;
                int diff_x = point.x - points_prev[i].x;
                int diff_y = point.y - points_prev[i].y;
                int distance_squared = diff_x * diff_x + diff_y * diff_y;
                if (distance_squared < min_distance) {
                    prev_valid_point_index = i;
                    min_distance = distance_squared;
                }
            }
        }

        int hidden_point_index = (prev_valid_point_index + 1) % 2;
        bar_points[prev_valid_point_index] = point;
        int valid_camera_index = points_prev_camera_index[prev_valid_point_index];
        priv->ir_points_prev[prev_valid_point_index].x =
            0x8000 | valid_camera_index << 13 | point.x;
        priv->ir_points_prev[prev_valid_point_index].y = point.y;
        priv->ir_points_prev[hidden_point_index].x = 0;
        /* Compute the position of the hidden point */
        int dx = cosf(wiimote_roll) * priv->ir_bar_width_prev;
        int dy = sinf(wiimote_roll) * priv->ir_bar_width_prev;
        if (hidden_point_index == 0) {
            dx = -dx;
            dy = -dy;
        }
        bar_points[hidden_point_index].x = point.x + dx;
        bar_points[hidden_point_index].y = point.y + dy;

        *ir_roll = wiimote_roll;
        *bar_width = priv->ir_bar_width_prev;
        return 1;
    }

done:
    for (int i = 0; i < num_found_points; i++) {
        egc_point_t p = points[new_camera_index[i]];
        bar_points[i] = p;

        /* Also save them in the private structure for the next iteration */
        priv->ir_points_prev[i].x = 0x8000 | new_camera_index[i] << 13 | p.x;
        priv->ir_points_prev[i].y = p.y;
    }
    *ir_roll = isnan(new_roll) ? compute_angle(bar_points[0], bar_points[1], &new_width_squared)
                               : new_roll;
    *bar_width = sqrtf(new_width_squared);
    priv->ir_bar_width_prev = *bar_width;
    return 2;

not_found:
    /* Clear the previous points */
    priv->ir_points_prev[0].x = priv->ir_points_prev[1].x = 0;
    return 0;
}

static void wm_ir_resolve(egc_input_device_t *device, egc_point_t *points, egc_input_state_t *state)
{
    struct wm_private_data_t *priv = PRIV(device);

    egc_point_t bar_points[2];
    float ir_roll = 0.0f;
    float bar_width;
    egc_accelerometer_t *accel = egc_device_driver_get_accelerometer(device, state, 0);
    float accel_roll = atan2f(accel->x, accel->y);
    int found = wm_ir_find_bar(device, points, accel_roll, bar_points, &ir_roll, &bar_width);
    if (!found) {
        egc_point_t invalid = { -1, -1 };
        egc_device_driver_set_touch_point(device, state, 0, invalid);
        return;
    }

    float diff = ir_roll - accel_roll;
    if (diff < 0)
        diff += M_PI * 2;
    if (diff > M_PI / 2 && diff < M_PI * 3 / 2) {
        ir_roll += M_PI;
        if (ir_roll > M_PI) {
            ir_roll -= M_PI * 2;
        }
        egc_point_t tmp = bar_points[0];
        bar_points[0] = bar_points[1];
        bar_points[1] = tmp;

        tmp = priv->ir_points_prev[0];
        priv->ir_points_prev[0] = priv->ir_points_prev[1];
        priv->ir_points_prev[1] = tmp;
    }
    /* Rotate the first point around the camera center */
    const int camera_width = 0x3ff;
    const int camera_height = 780;
    float center_x = (camera_width - 1) / 2.0f;
    float center_y = camera_height / 2.0f;
    /* Use the center as origin */
    float p0x = bar_points[0].x - center_x;
    float p0y = bar_points[0].y - center_y;
    /* Rotate the point */
    float sine = sinf(ir_roll);
    float cosine = cosf(ir_roll);
    float r0x = cosine * p0x + sine * p0y;
    float r0y = -sine * p0x + cosine * p0y;

    float px = -r0x - bar_width / 2;
    float py = r0y;

    /* Map the coordinates to the range [0 - EGC_GAMEPAD_TOUCH_RES], taking
     * into account that the coordinates we obtained are inversely proportinal
     * to the distance (that is, they are directly proportional to bar_width).
     * The scale factor is chosen in such a way that we can cover most of the
     * [-1,1] range while tracking two points.
     */
    const float scale_factor_x = 0.01f;
    const float scale_factor_y = 0.015f;
    px = px * scale_factor_x * bar_width / camera_width;
    py = (py + s_bar_offset_y) * scale_factor_y * bar_width / camera_height;
    /* If we are beyong the border, keep the cursor within the range */
    if (px < -1.0f) {
        px = -1.0f;
        py /= -px;
    } else if (px > 1.0f) {
        px = 1.0f;
        py /= px;
    }
    if (py < -1.0f) {
        px /= -py;
        py = -1.0f;
    } else if (py > 1.0f) {
        px /= py;
        py = 1.0f;
    }

    /* Now map the [-1,1] range to [0, EGC_GAMEPAD_TOUCH_RES] */
    int half_res = (EGC_GAMEPAD_TOUCH_RES + 1) / 2;
    int x = ceilf(px * half_res) + half_res;
    int y = ceilf(py * half_res) + half_res;
    if (x > EGC_GAMEPAD_TOUCH_RES)
        x = EGC_GAMEPAD_TOUCH_RES;
    if (y > EGC_GAMEPAD_TOUCH_RES)
        y = EGC_GAMEPAD_TOUCH_RES;
    egc_point_t point = { x, y };
    egc_device_driver_set_touch_point(device, state, 0, point);
}

static void wm_parse_ir_basic(const u8 *data, egc_point_t *points)
{
    for (int i = 0; i < 2; i++, data += 5) {
        points[i * 2].x = ((data[2] & 0x30) << 4) | data[0];
        points[i * 2].y = ((data[2] & 0xc0) << 2) | data[1];
        points[i * 2 + 1].x = ((data[2] & 0x03) << 8) | data[3];
        points[i * 2 + 1].y = ((data[2] & 0x0c) << 6) | data[4];
    }
}

static int wm_expansion_step(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);
    int rc = -1;

    if (priv->state == WM_STATE_EXP_DECRYPT_1) {
        u8 value = 0x55;
        rc = wm_write_data(device, WM_REG_EXP_DECRYPT1, &value, sizeof(value));
        /* This operation disables the Motion+ */
        priv->motion_plus_enabled = false;
    } else if (priv->state == WM_STATE_EXP_DECRYPT_2) {
        u8 value = 0x0;
        rc = wm_write_data(device, WM_REG_EXP_DECRYPT2, &value, sizeof(value));
    } else if (priv->state == WM_STATE_EXP_IDENTIFICATION ||
               priv->state == WM_STATE_EXP_CHECK_ENCRYPTED) {
        rc = wm_read_data(device, WM_REG_EXP_TYPE, 6);
    } else if (priv->state == WM_STATE_EXP_READ_CALIBRATION) {
        u32 offset = ((priv->remaining_attempts / 2) % 2 == 0) ? 0 : WM_REG_EXP_CALIBRATION_LEN;
        rc = wm_read_data(device, WM_REG_EXP_CALIBRATION + offset, 16);
    }

    return rc;
}

static void wm_expansion_remove(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);
    priv->exp_ready = false;
    priv->exp_type = EGC_WIIMOTE_EXP_NONE;
    /* Return to the description of the lone wiimote */
    memcpy((struct egc_device_description_t *)device->desc, &s_device_description_wiimote,
           sizeof(s_device_description_wiimote));
}

static s16 wm_rotation_value(s16 raw, bool slow)
{
    /* According to
     * https://wiibrew.org/wiki/Wiimote/Extension_Controllers/Wii_Motion_Plus#Data_Format
     * we need the following conversions to get the rotation speed in degrees per second:
     * 1. Divide by 8192/595 (that is, multiply by 595/8192)
     * 2. In fast mode, multiply by 2000 / 440
     * To get the rotation speed in radians per second we then
     * 3. Multiply by PI/180, which we approximate as 31416 / (10000 * 180);
     *    that is, 1309 / 75000
     * Which some simplifications our formula becomes:
     *   v * 595 * 1309 / (8192 * 75000) = v * 119 * 1309 / (8192 * 15000)
     *   ≈ v * 126 / 100000
     */
    s16 value = (raw - 0x2000) * 126 * EGC_GYROSCOPE_RES / 100000;
    return slow ? value : (value * 2000 / 440);
}

static void wm_parse_exp(egc_input_device_t *device, const u8 *data, egc_input_state_t *state)
{
    struct wm_private_data_t *priv = PRIV(device);

    if (!priv->exp_ready) {
        return;
    }

    if (priv->exp_type == EGC_WIIMOTE_EXP_NUNCHUCK) {
        if (!(data[5] & 0x1)) {
            egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFT_TRIGGER, INT16_MAX);
        }
        if (!(data[5] & 0x2)) {
            egc_device_driver_set_button(state, EGC_GAMEPAD_BUTTON_LEFT_SHOULDER);
        }

        s16 value = wm_axis_value(data[0], &priv->exp_cal.nunchuck.stick_x);
        egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFTX, value);

        value = wm_axis_value(data[1], &priv->exp_cal.nunchuck.stick_y);
        egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFTY, -1 - value);

        egc_accelerometer_t *accel = egc_device_driver_get_accelerometer(device, state, 1);
        s16 x = (data[2] << 2) | ((data[5] >> 2) & 0x3);
        s16 y = (data[3] << 2) | ((data[5] >> 4) & 0x3);
        s16 z = (data[4] << 2) | ((data[5] >> 6) & 0x3);
        struct wm_calibration_accel_t *ac = &priv->exp_cal.nunchuck.accel;
        accel->x = wm_accel_value(x, ac, 0);
        accel->y = wm_accel_value(z, ac, 2);
        accel->z = -wm_accel_value(y, ac, 1);
    } else if (priv->exp_type == EGC_WIIMOTE_EXP_CLASSIC ||
               priv->exp_type == EGC_WIIMOTE_EXP_CLASSIC_PRO) {
        s16 value = wm_axis_value(data[0] & 0x3f, &priv->exp_cal.classic.l_stick_x);
        egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFTX, value);
        value = wm_axis_value(data[1] & 0x3f, &priv->exp_cal.classic.l_stick_y);
        egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFTY, -1 - value);

        value = ((data[0] >> 3) & 0x18) | ((data[1] >> 5) & 0x6) | (data[2] >> 7);
        value = wm_axis_value(value, &priv->exp_cal.classic.r_stick_x);
        egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_RIGHTX, value);
        value = wm_axis_value(data[2] & 0x1f, &priv->exp_cal.classic.r_stick_y);
        egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_RIGHTY, -1 - value);

        value = ((data[2] >> 2) & 0x18) | ((data[3] >> 5) & 0x7);
        value = value * INT16_MAX / 0x1f;
        egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFT_TRIGGER, value);
        value = data[3] & 0x1f;
        value = value * INT16_MAX / 0x1f;
        egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_RIGHT_TRIGGER, value);

        egc_device_driver_parse_report(data + 4, s_elements_classic_btn, state);
    } else if (priv->exp_type == EGC_WIIMOTE_EXP_MOTION_PLUS) {
        bool exp_connected = data[4] & 0x01;
        if (exp_connected && priv->state == WM_STATE_IDLE) {
            /* Disable the Motion+ and enable the connected expansion.
             * In the future we might want a passthrough mode instead. */
            wm_expansion_remove(device);
            priv->state = WM_STATE_EXP_DECRYPT_1;
            wm_step(device);
        }

        bool slow_yaw = (data[3] & 0x02) != 0;
        bool slow_pitch = (data[3] & 0x01) != 0;
        bool slow_roll = (data[4] & 0x02) != 0;
        egc_gyroscope_t *gyro = egc_device_driver_get_gyroscope(device, state, 0);
        s16 value = ((data[3] & 0xfc) << 6) | data[0]; /* Yaw */
        gyro->y = wm_rotation_value(value, slow_yaw);
        value = ((data[4] & 0xfc) << 6) | data[1]; /* Roll */
        gyro->z = wm_rotation_value(value, slow_roll);
        value = ((data[5] & 0xfc) << 6) | data[2]; /* Pitch */
        gyro->x = -wm_rotation_value(value, slow_pitch);
    }
}

static bool wm_expansion_calibration_parse(egc_input_device_t *device, const u8 *data)
{
    struct wm_private_data_t *priv = PRIV(device);

    if (priv->exp_type == EGC_WIIMOTE_EXP_NUNCHUCK) {
        if (!wm_checksum(data, 15)) {
            return false;
        }
        wm_parse_3u16(data, priv->exp_cal.nunchuck.accel.zero);
        wm_parse_3u16(data + 4, priv->exp_cal.nunchuck.accel.g_force);
        priv->exp_cal.nunchuck.stick_x.max = data[8];
        priv->exp_cal.nunchuck.stick_x.min = data[9];
        priv->exp_cal.nunchuck.stick_x.center = data[10];
        priv->exp_cal.nunchuck.stick_y.max = data[11];
        priv->exp_cal.nunchuck.stick_y.min = data[12];
        priv->exp_cal.nunchuck.stick_y.center = data[13];
    } else if (priv->exp_type == EGC_WIIMOTE_EXP_CLASSIC ||
               priv->exp_type == EGC_WIIMOTE_EXP_CLASSIC_PRO) {
        if (!wm_checksum(data, 15)) {
            return false;
        }
        struct wm_exp_cal_classic_t *cal = &priv->exp_cal.classic;
        cal->l_stick_x.max = data[0] >> 2;
        cal->l_stick_x.min = data[1] >> 2;
        cal->l_stick_x.center = data[2] >> 2;
        cal->l_stick_y.max = data[3] >> 2;
        cal->l_stick_y.min = data[4] >> 2;
        cal->l_stick_y.center = data[5] >> 2;
        cal->r_stick_x.max = data[6] >> 3;
        cal->r_stick_x.min = data[7] >> 3;
        cal->r_stick_x.center = data[8] >> 3;
        cal->r_stick_y.max = data[9] >> 3;
        cal->r_stick_y.min = data[10] >> 3;
        cal->r_stick_y.center = data[11] >> 3;
    } else if (priv->exp_type == EGC_WIIMOTE_EXP_MOTION_PLUS) {
        /* TODO: This data is garbage on my chinese replica */
    }

    return true;
}

static void wm_expansion_setup(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);
    egc_device_description_t *desc = (egc_device_description_t *)device->desc;

    /* Here we setup the default calibration and update the device description */
    if (priv->exp_type == EGC_WIIMOTE_EXP_NUNCHUCK) {
        struct wm_exp_cal_nunchuck_t *cal = &priv->exp_cal.nunchuck;
        cal->accel.zero[0] = cal->accel.zero[1] = cal->accel.zero[2] = 512;
        cal->accel.g_force[0] = cal->accel.g_force[1] = cal->accel.g_force[2] = 716;
        cal->stick_x.max = 256 - 32;
        cal->stick_x.min = 32;
        cal->stick_x.center = 128;
        cal->stick_y.max = 256 - 32;
        cal->stick_y.min = 32;
        cal->stick_y.center = 128;

        desc->available_buttons |= BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER);
        desc->available_axes |= BIT(EGC_GAMEPAD_AXIS_LEFT_TRIGGER) | BIT(EGC_GAMEPAD_AXIS_LEFTX) |
                                BIT(EGC_GAMEPAD_AXIS_LEFTY);
        desc->num_accelerometers = 2;
    } else if (priv->exp_type == EGC_WIIMOTE_EXP_CLASSIC ||
               priv->exp_type == EGC_WIIMOTE_EXP_CLASSIC_PRO) {
        struct wm_exp_cal_classic_t *cal = &priv->exp_cal.classic;
        cal->l_stick_x.min = cal->l_stick_y.min = 0;
        cal->l_stick_x.max = cal->l_stick_y.max = 63;
        cal->l_stick_x.center = cal->l_stick_y.center = 32;
        cal->r_stick_x.min = cal->r_stick_y.min = 0;
        cal->r_stick_x.max = cal->r_stick_y.max = 31;
        cal->r_stick_x.center = cal->r_stick_y.center = 16;

        /* clang-format off */
        desc->available_buttons |=
            BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER) |
            BIT(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER);
        desc->available_axes |=
            BIT(EGC_GAMEPAD_AXIS_LEFT_TRIGGER) | BIT(EGC_GAMEPAD_AXIS_RIGHT_TRIGGER) |
            BIT(EGC_GAMEPAD_AXIS_LEFTX) | BIT(EGC_GAMEPAD_AXIS_LEFTY) |
            BIT(EGC_GAMEPAD_AXIS_RIGHTX) | BIT(EGC_GAMEPAD_AXIS_RIGHTY);
        /* clang-format on */
    } else if (priv->exp_type == EGC_WIIMOTE_EXP_MOTION_PLUS) {
        desc->num_gyroscopes = 1;
    }
}

static bool wm_expansion_identify(egc_input_device_t *device, const u8 *data)
{
    struct wm_private_data_t *priv = PRIV(device);
    u16 id_minor = be16toh(*(u16 *)(data + 4));
    u16 id_major = be16toh(*(u16 *)(data));
    u32 full_id = (id_major << 16) | id_minor;
    switch (full_id) {
    case 0x00000000:
    case 0x00000505:
        priv->exp_type = EGC_WIIMOTE_EXP_NUNCHUCK;
        break;
    case 0x00000005:
        priv->exp_type = EGC_WIIMOTE_EXP_NONE;
        break;
    case 0x00000405:
        priv->exp_type = EGC_WIIMOTE_EXP_MOTION_PLUS;
        break;
    case 0x00000705:
    case 0x00000101:
        priv->exp_type = EGC_WIIMOTE_EXP_CLASSIC;
        break;
    case 0x01000101:
        priv->exp_type = EGC_WIIMOTE_EXP_CLASSIC_PRO;
        break;
    case 0xff000013:
        priv->exp_type = EGC_WIIMOTE_EXP_DRAWSOME_TABLET;
        break;
    case 0x00000103:
        priv->exp_type = EGC_WIIMOTE_EXP_GUITAR_HERO_3;
        break;
    case 0x01000103:
        priv->exp_type = EGC_WIIMOTE_EXP_GUITAR_HERO_DRUMS;
        break;
    case 0x03000103:
        priv->exp_type = EGC_WIIMOTE_EXP_DJ_HERO;
        break;
    case 0x00000111:
        priv->exp_type = EGC_WIIMOTE_EXP_TAIKO_DRUM;
        break;
    case 0xff000112:
        priv->exp_type = EGC_WIIMOTE_EXP_UDRAW_TABLET;
        break;
    case 0x00000310:
        priv->exp_type = EGC_WIIMOTE_EXP_SHINKANSEN;
        break;
    case 0x00000402:
        priv->exp_type = EGC_WIIMOTE_EXP_BALANCE_BOARD;
        break;
    default:
        return false;
    }

    return true;
}

static bool wm_calibration_parse(egc_input_device_t *device, const u8 *data)
{
    struct wm_private_data_t *priv = PRIV(device);

    if (!wm_checksum(data, WM_MEM_CALIBRATION_LEN)) {
        return false;
    }

    wm_parse_3u16(data, priv->cal.zero);
    wm_parse_3u16(data + 4, priv->cal.g_force);
    return true;
}

static int wm_calibration_read(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);
    /* Make two attempts on each offset, then try the other offset. This can be
     * repeated a few times, depending on how big remaining_attempts initially
     * was. */
    u32 offset = ((priv->remaining_attempts / 2) % 2 == 0) ? 0 : WM_MEM_CALIBRATION_LEN;
    return wm_read_data(device, WM_MEM_CALIBRATION + offset, WM_MEM_CALIBRATION_LEN);
}

static int wm_step(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);

    WmState old_state;
    do {
        EGC_DEBUG("state = %d", priv->state);
        old_state = priv->state;
        if (priv->state == WM_STATE_IDLE) {
            if (!priv->calibrated) {
                priv->state = WM_STATE_CALIBRATION;
                wm_calibration_read(device);
            } else if (priv->leds != priv->requested_leds) {
                priv->state = WM_STATE_LEDS_REQ;
            } else if (priv->ir_enabled != priv->ir_requested) {
                priv->state = WM_STATE_IR_FIRST;
            } else if (priv->exp_attached != priv->exp_ready) {
                if (!priv->exp_attached) {
                    /* If we disconnected an expansion and we have the Motion+,
                     * re-enable it */
                    bool enable_mp =
                        priv->exp_motion_plus && priv->exp_type != EGC_WIIMOTE_EXP_MOTION_PLUS;

                    wm_expansion_remove(device);
                    priv->state = enable_mp ? WM_STATE_MP_INITIALIZING : WM_STATE_REPORT_REQ;
                } else {
                    priv->state = WM_STATE_EXP_FIRST;
                }
            } else if (!priv->exp_attached && !priv->motion_plus_probed) {
                /* Wiibrew wiki says official games try up to three times. */
                priv->remaining_attempts = 3;
                priv->state = WM_STATE_MP_FIRST;
            } else if (priv->requested_rumble != priv->active_rumble) {
                priv->state = WM_STATE_RUMBLE_REQ;
            } else if (!priv->report_type_requested) {
                priv->state = WM_STATE_REPORT_REQ;
            }
        }

        if (priv->state >= WM_STATE_IR_FIRST && priv->state <= WM_STATE_IR_LAST) {
            wm_ir_step(device);
        } else if (priv->state >= WM_STATE_EXP_FIRST && priv->state <= WM_STATE_EXP_LAST) {
            wm_expansion_step(device);
        } else if (priv->state >= WM_STATE_MP_FIRST && priv->state <= WM_STATE_MP_LAST) {
            wm_mp_step(device);
        } else if (priv->state == WM_STATE_LEDS_REQ) {
            wm_set_leds(device, priv->requested_leds);
        } else if (priv->state == WM_STATE_RUMBLE_REQ) {
            if (priv->requested_rumble == priv->active_rumble) {
                priv->state = WM_STATE_IDLE;
            } else {
                wm_request_status(device);
            }
        } else if (priv->state == WM_STATE_REPORT_REQ) {
            wm_set_report_type(device, WM_REP_BTN_ACC_IR_EXP);
        }
    } while (priv->state == WM_STATE_IDLE && priv->state != old_state);
    return 0;
}

static void wm_step_next(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);
    priv->remaining_attempts = WM_NUM_ATTEMPTS_DEFAULT;

    if ((priv->state >= WM_STATE_IR_FIRST && priv->state < WM_STATE_IR_LAST) ||
        (priv->state >= WM_STATE_EXP_FIRST && priv->state < WM_STATE_EXP_LAST) ||
        (priv->state >= WM_STATE_MP_FIRST && priv->state < WM_STATE_MP_LAST)) {
        priv->state++;
    } else if (priv->state == WM_STATE_MP_LAST) {
        /* Initialize the Motion+ as a normal extension, but skip the decrypt
         * steps, as they disable the Motion+ */
        priv->state = WM_STATE_EXP_IDENTIFICATION;
    } else {
        priv->state = WM_STATE_IDLE;
    }
    wm_step(device);
}

static void wm_handle_error(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);

    EGC_DEBUG("state: %d", priv->state);
    if (priv->state >= WM_STATE_EXP_FIRST && priv->state <= WM_STATE_EXP_LAST &&
        /* Special case: failure to get the calibration data is not fatal */
        priv->state != WM_STATE_EXP_READ_CALIBRATION) {
        wm_expansion_remove(device);
        /* Set the ready state in order to avoid retrying the intialization forever */
        priv->exp_ready = priv->exp_attached;
    } else if (priv->state == WM_STATE_MP_PROBE) {
        priv->exp_motion_plus = false;
        priv->motion_plus_probed = true;
        priv->state = WM_STATE_IDLE;
    }
}

static bool wm_step_retry(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);

    if (priv->remaining_attempts == 0) {
        priv->remaining_attempts = WM_NUM_ATTEMPTS_DEFAULT;
        wm_handle_error(device);
        return false;
    }
    priv->remaining_attempts--;
    return wm_step(device) >= 0;
}

static void wm_read_cb(egc_input_device_t *device, const u8 *data, u8 actual_length)
{
    struct wm_private_data_t *priv = PRIV(device);

    u8 length = (data[0] >> 4) + 1;
    u8 error_code = data[0] & 0xf;
    if (s_client_cb.read_data) {
        EgcDriverWiimoteReadDataCb cb = s_client_cb.read_data;
        s_client_cb.read_data = NULL;
        cb(device, error_code, length, data + 3);
        return;
    }
    bool error = length > actual_length || error_code != 0;
    if (error) {
        if (!wm_step_retry(device)) {
            wm_step_next(device);
        }
        return;
    }
    data += 3;

    if (priv->state == WM_STATE_CALIBRATION) {
        if (!wm_calibration_parse(device, data)) {
            if (wm_step_retry(device))
                return;
        } else {
            priv->calibrated = true;
        }
    } else if (priv->state == WM_STATE_EXP_CHECK_ENCRYPTED) {
        if (wm_expansion_identify(device, data) && priv->exp_type == EGC_WIIMOTE_EXP_MOTION_PLUS) {
            EGC_DEBUG("Motion plus, skipping decryption");
            wm_expansion_setup(device);
            /* Pretend to be in the WM_STATE_EXP_IDENTIFICATION step in order
             * to skip decryption, which is not needed (and would disable the
             * Motion+) */
            priv->state = WM_STATE_EXP_IDENTIFICATION;
        }
    } else if (priv->state == WM_STATE_EXP_IDENTIFICATION) {
        if (!wm_expansion_identify(device, data)) {
            if (wm_step_retry(device))
                return;
        }
        wm_expansion_setup(device);
    } else if (priv->state == WM_STATE_EXP_READ_CALIBRATION) {
        if (s_calibration_enabled && !wm_expansion_calibration_parse(device, data)) {
            if (wm_step_retry(device))
                return;
        }
        priv->exp_ready = true;
    } else if (priv->state == WM_STATE_MP_PROBE) {
        priv->exp_motion_plus = data[1] == 0x5;
        priv->motion_plus_probed = true;
    }

    wm_step_next(device);
}

static void wm_status_cb(egc_input_device_t *device, const u8 *data)
{
    struct wm_private_data_t *priv = PRIV(device);

    /* We might have to request the report type again */
    priv->report_type_requested = false;

    u8 status = data[2];
    priv->leds = status >> 4;
    /* No use for this at the moment:
    priv->battery_critical = status & WM_STATUS_BATTERY_CRITICAL;
    */
    priv->exp_attached = status & WM_STATUS_EXP_ATTACHED;
    priv->ir_enabled = status & WM_STATUS_IR_ENABLED;

    wm_step(device);
}

static void wm_handle_ack(egc_input_device_t *device, const u8 *report)
{
    struct wm_private_data_t *priv = PRIV(device);

    u8 command = report[2];
    u8 error = report[3];
    /* When writing data to the expansion calibration area, "error" seems to
     * contain the number of bytes written */
    if (command != WM_CMD_WRITE_DATA && error != 0) {
        EGC_WARN("Error %02x on command %02x!", error, command);
        wm_step_retry(device);
        return;
    }
    if (command == WM_CMD_SET_LEDS) {
        priv->leds = priv->requested_leds;
        priv->state = WM_STATE_IDLE;
    } else if (command == WM_CMD_SET_REPORT_TYPE) {
        priv->report_type_requested = true;
        priv->state = WM_STATE_IDLE;
    } else if (priv->state == WM_STATE_IR_SET_OP2) {
        priv->ir_enabled = true;
    } else if (priv->state == WM_STATE_MP_ENABLING) {
        priv->motion_plus_enabled = true;
    } else if (s_client_cb.write_data) {
        EgcDriverWiimoteWriteDataCb cb = s_client_cb.write_data;
        s_client_cb.write_data = NULL;
        cb(device, error);
    }

    wm_step_next(device);
}

static void wm_driver_ops_intr_event(egc_input_device_t *device, const void *data, u16 length)
{
    struct wm_private_data_t *priv = PRIV(device);
    struct egc_input_state_t state = {
        0,
    };
    egc_point_t ir_points[4];
    bool has_ir = false;
    const u8 *report = data;

    // EGC_DEBUG_DATA(data, length);
    u8 report_type = report[0];
    report++;
    length--;
    switch (report_type) {
    case WM_REP_STATUS:
        if (length < WM_REP_STATUS_LEN)
            return;
        priv->status_pending = false;
        wm_status_cb(device, report);
        wm_parse_buttons(report, &state);
        break;
    case WM_REP_READ:
        if (length < WM_REP_READ_LEN)
            return;
        wm_read_cb(device, report + 2, length - WM_REP_READ_LEN);
        wm_parse_buttons(report, &state);
        break;
    case WM_REP_ACK:
        if (length < WM_REP_ACK_LEN)
            return;
        wm_handle_ack(device, report);
        wm_parse_buttons(report, &state);
        break;
    case WM_REP_BTN_ACC_IR_EXP:
        if (length < WM_REP_BTN_ACC_IR_EXP_LEN)
            return;
        wm_parse_ir_basic(report + WM_REP_BTN_ACC_LEN, ir_points);
        has_ir = true;
        wm_parse_exp(device, report + WM_REP_BTN_ACC_LEN + WM_REP_IR_BASIC_LEN, &state);
        // fallthrough
    case WM_REP_BTN_ACC:
        if (length < WM_REP_BTN_ACC_LEN)
            return;
        wm_parse_accel(device, report, &priv->cal, &state);
        // fallthrough
    case WM_REP_BTN:
        if (length < WM_REP_BTN_LEN)
            return;
        wm_parse_buttons(report, &state);
        break;
    }

    if (has_ir) {
        wm_ir_resolve(device, ir_points, &state);
    }

    // egc_device_driver_parse_report(data, priv->report_elements, &state);
    egc_device_driver_report_input(device, &state);
}

static bool wm_driver_ops_probe(u16 vid, u16 pid)
{
    static const egc_device_id_t compatible[] = {
        { WM_VID_NINTENDO, WM_PID_WIIMOTE },
    };

    return egc_device_driver_is_compatible(vid, pid, compatible, ARRAY_SIZE(compatible));
}

static int wm_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    struct wm_private_data_t *priv = PRIV(device);

    priv->ir_requested = true;
    priv->cal.zero[0] = priv->cal.zero[1] = priv->cal.zero[2] = 0x200;
    priv->cal.g_force[0] = priv->cal.g_force[1] = priv->cal.g_force[2] = 108;

    priv->requested_leds = 1; /* otherwise they will blink forever */
    egc_device_description_t *desc = egc_device_driver_alloc_desc(device);
    memcpy(desc, &s_device_description_wiimote, sizeof(s_device_description_wiimote));
    device->desc = desc;

    egc_device_driver_set_endpoints(device, EGC_USB_ENDPOINT_IN | 1, 5, EGC_USB_ENDPOINT_OUT, 5);

    wm_request_status(device);
    /* Set a half-second timer to let the device stabilize before requesting
     * updates */
    egc_device_driver_set_timer(device, 1000 * 500, 0);
    return 0;
}

static int wm_driver_ops_set_leds(egc_input_device_t *device, u32 leds)
{
    struct wm_private_data_t *priv = PRIV(device);

    EGC_DEBUG("leds: %" PRIu32, leds);
    priv->requested_leds = leds;
    return wm_request_status(device);
}

static int wm_driver_ops_set_rumble(egc_input_device_t *device, u16 low_frequency,
                                    u16 high_frequency)
{
    struct wm_private_data_t *priv = PRIV(device);
    EGC_DEBUG("low %d, high %d", low_frequency, high_frequency);
    priv->requested_rumble = low_frequency > 0 || high_frequency > 0;
    if (priv->state == WM_STATE_IDLE) {
        wm_step(device);
    }
    return 0;
}

const egc_device_driver_t wm_device_driver = {
    .probe = wm_driver_ops_probe,
    .init = wm_driver_ops_init,
    .set_leds = wm_driver_ops_set_leds,
    .set_rumble = wm_driver_ops_set_rumble,
    .intr_event = wm_driver_ops_intr_event,
};
bool wm_statusbar_on_top = false;

EgcWiimoteExpType egc_driver_wiimote_get_exp_type(egc_input_device_t *device)
{
    struct wm_private_data_t *priv = PRIV(device);
    return priv->exp_type;
}

void egc_driver_wiimote_set_bar_position(EgcDriverWiimoteBarPos position)
{
    s_bar_offset_y =
        position == EGC_DRIVER_WIIMOTE_BAR_POS_BOTTOM ? WM_BAR_OFFSET_Y : -WM_BAR_OFFSET_Y;
}

void egc_driver_wiimote_set_calibration_enabled(bool enabled)
{
    s_calibration_enabled = enabled;
}

bool egc_driver_wiimote_read_data(egc_input_device_t *device, u32 address, u16 size,
                                  EgcDriverWiimoteReadDataCb callback)
{
    s_client_cb.read_data = callback;
    return wm_read_data(device, address, size) >= 0;
}

bool egc_driver_wiimote_write_data(egc_input_device_t *device, u32 address, void *data, u8 size,
                                   EgcDriverWiimoteWriteDataCb callback)
{
    s_client_cb.write_data = callback;
    return wm_write_data(device, address, data, size) >= 0;
}

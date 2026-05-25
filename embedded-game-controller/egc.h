#ifndef EGC_H
#define EGC_H

#include <assert.h>

#include "egc_types.h"

/* Deprecated constants: the number of accelerometer, touch points, etc. is now
 * determined by the available place in the egc_input_state_t structure. */
#define EGC_MAX_ACCELEROMETERS 2
#define EGC_MAX_TOUCH_POINTS   2

typedef struct egc_input_device_t egc_input_device_t;
typedef struct egc_device_driver_t egc_device_driver_t;

/* Let's reserve the higher values for special functions */
#define EGC_RUMBLE_MAX 0x7fffffff
#define EGC_RUMBLE_OFF 0

typedef enum ATTRIBUTE_PACKED {
    EGC_DEVICE_TYPE_GAMEPAD,
    EGC_DEVICE_TYPE_GUITAR,
    EGC_DEVICE_TYPE_DRUMS,
} egc_device_type_e;
static_assert(sizeof(egc_device_type_e) == 1);

/* Any similarity with SDL3's SDL_GamepadButton is completely *not* accidental. */
typedef enum ATTRIBUTE_PACKED {
    EGC_GAMEPAD_BUTTON_SOUTH, /**< Bottom face button (e.g. Xbox A button) */
    EGC_GAMEPAD_BUTTON_EAST,  /**< Right face button (e.g. Xbox B button) */
    EGC_GAMEPAD_BUTTON_WEST,  /**< Left face button (e.g. Xbox X button) */
    EGC_GAMEPAD_BUTTON_NORTH, /**< Top face button (e.g. Xbox Y button) */
    EGC_GAMEPAD_BUTTON_BACK,
    EGC_GAMEPAD_BUTTON_GUIDE,
    EGC_GAMEPAD_BUTTON_START,
    EGC_GAMEPAD_BUTTON_LEFT_STICK,
    EGC_GAMEPAD_BUTTON_RIGHT_STICK,
    EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
    EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
    EGC_GAMEPAD_BUTTON_DPAD_UP,
    EGC_GAMEPAD_BUTTON_DPAD_DOWN,
    EGC_GAMEPAD_BUTTON_DPAD_LEFT,
    EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
    EGC_GAMEPAD_BUTTON_MISC1,         /**< Additional button (e.g. Xbox Series X share button, PS5
                                         microphone button, Nintendo Switch Pro capture button, Amazon Luna
                                         microphone button, Google Stadia capture button) */
    EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1, /**< Upper or primary paddle, under your right hand (e.g. Xbox
                                         Elite paddle P1) */
    EGC_GAMEPAD_BUTTON_LEFT_PADDLE1,  /**< Upper or primary paddle, under your left hand (e.g. Xbox
                                         Elite paddle P3) */
    EGC_GAMEPAD_BUTTON_RIGHT_PADDLE2, /**< Lower or secondary paddle, under your right hand (e.g.
                                         Xbox Elite paddle P2) */
    EGC_GAMEPAD_BUTTON_LEFT_PADDLE2, /**< Lower or secondary paddle, under your left hand (e.g. Xbox
                                        Elite paddle P4) */
    EGC_GAMEPAD_BUTTON_TOUCHPAD,     /**< PS4/PS5 touchpad button */
    EGC_GAMEPAD_BUTTON_MISC2,        /**< Additional button */
    EGC_GAMEPAD_BUTTON_MISC3,        /**< Additional button */
    EGC_GAMEPAD_BUTTON_MISC4,        /**< Additional button */
    EGC_GAMEPAD_BUTTON_MISC5,        /**< Additional button */
    EGC_GAMEPAD_BUTTON_MISC6,        /**< Additional button */
    EGC_GAMEPAD_BUTTON_COUNT,
    EGC_GAMEPAD_BUTTON_INVALID = 0xff
} egc_gamepad_button_e;
/* Each enum value corresponds to a bit in a 32 bit-wide field */
static_assert(EGC_GAMEPAD_BUTTON_COUNT <= 32);
static_assert(sizeof(egc_gamepad_button_e) == 1);

typedef enum ATTRIBUTE_PACKED {
    EGC_GAMEPAD_AXIS_LEFTX,
    EGC_GAMEPAD_AXIS_LEFTY, /* Y down is positive */
    EGC_GAMEPAD_AXIS_RIGHTX,
    EGC_GAMEPAD_AXIS_RIGHTY,       /* Y down is positive */
    EGC_GAMEPAD_AXIS_LEFT_TRIGGER, /* 0 (not pressed) - INT_MAX */
    EGC_GAMEPAD_AXIS_RIGHT_TRIGGER,
    EGC_GAMEPAD_AXIS_COUNT
} egc_gamepad_axis_e;
static_assert(sizeof(egc_gamepad_axis_e) == 1);

/* Resolution is 4096, therefore the range is +/- 8g */
#define EGC_ACCELEROMETER_RES_PER_G 4096

/*
 * Axes are defined like in SDL (https://wiki.libsdl.org/SDL3/SDL_SensorType):
 * - x: measures left (-) / right (+)
 * - y: gravity: when y is EGC_ACCELEROMETER_RES_PER_G, it means that the
 *   device is at rest
 * - z: measures farther (-) / closer (+)
 */
typedef struct {
    s16 x;
    s16 y;
    s16 z;
} ATTRIBUTE_PACKED egc_accelerometer_t;
static_assert(sizeof(egc_accelerometer_t) == 6);

/* For the gyroscope we reuse the same structure as the accelerometer, and the
 * meaning of the x, y, z members is the angular speed around the omonymous
 * axis, expressed in 1 / EGC_GYROSCOPE_RES of radians per second. That is, a
 * the value of EGC_GYROSCOPE_RES means a rotation of 1 radian per second.
 *
 * The orientation is the same defined in libSDL: the rotation is positive when
 * the device is rotating counter-clockwise from the poing of view of an
 * observer staying on a point on the positive direction of the axis. That is
 * from the point of view of user holding the controller:
 * - x: observer placed on the right of the device
 * - y: observer placed above the device
 * - z: observer is the user
 */
typedef egc_accelerometer_t egc_gyroscope_t;

/* Range is 0-32767. A negative x means that there is no touch */
#define EGC_GAMEPAD_TOUCH_RES 0x7fff

typedef struct {
    s16 x;
    s16 y;
} ATTRIBUTE_PACKED egc_point_t;
static_assert(sizeof(egc_point_t) == 4);

typedef struct egc_input_state_t {
    /* TODO: maybe add a timestamp or a counter, to let the client know if the
     * data was updated? */
    union {
        u8 bytes[sizeof(u32) /* buttons */
                 + sizeof(s16) * EGC_GAMEPAD_AXIS_COUNT
                 /* This composition is just an example to give an idea of what data
                  * can fit; the actual composition is given by the `num_*` members
                  * of the egc_device_description_t structure */
                 + sizeof(egc_accelerometer_t) * 2 + sizeof(egc_gyroscope_t) +
                 sizeof(egc_point_t) * 2];

        struct {
            u32 buttons;
            s16 axes[EGC_GAMEPAD_AXIS_COUNT] ATTRIBUTE_ALIGN(2);
            egc_accelerometer_t accelerometer[EGC_MAX_ACCELEROMETERS];
            egc_point_t touch_points[EGC_MAX_TOUCH_POINTS];
        } ATTRIBUTE_PACKED gamepad ATTRIBUTE_DEPRECATED;
        /* TODO: add struct for guitar and drums */
    };
} egc_input_state_t;

typedef enum ATTRIBUTE_PACKED {
    EGC_CONNECTION_DISCONNECTED,
    EGC_CONNECTION_USB,
    EGC_CONNECTION_BT,
} egc_connection_e;
static_assert(sizeof(egc_connection_e) == 1);

typedef struct egc_device_description_t {
    u16 vendor_id;
    u16 product_id;
    u32 available_buttons; /* bitmask indexed by egc_gamepad_button_e */
    u32 available_axes;    /* bitmask indexed by egc_gamepad_axis_e */
    egc_device_type_e type;
    u8 num_touch_points;
    u8 num_accelerometers;
    u8 num_gyroscopes;
    u8 num_leds;
    bool has_rumble;
} ATTRIBUTE_PACKED egc_device_description_t;

struct egc_input_device_t {
    const egc_device_description_t *desc;
    egc_input_state_t state ATTRIBUTE_ALIGN(4);
    egc_connection_e connection;
    bool suspended;
} ATTRIBUTE_PACKED ATTRIBUTE_ALIGN(8);

typedef void (*egc_input_device_cb)(egc_input_device_t *device, void *userdata);

int egc_initialize(egc_input_device_cb added_cb, egc_input_device_cb removed_cb, void *userdata);
int egc_input_device_suspend(egc_input_device_t *device);
int egc_input_device_resume(egc_input_device_t *device);
/* led_state is a bit mask of the active leds */
int egc_input_device_set_leds(egc_input_device_t *device, u32 led_state);
int egc_input_device_set_rumble(egc_input_device_t *device, u16 low_frequency, u16 high_frequency);

/* These macros are for internal use */
#define _EGC_STATE_OFFSET_AXES  sizeof(u32)
#define _EGC_STATE_OFFSET_ACCEL (_EGC_STATE_OFFSET_AXES + sizeof(s16) * EGC_GAMEPAD_AXIS_COUNT)

static inline u32 egc_input_device_read_buttons(egc_input_device_t *device)
{
    return *(u32 *)device->state.bytes;
}

static inline s16 egc_input_device_read_axis(egc_input_device_t *device, egc_gamepad_axis_e axis)
{
    s16 *axes = (s16 *)(device->state.bytes + _EGC_STATE_OFFSET_AXES);
    return axes[axis];
}

static inline const egc_accelerometer_t *
egc_input_device_read_accelerometer(egc_input_device_t *device, int index)
{
    egc_accelerometer_t *accel =
        (egc_accelerometer_t *)(device->state.bytes + _EGC_STATE_OFFSET_ACCEL);
    return &accel[index];
}

static inline const egc_gyroscope_t *egc_input_device_read_gyroscope(egc_input_device_t *device,
                                                                     int index)
{
    egc_gyroscope_t *gyro =
        (egc_gyroscope_t *)(device->state.bytes + _EGC_STATE_OFFSET_ACCEL +
                            sizeof(egc_accelerometer_t) * device->desc->num_accelerometers);
    return &gyro[index];
}

static inline egc_point_t egc_input_device_read_touch_point(egc_input_device_t *device, int index)
{
    egc_point_t *points =
        (egc_point_t *)(device->state.bytes + _EGC_STATE_OFFSET_ACCEL +
                        sizeof(egc_accelerometer_t) * device->desc->num_accelerometers +
                        sizeof(egc_gyroscope_t) * device->desc->num_gyroscopes);
    return points[index];
}

/* Note: suspending might not be supported by all backends */
int egc_input_device_set_suspended(egc_input_device_t *device, bool suspended);

/* Fetch events and invoke callbacks. */
int egc_handle_events(void);
/* Fetch events and invoke callbacks. Wait up to \a timeout_us microseconds for new events. */
int egc_wait_events(u32 timeout_us);

int egc_bt_start_scan();
int egc_bt_stop_scan();

int egc_bt_enter_page_mode();
int egc_bt_leave_page_mode();

#endif

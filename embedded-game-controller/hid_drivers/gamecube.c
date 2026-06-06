#include <assert.h>
#include <ogc/pad.h>

#include "driver_api.h"
#include "gamecube.h"
#include "utils.h"

#define GC_VID_NINTENDO            0x057e
#define GC_PID_GAMECUBE_CONTROLLER 0x0337

typedef struct {
    /* This must be the first member, since we use it for casting */
    egc_device_priv_t priv;
} gc_device_t;

#define PUB(d) (&(d)->priv.pub)

static gc_device_t s_devices[PAD_CHANMAX];
static PADStatus s_pad[PAD_CHANMAX];

enum gc_buttons_e {
    GC_BUTTON_LEFT,
    GC_BUTTON_RIGHT,
    GC_BUTTON_DOWN,
    GC_BUTTON_UP,
    GC_BUTTON_Z,
    GC_BUTTON_R,
    GC_BUTTON_L,
    GC_BUTTON_UNUSED,
    GC_BUTTON_A,
    GC_BUTTON_B,
    GC_BUTTON_X,
    GC_BUTTON_Y,
    GC_BUTTON_START,
    GC_BUTTON_COUNT
};

static const egc_gamepad_button_e s_button_map[GC_BUTTON_COUNT] = {
    [GC_BUTTON_LEFT] = EGC_GAMEPAD_BUTTON_DPAD_LEFT,
    [GC_BUTTON_RIGHT] = EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
    [GC_BUTTON_DOWN] = EGC_GAMEPAD_BUTTON_DPAD_DOWN,
    [GC_BUTTON_UP] = EGC_GAMEPAD_BUTTON_DPAD_UP,
    [GC_BUTTON_Z] = EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
    [GC_BUTTON_R] = EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1,
    [GC_BUTTON_L] = EGC_GAMEPAD_BUTTON_LEFT_PADDLE1,
    [GC_BUTTON_UNUSED] = EGC_GAMEPAD_BUTTON_INVALID,
    [GC_BUTTON_A] = EGC_GAMEPAD_BUTTON_SOUTH,
    [GC_BUTTON_B] = EGC_GAMEPAD_BUTTON_WEST,
    [GC_BUTTON_X] = EGC_GAMEPAD_BUTTON_EAST,
    [GC_BUTTON_Y] = EGC_GAMEPAD_BUTTON_NORTH,
    [GC_BUTTON_START] = EGC_GAMEPAD_BUTTON_START,
};

static const egc_device_description_t s_device_description = {
    .vendor_id = GC_VID_NINTENDO,
    .product_id = GC_PID_GAMECUBE_CONTROLLER,
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
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER),
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
    .has_rumble = true,
};

static inline gc_device_t *gc_device_from_input_device(egc_input_device_t *input_device)
{
    return (gc_device_t *)input_device;
}

static inline u8 get_channel(egc_input_device_t *input_device)
{
    return gc_device_from_input_device(input_device) - s_devices;
}

static inline PADStatus *get_status(egc_input_device_t *input_device)
{
    return &s_pad[get_channel(input_device)];
}

static inline s16 stick_value(s8 v)
{
    /* Maximum extents range from 96 to 105 on my controller, so let's assume
     * 96 as maximum */
    if (v > 96)
        return INT16_MAX;
    if (v < -96)
        return INT16_MIN;
    return v * INT16_MAX / 96;
}

static inline s16 trigger_value(u8 v)
{
    /* Maximum extent is about 200 */
    return v > 200 ? INT16_MAX : (v * INT16_MAX / 200);
}

int _egc_gc_process_events(egc_event_cb event_handler)
{
    int count = 0;

    /* The PAD API does not have a way to notify when an event triggered; so
     * here we just read the current status and return immediately */
    PAD_Read(s_pad);
    u32 resetBits = 0;
    for (int i = 0; i < PAD_CHANMAX; i++) {
        gc_device_t *device = &s_devices[i];
        egc_input_device_t *input_device = PUB(device);
        bool was_connected = input_device->connection != EGC_CONNECTION_DISCONNECTED;
        s8 err = s_pad[i].err;
        bool connected = err == PAD_ERR_NONE || (was_connected && err != PAD_ERR_NO_CONTROLLER);
        if (connected != was_connected) {
            if (connected) {
                input_device->connection = EGC_CONNECTION_SERIAL;
                input_device->desc = &s_device_description;
                input_device->suspended = false;
                /* Hardcode our driver on the device */
                get_priv(input_device)->driver = &gc_device_driver;
                event_handler(input_device, EGC_EVENT_DEVICE_ADDED, GC_VID_NINTENDO,
                              GC_PID_GAMECUBE_CONTROLLER);
            } else {
                event_handler(input_device, EGC_EVENT_DEVICE_REMOVED);
                /* Mark this device as not valid */
                input_device->connection = EGC_CONNECTION_DISCONNECTED;
            }
            count++;
        }

        if (err == PAD_ERR_NONE) {
            /* We don't use egc_device_driver_report_input(), since we know we
             * are running in the main EGC thread and can modify the fields
             * directly */
            egc_input_state_t *state = &input_device->state;
            memset(state, 0, sizeof(*state));

            u32 buttons =
                egc_device_driver_map_buttons(s_pad[i].button, GC_BUTTON_COUNT, s_button_map);
            egc_device_driver_set_buttons(state, buttons);
            egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFTX, stick_value(s_pad[i].stickX));
            egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFTY,
                                       -stick_value(s_pad[i].stickY));
            egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_RIGHTX,
                                       stick_value(s_pad[i].substickX));
            egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_RIGHTY,
                                       stick_value(-s_pad[i].substickY));
            egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_LEFT_TRIGGER,
                                       trigger_value(s_pad[i].triggerL));
            egc_device_driver_set_axis(state, EGC_GAMEPAD_AXIS_RIGHT_TRIGGER,
                                       trigger_value(s_pad[i].triggerR));
            count++;
        } else if (err == PAD_ERR_NO_CONTROLLER) {
            resetBits |= (PAD_CHAN0_BIT >> i);
        }
    }

    if (resetBits != 0) {
        PAD_Reset(resetBits);
    }

    return count;
}

int _egc_gc_init(egc_event_cb event_handler)
{
    PAD_Init();

    for (int i = 0; i < ARRAY_SIZE(s_devices); i++)
        PUB(&s_devices[i])->connection = EGC_CONNECTION_DISCONNECTED;

    return _egc_gc_process_events(event_handler) >= 0 ? 0 : -1;
}

static int gc_driver_ops_set_rumble(egc_input_device_t *device, u16 low_frequency,
                                    u16 high_frequency)
{
    bool must_rumble = low_frequency > 0 || high_frequency > 0;
    PAD_ControlMotor(get_channel(device), must_rumble ? PAD_MOTOR_RUMBLE : PAD_MOTOR_STOP);
    return 0;
}

const egc_device_driver_t gc_device_driver = {
    .set_rumble = gc_driver_ops_set_rumble,
};

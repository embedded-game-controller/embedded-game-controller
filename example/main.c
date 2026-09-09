#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "terminal.h"

#include "embedded-game-controller/egc.h"

#define MAX_DEVICES 4
#define BIT(x)      (1 << x)

static egc_input_device_t *s_devices[MAX_DEVICES];

static const char *s_button_names_gamepad[] = {
    /* clang-format off */
    "SOUTH",
    "EAST",
    "WEST",
    "NORTH",
    "BACK",
    "GUIDE",
    "START",
    "LEFT_STICK",
    "RIGHT_STICK",
    "LEFT_SHOULDER",
    "RIGHT_SHOULDER",
    "↑",
    "↓",
    "←",
    "→",
    "MISC1",
    "RIGHT_PADDLE1",
    "LEFT_PADDLE1",
    "RIGHT_PADDLE2",
    "LEFT_PADDLE2",
    "TOUCHPAD",
    "MISC2",
    "MISC3",
    "MISC4",
    "MISC5",
    "MISC6",
    /* clang-format on */
};
static_assert(sizeof(s_button_names_gamepad) == sizeof(char *) * EGC_GAMEPAD_BUTTON_COUNT);

static const char *s_button_names_guitar[] = {
    /* clang-format off */
    "FRET0",
    "FRET1",
    "FRET2",
    "FRET3",
    "FRET4",
    "FRET5",
    "FRET6",
    "FRET7",
    "FRET8",
    "FRET9",
    "FRET10",
    "FRET11",
    "STRUM_UP",
    "STRUM_DOWN",
    "START",
    "BACK",
    "GUIDE",
    "DPAD_UP",
    "DPAD_DOWN",
    "DPAD_LEFT",
    "DPAD_RIGHT",
    /* clang-format on */
};
static_assert(sizeof(s_button_names_guitar) == sizeof(char *) * EGC_GUITAR_BUTTON_COUNT);

static void print_buttons(egc_input_device_t *device, const char **names)
{
    u32 buttons = egc_input_device_read_buttons(device);

    for (int i = 0; i < EGC_GAMEPAD_BUTTON_COUNT; i++) {
        u32 mask = 1 << i;
        if ((device->desc->available_buttons & mask) && (buttons & mask)) {
            printf("%s ", names[i]);
        }
    }
    printf("\n  ");
}

static void print_status_gamepad(egc_input_device_t *device)
{
    print_buttons(device, s_button_names_gamepad);

#define HAS_AXIS(x) (device->desc->available_axes & (1 << x))
    if (HAS_AXIS(EGC_GAMEPAD_AXIS_LEFTX)) {
        printf("L stick: %d,%d ", egc_input_device_read_axis(device, EGC_GAMEPAD_AXIS_LEFTX),
               egc_input_device_read_axis(device, EGC_GAMEPAD_AXIS_LEFTY));
    }

    if (HAS_AXIS(EGC_GAMEPAD_AXIS_RIGHTX)) {
        printf("R stick: %d,%d ", egc_input_device_read_axis(device, EGC_GAMEPAD_AXIS_RIGHTX),
               egc_input_device_read_axis(device, EGC_GAMEPAD_AXIS_RIGHTY));
    }

    if (HAS_AXIS(EGC_GAMEPAD_AXIS_LEFT_TRIGGER)) {
        printf("L trigger: %d ", egc_input_device_read_axis(device, EGC_GAMEPAD_AXIS_LEFT_TRIGGER));
    }

    if (HAS_AXIS(EGC_GAMEPAD_AXIS_RIGHT_TRIGGER)) {
        printf("R trigger: %d ",
               egc_input_device_read_axis(device, EGC_GAMEPAD_AXIS_RIGHT_TRIGGER));
    }

    for (int i = 0; i < device->desc->num_accelerometers; i++) {
        const egc_accelerometer_t *accel = egc_input_device_read_accelerometer(device, i);
        printf("Accel%d (%d %d %d) ", i, accel->x, accel->y, accel->z);
    }

    for (int i = 0; i < device->desc->num_gyroscopes; i++) {
        const egc_gyroscope_t *gyro = egc_input_device_read_gyroscope(device, i);
        printf("Gyro%d (%d %d %d) ", i, gyro->x, gyro->y, gyro->z);
    }

    for (int i = 0; i < device->desc->num_touch_points; i++) {
        egc_point_t p = egc_input_device_read_touch_point(device, i);
        if (p.x >= 0) {
            printf("Touch%d (%4d %4d) ", i, p.x >> 5, p.y >> 5);
        }
    }

    if (device->desc->available_axes || device->desc->num_accelerometers > 0) {
        printf("\n");
    }
}

static void print_status_guitar(egc_input_device_t *device)
{
    print_buttons(device, s_button_names_guitar);

    s16 *axes;
    egc_accelerometer_t *accelerometers;
    egc_input_device_read_data(device, EGC_READ_AXES | EGC_READ_ACCELEROMETERS, &axes,
                               &accelerometers);
    if (device->desc->available_axes & BIT(EGC_GUITAR_AXIS_WHAMMY_BAR)) {
        printf("Whammy %5d ", axes[EGC_GUITAR_AXIS_WHAMMY_BAR]);
    }
    if (device->desc->available_axes & BIT(EGC_GUITAR_AXIS_EFFECT)) {
        printf("Effect %5d ", axes[EGC_GUITAR_AXIS_EFFECT]);
    }
    if (device->desc->available_axes & BIT(EGC_GUITAR_AXIS_STICKX)) {
        printf("Stick (%5d,%5d) ", axes[EGC_GUITAR_AXIS_STICKX], axes[EGC_GUITAR_AXIS_STICKY]);
    } else
        printf("Axes %08x ", device->desc->available_axes);

    for (int i = 0; i < device->desc->num_accelerometers; i++) {
        const egc_accelerometer_t *accel = &accelerometers[i];
        printf("Accel%d (%d %d %d) ", i, accel->x, accel->y, accel->z);
    }

    if (device->desc->available_axes || device->desc->num_accelerometers > 0) {
        printf("\n");
    }
}

static void print_status_balance_board(egc_input_device_t *device)
{
    print_buttons(device, s_button_names_gamepad);

    float corners[4];
    for (int i = 0; i < EGC_BOARD_AXIS_COUNT; i++) {
        u16 value = (u16)egc_input_device_read_axis(device, i);
        corners[i] = value / (float)EGC_BOARD_RES_PER_100KG;
    }
    printf("TL: %.3f TR: %.3f BR: %.3f BL: %.3f, Total %.3f\n", corners[0], corners[1], corners[2],
           corners[3], corners[0] + corners[1] + corners[2] + corners[3]);
}

static void print_status(egc_input_device_t *device)
{
    if (device->desc->type == EGC_DEVICE_TYPE_GAMEPAD) {
        print_status_gamepad(device);
    } else if (device->desc->type == EGC_DEVICE_TYPE_GUITAR) {
        print_status_guitar(device);
    } else if (device->desc->type == EGC_DEVICE_TYPE_BALANCE_BOARD) {
        print_status_balance_board(device);
    }
}

static void on_device_added(egc_input_device_t *device, void *userdata)
{
    bool added = false;
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!s_devices[i]) {
            s_devices[i] = device;
            printf("Added %04x:%04x on slot %d\n", device->desc->vendor_id,
                   device->desc->product_id, i);
            added = true;
            break;
        }
    }
    if (!added) {
        fprintf(stderr, "No free device slots for %04x:%04x\n", device->desc->vendor_id,
                device->desc->product_id);
    }
}

static void on_device_removed(egc_input_device_t *device, void *userdata)
{
    if (!device->desc)
        return;

    bool removed = false;
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (s_devices[i] == device) {
            s_devices[i] = NULL;
            printf("Removed %04x:%04x from slot %d\n", device->desc->vendor_id,
                   device->desc->product_id, i);
            removed = true;
        }
    }
    if (!removed) {
        fprintf(stderr, "Device %04x:%04x was not watched\n", device->desc->vendor_id,
                device->desc->product_id);
    }
}

int main(int argc, char **argv)
{
    quit_requested = false;

    /* Some platforms need to perform some more steps before having the console
     * output setup. */
    terminal_init();

    printf("Initializing...\n");
    int rc = egc_initialize(on_device_added, on_device_removed, NULL);
    printf("egc_initialize returned %d\n", rc);
    int led = 0;
    u32 rumble_intensity = 0;

    u32 previously_down[MAX_DEVICES] = {
        0,
    };

    egc_bt_enter_page_mode();
    egc_bt_start_scan();

    while (!quit_requested) {
        egc_wait_events(1000000);

        for (int i = 0; i < MAX_DEVICES; i++) {
            egc_input_device_t *device = s_devices[i];
            if (!device)
                continue;

            u32 down = egc_input_device_read_buttons(device);
            u32 released = previously_down[i] & ~down;
            previously_down[i] = down;

            if (down)
                print_status(device);

            if (down & (1 << EGC_GAMEPAD_BUTTON_SOUTH)) {
                u32 led_btn =
                    (1 << EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER) | (1 << EGC_GAMEPAD_BUTTON_DPAD_RIGHT);
                u32 rumble_btn =
                    (1 << EGC_GAMEPAD_BUTTON_LEFT_SHOULDER) | (1 << EGC_GAMEPAD_BUTTON_DPAD_LEFT);
                if (device->desc->num_leds > 0 && released & led_btn) {
                    led = (led + 1) % device->desc->num_leds;
                    egc_input_device_set_leds(device, 1 << led);
                }

                if (device->desc->has_rumble) {
                    u16 new_intensity = down & rumble_btn ? 0xffff : 0;
                    if (new_intensity != rumble_intensity) {
                        egc_input_device_set_rumble(device, new_intensity, new_intensity);
                        rumble_intensity = new_intensity;
                    }
                }
            }
            if (down & (1 << EGC_GAMEPAD_BUTTON_EAST)) {
                if (released & (1 << EGC_GAMEPAD_BUTTON_DPAD_LEFT)) {
                    static bool enabled = true;
                    enabled = !enabled;
                    printf("%s accelerometer\n", enabled ? "Enabling" : "Disabling");
                    egc_input_device_enable_accelerometer(device, 0, enabled);
                }
                if (released & (1 << EGC_GAMEPAD_BUTTON_DPAD_DOWN)) {
                    static bool enabled = true;
                    enabled = !enabled;
                    printf("%s IR\n", enabled ? "Enabling" : "Disabling");
                    egc_input_device_enable_touch_point(device, 0, enabled);
                }
                if (released & (1 << EGC_GAMEPAD_BUTTON_DPAD_UP)) {
                    static bool enabled = true;
                    enabled = !enabled;
                    printf("%s gyroscope\n", enabled ? "Enabling" : "Disabling");
                    egc_input_device_enable_gyroscope(device, 0, enabled);
                }
            }
        }
    }

    return EXIT_SUCCESS;
}

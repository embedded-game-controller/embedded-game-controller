#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "egc.h"
#include "terminal.h"

#define MAX_DEVICES 4

static egc_input_device_t *s_devices[MAX_DEVICES];

static void print_status(egc_input_device_t *device)
{
    u32 buttons = egc_input_device_read_buttons(device);

#define PRESSED(b) (buttons & (1 << b))
    if (PRESSED(EGC_GAMEPAD_BUTTON_SOUTH)) {
        printf("SOUTH ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_EAST)) {
        printf("EAST ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_WEST)) {
        printf("WEST ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_NORTH)) {
        printf("NORTH ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_BACK)) {
        printf("BACK ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_GUIDE)) {
        printf("GUIDE ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_START)) {
        printf("START ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_STICK)) {
        printf("LEFT-STICK ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_RIGHT_STICK)) {
        printf("RIGHT-STICK ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER)) {
        printf("LEFT-SHOULDER ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER)) {
        printf("RIGHT-SHOULDER ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_DOWN)) {
        printf("↓ ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_LEFT)) {
        printf("← ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_RIGHT)) {
        printf("→ ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_UP)) {
        printf("↑ ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_MISC1)) {
        printf("MISC1 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_PADDLE1)) {
        printf("LEFT-PADDLE1 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1)) {
        printf("RIGHT-PADDLE1 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_PADDLE2)) {
        printf("LEFT-PADDLE2 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_RIGHT_PADDLE2)) {
        printf("RIGHT-PADDLE2 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_TOUCHPAD)) {
        printf("TOUCHPAD ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_MISC2)) {
        printf("MISC2 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_MISC3)) {
        printf("MISC3 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_MISC4)) {
        printf("MISC4 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_MISC5)) {
        printf("MISC5 ");
    }
    if (PRESSED(EGC_GAMEPAD_BUTTON_MISC6)) {
        printf("MISC6 ");
    }
    printf("\n  ");
#undef PRESSED

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

    u32 previously_down = 0;

    egc_bt_enter_page_mode();
    egc_bt_start_scan();

    while (!quit_requested) {
        egc_wait_events(1000000);

        for (int i = 0; i < MAX_DEVICES; i++) {
            egc_input_device_t *device = s_devices[i];
            if (!device)
                continue;

            u32 down = egc_input_device_read_buttons(device);
            u32 released = previously_down & ~down;
            previously_down = down;

            if (down)
                print_status(device);

            if (down & (1 << EGC_GAMEPAD_BUTTON_SOUTH)) {
                if (device->desc->num_leds > 0 &&
                    released & (1 << EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER)) {
                    led = (led + 1) % device->desc->num_leds;
                    egc_input_device_set_leds(device, 1 << led);
                }

                if (device->desc->has_rumble) {
                    u16 new_intensity = down & (1 << EGC_GAMEPAD_BUTTON_LEFT_SHOULDER) ? 0xffff : 0;
                    if (new_intensity != rumble_intensity) {
                        egc_input_device_set_rumble(device, new_intensity, new_intensity);
                        rumble_intensity = new_intensity;
                    }
                }
            }
        }
    }

    return EXIT_SUCCESS;
}

#include "generic_mappings.h"

typedef struct {
    u16 vid;
    u16 pid;
    const u8 *elements;
} Mapping;

static const u8 s_elements_dragonrise[] = {
    /* clang-format off */
    EGC_INPUT_REPORT_TYPE_AXIS_U8, EGC_GAMEPAD_AXIS_LEFTX,
    EGC_INPUT_REPORT_TYPE_AXIS_U8, EGC_GAMEPAD_AXIS_LEFTY,
    EGC_INPUT_REPORT_TYPE_SKIP, 8,
    EGC_INPUT_REPORT_TYPE_AXIS_U8, EGC_GAMEPAD_AXIS_RIGHTX,
    EGC_INPUT_REPORT_TYPE_AXIS_U8, EGC_GAMEPAD_AXIS_RIGHTY,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_WEST,
        EGC_GAMEPAD_BUTTON_SOUTH,
        EGC_GAMEPAD_BUTTON_EAST,
        EGC_GAMEPAD_BUTTON_NORTH,
    EGC_INPUT_REPORT_TYPE_DPAD,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_RIGHT_STICK,
        EGC_GAMEPAD_BUTTON_LEFT_STICK,
        EGC_GAMEPAD_BUTTON_START,
        EGC_GAMEPAD_BUTTON_BACK,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_RIGHT_TRIGGER,
        EGC_GAMEPAD_BUTTON_LEFT_TRIGGER,
        EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
        EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
    EGC_INPUT_REPORT_TYPE_SKIP, 8,
    EGC_INPUT_REPORT_TYPE_END
    /* clang-format on */
};

static const u8 s_elements_fake_snes[] = {
    /* clang-format off */
    EGC_INPUT_REPORT_TYPE_AXIS_U8, EGC_GAMEPAD_AXIS_DPADX,
    EGC_INPUT_REPORT_TYPE_AXIS_U8, EGC_GAMEPAD_AXIS_DPADY,
    EGC_INPUT_REPORT_TYPE_SKIP, 24,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_WEST,
        EGC_GAMEPAD_BUTTON_SOUTH,
        EGC_GAMEPAD_BUTTON_EAST,
        EGC_GAMEPAD_BUTTON_NORTH,
    EGC_INPUT_REPORT_TYPE_SKIP, 4,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_START,
        EGC_GAMEPAD_BUTTON_BACK,
    EGC_INPUT_REPORT_TYPE_BUTTON4,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_INVALID,
        EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
        EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
    EGC_INPUT_REPORT_TYPE_SKIP, 8,
    EGC_INPUT_REPORT_TYPE_END
    /* clang-format on */
};

static const Mapping s_mappings[] = {
    { 0x0079, 0x0006, s_elements_dragonrise }, /* DragonRise Inc. | PC TWIN SHOCK Gamepad */
    { 0x081f, 0xe401,  s_elements_fake_snes },
};

const u8 *egc_device_driver_input_parser_for(u16 vid, u16 pid)
{
    return egc_device_driver_hardcoded_mapping(vid, pid);
}

const u8 *egc_device_driver_hardcoded_mapping(u16 vid, u16 pid)
{
    for (int i = 0; i < ARRAY_SIZE(s_mappings); i++) {
        const Mapping *m = &s_mappings[i];
        if (m->vid == vid && m->pid == pid)
            return m->elements;
    }
    return NULL;
}

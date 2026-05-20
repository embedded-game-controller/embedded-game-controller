#ifndef EGC_DRIVERS_WIIMOTE_H
#define EGC_DRIVERS_WIIMOTE_H

typedef enum ATTRIBUTE_PACKED {
    EGC_WIIMOTE_EXP_NONE = 0,
    EGC_WIIMOTE_EXP_NUNCHUCK,
    EGC_WIIMOTE_EXP_CLASSIC,
    EGC_WIIMOTE_EXP_CLASSIC_PRO,
    EGC_WIIMOTE_EXP_DRAWSOME_TABLET,
    EGC_WIIMOTE_EXP_GUITAR_HERO_3,
    EGC_WIIMOTE_EXP_GUITAR_HERO_DRUMS,
    EGC_WIIMOTE_EXP_BALANCE_BOARD,
    EGC_WIIMOTE_EXP_DJ_HERO,
    EGC_WIIMOTE_EXP_TAIKO_DRUM,
    EGC_WIIMOTE_EXP_UDRAW_TABLET,
    EGC_WIIMOTE_EXP_SHINKANSEN,
} EgcWiimoteExpType;

EgcWiimoteExpType egc_driver_wiimote_get_exp_type(egc_input_device_t *device);

typedef enum {
    EGC_DRIVER_WIIMOTE_BAR_POS_BOTTOM = 0,
    EGC_DRIVER_WIIMOTE_BAR_POS_TOP,
} EgcDriverWiimoteBarPos;

void egc_driver_wiimote_set_bar_position(EgcDriverWiimoteBarPos position);

/* Enable calibration of the sticks (in the future, accelerometer too). This
 * setting takes effect only after reconnecting the expansion. */
void egc_driver_wiimote_set_calibration_enabled(bool enabled);

typedef void (*EgcDriverWiimoteReadDataCb)(egc_input_device_t *device, int error_code, u16 size,
                                           const void *buffer);
bool egc_driver_wiimote_read_data(egc_input_device_t *device, u32 address, u16 size,
                                  EgcDriverWiimoteReadDataCb callback);

typedef void (*EgcDriverWiimoteWriteDataCb)(egc_input_device_t *device, int error_code);
bool egc_driver_wiimote_write_data(egc_input_device_t *device, u32 address, void *data, u8 size,
                                   EgcDriverWiimoteWriteDataCb callback);

#endif /* EGC_DRIVERS_WIIMOTE_H */

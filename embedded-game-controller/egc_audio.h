#ifndef EGC_AUDIO_H
#define EGC_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "egc_types.h"

typedef struct egc_input_device_t egc_input_device_t;
typedef struct egc_device_driver_t egc_device_driver_t;

typedef enum egc_audio_format_e {
    EGC_AUDIO_FORMAT_INVALID = 0,
    EGC_AUDIO_FORMAT_S8,
    EGC_AUDIO_FORMAT_ADPCM_YAMAHA,
} egc_audio_format_e;

typedef struct egc_audio_spec_t {
    egc_audio_format_e format;
    u8 num_channels;
    u16 frequency;
} egc_audio_spec_t;

typedef struct egc_audio_options_t {
    egc_audio_spec_t spec;
} egc_audio_options_t;

typedef struct egc_audio_device_t egc_audio_device_t;

egc_audio_device_t *egc_input_device_open_audio(egc_input_device_t *device,
                                                const egc_audio_options_t *options);
const egc_audio_spec_t *egc_audio_device_get_spec(egc_audio_device_t *device);
int egc_audio_device_get_max_stream_size(egc_audio_device_t *device);
int egc_audio_device_stream(egc_audio_device_t *device, const void *data, u16 size);
int egc_audio_device_close(egc_audio_device_t *device);

#ifdef __cplusplus
}
#endif

#endif

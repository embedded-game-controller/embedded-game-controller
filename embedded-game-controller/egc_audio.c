#include <errno.h>
#include <stdarg.h>
#include <string.h>

#include "driver_api.h"
#include "internals.h"
#include "platform.h"
#include "utils.h"

egc_audio_device_t *_egc_audio_devices = NULL;

static void audio_device_remove(egc_audio_device_t *device)
{
    egc_audio_device_t **target = &_egc_audio_devices;
    while (*target != device) {
        target = &(*target)->next;
    }
    *target = device->next;
    egc_free(device);
}

egc_audio_device_t *_egc_input_device_get_audio(egc_input_device_t *device)
{
    egc_audio_device_t *node = _egc_audio_devices;
    while (node) {
        if (node->input_device == device)
            break;
        node = node->next;
    }
    return node;
}

egc_audio_device_t *egc_input_device_open_audio(egc_input_device_t *device,
                                                const egc_audio_options_t *options)
{
    egc_device_priv_t *priv = get_priv(device);
    if (!priv->driver->open_audio)
        return NULL;

    egc_audio_device_t *audio_device = egc_malloc(sizeof(egc_audio_device_t));
    if (!audio_device)
        return NULL;

    memset(audio_device, 0, sizeof(*audio_device));
    audio_device->input_device = device;
    int rc = priv->driver->open_audio(device, audio_device, options);
    if (rc < 0) {
        egc_free(audio_device);
        return NULL;
    }

    egc_audio_device_t **target = &_egc_audio_devices;
    while (*target) {
        target = &(*target)->next;
    }
    *target = audio_device;

    return audio_device;
}

const egc_audio_spec_t *egc_audio_device_get_spec(egc_audio_device_t *device)
{
    return &device->spec;
}

int egc_audio_device_get_max_stream_size(egc_audio_device_t *device)
{
    return device->max_stream_size;
}

int egc_audio_device_stream(egc_audio_device_t *device, const void *data, u16 size)
{
    return device->driver->stream(device, data, size);
}

int egc_audio_device_close(egc_audio_device_t *device)
{
    int rc = device->driver->close(device);
    if (rc >= 0) {
        audio_device_remove(device);
    }
    return rc;
}

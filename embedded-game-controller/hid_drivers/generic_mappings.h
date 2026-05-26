#ifndef EGC_GENERIC_MAPPINGS_H
#define EGC_GENERIC_MAPPINGS_H

#include "driver_api.h"

#ifdef WITH_GENERIC_MAPPINGS
const u8 *egc_device_driver_input_parser_for(u16 vid, u16 pid);
#else
const u8 *__attribute__((weak)) egc_device_driver_input_parser_for(u16 vid, u16 pid)
{
    return NULL;
}
#endif
const u8 *egc_device_driver_hardcoded_mapping(u16 vid, u16 pid);

#endif /* EGC_GENERIC_MAPPINGS_H */

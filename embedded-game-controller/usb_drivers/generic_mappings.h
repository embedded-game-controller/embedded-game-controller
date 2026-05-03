#ifndef EGC_GENERIC_MAPPINGS_H
#define EGC_GENERIC_MAPPINGS_H

#include "driver_api.h"

const u8 *egc_device_driver_input_parser_for(u16 vid, u16 pid);
const u8 *egc_device_driver_hardcoded_mapping(u16 vid, u16 pid);

#endif /* EGC_GENERIC_MAPPINGS_H */

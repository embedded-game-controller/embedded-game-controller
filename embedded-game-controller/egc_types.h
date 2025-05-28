#ifndef EGC_TYPES_H
#define EGC_TYPES_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

/* Data types */
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

/* Attributes */
#ifndef ATTRIBUTE_ALIGN
#define ATTRIBUTE_ALIGN(v) __attribute__((aligned(v)))
#endif
#ifndef ATTRIBUTE_PACKED
#define ATTRIBUTE_PACKED __attribute__((packed))
#endif

#endif

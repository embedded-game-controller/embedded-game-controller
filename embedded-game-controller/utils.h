#ifndef UTILS_H
#define UTILS_H

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "egc_types.h"

#ifndef le16toh
#define le16toh(x) __builtin_bswap16(x)
#define htole16(x) __builtin_bswap16(x)
#endif

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define BIT(nr)        (1ull << (nr))
#define MIN2(x, y)     (((x) < (y)) ? (x) : (y))
#define MAX2(x, y)     (((x) > (y)) ? (x) : (y))
#define ROUNDUP32(x)   (((u32)(x) + 0x1f) & ~0x1f)
#define ROUNDDOWN32(x) (((u32)(x) - 0x1f) & ~0x1f)

#define UNUSED(x)                 (void)(x)
#define MEMBER_SIZE(type, member) sizeof(((type *)0)->member)

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

#ifndef NORETURN
#define NORETURN __attribute__((noreturn))
#endif

#define EGC_WITH_DEBUG

#ifndef EGC_LOG
#define EGC_LOG(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#endif

#define EGC_WARN(fmt, ...) EGC_LOG("[W] " fmt "\n", ##__VA_ARGS__)
#ifdef EGC_WITH_DEBUG
#define EGC_DEBUG(fmt, ...) EGC_LOG("[D] %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)

#define EGC_DEBUG_DATA(data, len) egc_debug_data(__func__, data, len)
#else
#define EGC_DEBUG(fmt, ...)       (void)0
#define EGC_DEBUG_DATA(data, len) (void)0
#endif

static inline void egc_debug_data(const char *prefix, const u8 *data, u16 length)
{
    char buffer[100];
    if (length > 30)
        length = 30;
    char *ptr = buffer;
    for (int i = 0; i < length; i++) {
        ptr += snprintf(ptr, sizeof(buffer) - (ptr - buffer), " %02x", data[i]);
    }
    EGC_LOG("%s%s\n", prefix, buffer);
}

static inline int memmismatch(const void *restrict a, const void *restrict b, int size)
{
    int i = 0;
    while (size) {
        if (((u8 *)a)[i] != ((u8 *)b)[i])
            return i;
        i++;
        size--;
    }
    return i;
}

static inline void reverse_memcpy(void *restrict dst, const void *restrict src, int size)
{
    u8 *d = dst;
    const u8 *s = src;
    for (int i = 0; i < size; i++)
        d[i] = s[size - 1 - i];
}

#endif

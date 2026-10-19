#ifndef EGC_MESSAGE_H
#define EGC_MESSAGE_H

#include <assert.h>

#include "egc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Note that the value 0 (or NULL) is reserved */
typedef intptr_t egc_message_t;

typedef void (*egc_message_cb)(egc_message_t message);

void egc_message_set_handler(egc_message_cb handler);

/* Unlike all other egc_* functions, this one can be called from any thread */
int egc_message_post(egc_message_t message);

#ifdef __cplusplus
}
#endif

#endif

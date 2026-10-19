#include "egc_message.h"
#include "internals.h"
#include "utils.h"

#include <errno.h>

static egc_message_cb s_message_cb = NULL;

int _egc_message_deliver(egc_message_t message)
{
    if (s_message_cb) s_message_cb(message);
    return 0;
}

void egc_message_set_handler(egc_message_cb handler)
{
    s_message_cb = handler;
}

__attribute__((weak))
int egc_message_post(egc_message_t message)
{
    EGC_WARN("egc_message_post not implemented by the platform backend");
    return -ENOSYS;
}

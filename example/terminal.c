#include "terminal.h"

bool quit_requested = false;

#ifdef __wii__

#include <gccore.h>
#include <stdlib.h>

static void *xfb = NULL;
static GXRModeObj *rmode = NULL;

void on_reset_pressed(u32 irq, void *ctx)
{
    quit_requested = true;
}

void terminal_init(void)
{
    VIDEO_Init();
    rmode = VIDEO_GetPreferredMode(NULL);
    xfb = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));
    console_init(xfb, 20, 20, rmode->fbWidth, rmode->xfbHeight, rmode->fbWidth * VI_DISPLAY_PIX_SZ);
    VIDEO_Configure(rmode);
    VIDEO_SetNextFramebuffer(xfb);
    VIDEO_SetBlack(false);
    VIDEO_Flush();
    VIDEO_WaitVSync();
    if (rmode->viTVMode & VI_NON_INTERLACE)
        VIDEO_WaitVSync();

    SYS_SetResetCallback(on_reset_pressed);
}
#else

void terminal_init(void)
{
}

#endif

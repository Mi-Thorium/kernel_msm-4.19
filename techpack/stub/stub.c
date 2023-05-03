// SPDX-License-Identifier: GPL-2.0-only

#include <linux/types.h>

#if IS_ENABLED(CONFIG_TECHPACK_AUDIO_DISABLE)
#if (defined(CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE) || defined(CONFIG_TOUCHSCREEN_SWEEP2WAKE))
bool gesture_incall = false;
#endif
#endif

static void _techpack_stub(void)
{
}

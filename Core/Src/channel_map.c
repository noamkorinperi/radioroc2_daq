#include "channel_map.h"
#include <string.h>

uint8_t g_active_channels[NUM_ACTIVE_CHANNELS] = ACTIVE_CHANNELS;

static uint8_t  s_last_channel = 0;
static bool     s_active_mask[64] = {false};

void channel_map_init(void)
{
    memset(s_active_mask, 0, sizeof(s_active_mask));
    s_last_channel = 0;

    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        uint8_t ch = g_active_channels[i];
        if (ch < 64) {
            s_active_mask[ch] = true;
            if (ch > s_last_channel)
                s_last_channel = ch;
        }
    }
}

bool channel_map_active(uint8_t ch)
{
    if (ch >= 64) return false;
    return s_active_mask[ch];
}

uint8_t channel_map_last(void)  { return s_last_channel; }
uint8_t channel_map_count(void) { return NUM_ACTIVE_CHANNELS; }

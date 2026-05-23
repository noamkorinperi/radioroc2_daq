#ifndef CHANNEL_MAP_H
#define CHANNEL_MAP_H

#include "config.h"

/* ================================================================
 * channel_map.h / channel_map.c
 *
 * Manages the list of active ASIC channels.
 * Key computed values:
 *   channel_map_last()   — highest active channel number
 *                          CK_READ stops here, not at 63
 *   channel_map_active() — true if channel n is in the active list
 * ================================================================ */

/* Initialise from the ACTIVE_CHANNELS list in config.h */
void     channel_map_init(void);

/* Returns true if channel n should be sampled */
bool     channel_map_active(uint8_t ch);

/* Returns the highest active channel number (CK_READ stop point) */
uint8_t  channel_map_last(void);

/* Returns number of active channels */
uint8_t  channel_map_count(void);

/* Raw list, populated by channel_map_init() */
extern uint8_t g_active_channels[NUM_ACTIVE_CHANNELS];

#endif /* CHANNEL_MAP_H */

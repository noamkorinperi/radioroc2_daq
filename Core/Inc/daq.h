#ifndef DAQ_H
#define DAQ_H

/* ================================================================
 * daq.h / daq.c
 *
 * Radiation event acquisition — ISR and main loop logic.
 *
 * Flow:
 *   NOR_T1OC falls → EXTI0 ISR → sets g_event_pending flag
 *   Main loop sees flag → calls daq_process_event()
 *     → RSTN_READ pulse (reset MUX to ch0)
 *     → MUX settle delay
 *     → for each active channel up to last:
 *          CK_READ pulse → wait 30µs → ADC read → store
 *     → RSTN_READ + RESET_N (release hold, reset peak detectors)
 *     → apply temperature and LY(T) correction → convert to keV
 *     → send to PC via USB
 * ================================================================ */

#include "stm32f7xx_hal.h"
#include "config.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------
 * Event flag — set by ISR, cleared by main loop
 * --------------------------------------------------------------- */
extern volatile bool g_event_pending;

/* ---------------------------------------------------------------
 * Raw ADC results per channel (HG and LG)
 * Indexed by position in g_active_channels[], not by channel number
 * --------------------------------------------------------------- */
typedef struct {
    uint8_t  channel;       /* ASIC channel number (0-63)           */
    uint16_t adc_hg;        /* Raw 12-bit ADC value from OUT_AMUXHG */
    uint16_t adc_lg;        /* Raw 12-bit ADC value from OUT_AMUXLG */
    float    energy_kev;    /* Corrected energy in keV              */
    bool     valid;         /* true if this channel had a real hit  */
} ChannelResult_t;

extern ChannelResult_t g_results[NUM_ACTIVE_CHANNELS];

/* ---------------------------------------------------------------
 * Function prototypes
 * --------------------------------------------------------------- */

/**
 * @brief  EXTI0 ISR handler for NOR_T1OC (PA0, falling edge).
 *         Only sets g_event_pending — no processing here.
 *         Place call in stm32f7xx_it.c EXTI0_IRQHandler.
 */
void daq_exti0_irq_handler(void);

/**
 * @brief  Process one pending event. Call from main loop when
 *         g_event_pending is true.
 *         Reads all active channels, applies corrections,
 *         sends results to PC.
 */
void daq_process_event(void);

/**
 * @brief  Returns the index of a channel in g_active_channels[],
 *         or -1 if not active.
 */
int8_t daq_channel_index(uint8_t ch);

#endif /* DAQ_H */

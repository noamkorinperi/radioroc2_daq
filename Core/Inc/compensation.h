#ifndef COMPENSATION_H
#define COMPENSATION_H

/* ================================================================
 * compensation.h / compensation.c
 *
 * Two-path thermal compensation:
 *
 * PATH 1 — Hardware feedback (Step 7):
 *   PI loop drives InDAC per channel to keep Vpeak stable.
 *   This locks SiPM G(T) regardless of temperature.
 *   Updated once per second from main loop.
 *
 * PATH 2 — Software correction (Step 8):
 *   Applies CsI(Tl) LY(T) polynomial correction to each ADC value.
 *   Then converts to keV using calibration factor S_T_REF.
 *   Applied to every event.
 *
 * Temperature source: STM32F722 internal sensor via ADC1_TEMP.
 * ================================================================ */

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

/* NOTE: S_T_REF is defined in config.h (included via compensation.h → config.h).
 *       Do not redefine it here. */

/* ---------------------------------------------------------------
 * Function prototypes
 * --------------------------------------------------------------- */

/**
 * @brief  Initialise compensation state for all active channels.
 *         Sets DAC integral accumulators to zero and DAC to baseline.
 *         Call once during system_init().
 */
void compensation_init(void);

/**
 * @brief  Read the STM32 internal temperature sensor.
 * @retval Temperature in degrees Celsius (float).
 */
float compensation_get_temperature(void);

/**
 * @brief  PATH 1 — PI feedback loop update.
 *         Call once per second from main loop.
 *         Reads latest ADC values from g_results[] and adjusts
 *         InDAC for each active channel via I2C.
 */
void compensation_update_dac(void);

/**
 * @brief  PATH 2 — Apply LY(T) correction and convert to keV.
 *         Call once per event, after ADC read.
 * @param  adc_raw   Raw 12-bit ADC value from OUT_AMUXHG
 * @param  T         Current temperature in °C
 * @retval Corrected energy in keV
 */
float compensation_apply(uint16_t adc_raw, float T);

/**
 * @brief  Returns true if the PI loop has converged for all channels.
 *         Used to gate software correction (PATH 2).
 */
bool compensation_is_converged(void);

#endif /* COMPENSATION_H */

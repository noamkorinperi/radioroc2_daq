#ifndef TIMERS_H
#define TIMERS_H

/* ================================================================
 * timers.h / timers.c
 *
 * Timer management for RADIOROC2 DAQ firmware.
 *
 * TIM1_CH1 → CLK_SM_I2C: 2 MHz continuous square wave on PA8
 *             Required by RADIOROC2 I2C slave core.
 *             Must be running before any I2C transaction.
 *             Configured in CubeMX: ARR=107, CCR1=53, no prescaler.
 *
 * TIM2_CH2 → CK_READ: single pulse of 30 µs on PA1
 *             Each call to timers_ck_read_pulse() fires one pulse.
 *             One Pulse Mode enabled in CubeMX.
 *             Configured in CubeMX: prescaler=215, ARR=29, CCR2=14.
 *
 * TIM3     → 1-second tick for temperature sampling and DAC update.
 *             Configured here in software (not in CubeMX).
 * ================================================================ */

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------
 * External HAL handles — defined in main.c by CubeMX
 * --------------------------------------------------------------- */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* ---------------------------------------------------------------
 * Function prototypes
 * --------------------------------------------------------------- */

/**
 * @brief  Start TIM1_CH1 PWM → CLK_SM_I2C 2 MHz continuous.
 *         Call once during system initialisation, before I2C.
 */
void timers_start_clk_sm_i2c(void);

/**
 * @brief  Stop CLK_SM_I2C (use only for shutdown/debug).
 */
void timers_stop_clk_sm_i2c(void);

/**
 * @brief  Fire one CK_READ pulse (30 µs) on PA1.
 *         Blocks until the pulse is complete (One Pulse Mode).
 *         Call once per channel during the readout sequence.
 */
void timers_ck_read_pulse(void);

/**
 * @brief  Initialise TIM3 as a 1-second periodic interrupt.
 *         The ISR sets a flag checked by the main loop for
 *         temperature sampling and DAC feedback update.
 */
void timers_start_1s_tick(void);

/**
 * @brief  Returns true if the 1-second tick has fired since
 *         the last call. Clears the flag automatically.
 *         Call from the main loop.
 */
bool timers_1s_elapsed(void);

/**
 * @brief  Called from HAL_TIM_PeriodElapsedCallback in main.c
 *         when TIM3 period elapses. Sets the 1-second flag.
 *         See timers.c for merge instructions.
 */
void timers_on_tim3_elapsed(void);

#endif /* TIMERS_H */

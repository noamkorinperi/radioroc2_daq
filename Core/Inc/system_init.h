#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

/* ================================================================
 * system_init.h / system_init.c
 *
 * Full ASIC and system initialisation sequence.
 * Must be called once in main() after HAL_Init() and
 * clock configuration, before the main loop.
 *
 * Sequence:
 *   1. Start CLK_SM_I2C (TIM1) — must come first
 *   2. Reset ASIC I2C slave core (RSTN_I2C)
 *   3. Reset ASIC digital core (RESET_N)
 *   4. Verify ASIC is alive (I2C ping)
 *   5. Configure global registers:
 *        - selTrig / selHoldExt
 *        - delay and slopeTrim (hold window)
 *   6. Configure per-channel registers for all active channels:
 *        - InDAC baseline
 *        - hgGain / lgGain
 *        - shaping time tauHG / tauLG
 *   7. Start 1-second tick timer (TIM3)
 *   8. Enable NOR_T1OC interrupt (EXTI0)
 * ================================================================ */

#include "stm32f7xx_hal.h"
#include <stdbool.h>

/* ---------------------------------------------------------------
 * Return codes
 * --------------------------------------------------------------- */
typedef enum {
    INIT_OK              = 0,
    INIT_ERR_ASIC_NOACK  = 1,   /* ASIC did not respond to I2C ping  */
    INIT_ERR_I2C_FAIL    = 2,   /* I2C write failed during config    */
} InitStatus_t;

/* ---------------------------------------------------------------
 * Function prototypes
 * --------------------------------------------------------------- */

/**
 * @brief  Run the full system and ASIC initialisation sequence.
 * @retval INIT_OK on success, error code otherwise.
 *         On error, an LED or UART message can be used to report.
 */
InitStatus_t system_init(void);

/**
 * @brief  Assert RESET_N low for exactly 20 ns then release.
 *         Used both during init and after each readout cycle.
 *         20 ns at 216 MHz ≈ 5 NOP instructions.
 */
void system_assert_reset_n(void);

/**
 * @brief  Assert RSTN_READ low then high.
 *         Resets the MUX read register to channel 0.
 */
void system_assert_rstn_read(void);

#endif /* SYSTEM_INIT_H */

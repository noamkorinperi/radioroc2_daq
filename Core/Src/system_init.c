/* ================================================================
 * system_init.c
 * ================================================================ */

#include "system_init.h"
#include "config.h"
#include "channel_map.h"
#include "radioroc2_i2c.h"
#include "timers.h"

/* ----------------------------------------------------------------
 * system_assert_reset_n
 *
 * RESET_N is active low. Pulse must be exactly 20 ns minimum.
 * At 216 MHz, one cycle = 4.6 ns → 5 NOPs ≈ 23 ns.
 * ---------------------------------------------------------------- */
void system_assert_reset_n(void)
{
    HAL_GPIO_WritePin(RESET_N_PORT, RESET_N_PIN, GPIO_PIN_RESET);
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();   /* ~23 ns */
    HAL_GPIO_WritePin(RESET_N_PORT, RESET_N_PIN, GPIO_PIN_SET);
}

/* ----------------------------------------------------------------
 * system_assert_rstn_read
 *
 * RSTN_READ is active low. A short pulse resets the MUX read
 * register back to channel 0.
 * ---------------------------------------------------------------- */
void system_assert_rstn_read(void)
{
    HAL_GPIO_WritePin(RSTN_READ_PORT, RSTN_READ_PIN, GPIO_PIN_RESET);
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    HAL_GPIO_WritePin(RSTN_READ_PORT, RSTN_READ_PIN, GPIO_PIN_SET);
    HAL_Delay(1);   /* 1 ms settle before first CK_READ */
}

/* ----------------------------------------------------------------
 * system_init
 * ---------------------------------------------------------------- */
InitStatus_t system_init(void)
{
    HAL_StatusTypeDef i2c_status;

    /* ----------------------------------------------------------
     * Step 1: Start CLK_SM_I2C — MUST come before any I2C access
     * ---------------------------------------------------------- */
    timers_start_clk_sm_i2c();
    HAL_Delay(1);   /* allow clock to stabilise */

    /* ----------------------------------------------------------
     * Step 2: Reset ASIC I2C slave core
     * ---------------------------------------------------------- */
    HAL_GPIO_WritePin(RSTN_I2C_PORT, RSTN_I2C_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(RSTN_I2C_PORT, RSTN_I2C_PIN, GPIO_PIN_SET);
    HAL_Delay(5);

    /* ----------------------------------------------------------
     * Step 3: Reset ASIC digital core (RESET_N)
     * ---------------------------------------------------------- */
    system_assert_reset_n();
    HAL_Delay(10);   /* allow ASIC to complete internal reset */

    /* ----------------------------------------------------------
     * Step 4: Verify ASIC responds to I2C
     * ---------------------------------------------------------- */
    if (!radioroc2_is_alive()) {
        return INIT_ERR_ASIC_NOACK;
    }

    /* ----------------------------------------------------------
     * Step 5: Configure global registers
     * ---------------------------------------------------------- */

    /* selTrig and selHoldExt — see config.h for HOLD_MODE */
    i2c_status = radioroc2_write_global(SC_GLOBAL_SELTRIG,
                                         SELTRIG_REG_VALUE);
    if (i2c_status != HAL_OK) return INIT_ERR_I2C_FAIL;

    /* Hold window: delay=ASIC_DELAY_VAL, slopeTrim=ASIC_SLOPE_TRIM
     * Register SC_GLOBAL_SLOPE: slopeTrim[7:4] | ibi_discri[3:0]=0x4
     * → register value = (ASIC_SLOPE_TRIM << 4) | 0x04             */
    i2c_status = radioroc2_write_global(SC_GLOBAL_DELAY, ASIC_DELAY_VAL);
    if (i2c_status != HAL_OK) return INIT_ERR_I2C_FAIL;

    i2c_status = radioroc2_write_global(SC_GLOBAL_SLOPE,
                                        (ASIC_SLOPE_TRIM << 4) | 0x04);
    if (i2c_status != HAL_OK) return INIT_ERR_I2C_FAIL;

    /* ----------------------------------------------------------
     * Step 6: Configure per-channel registers
     * ---------------------------------------------------------- */
    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        uint8_t ch = g_active_channels[i];

        /* InDAC baseline — SiPM overvoltage starting point */
        i2c_status = radioroc2_write(ch, SC_SUBADD_INDAC, DAC_BASELINE);
        if (i2c_status != HAL_OK) return INIT_ERR_I2C_FAIL;

        /* hgGain[3:0] = 8 (mid range), lgGain[7:4] = 4 (mid range)
         * Combined byte: 0x48                                        */
        i2c_status = radioroc2_write(ch, SC_SUBADD_GAIN, 0x48);
        if (i2c_status != HAL_OK) return INIT_ERR_I2C_FAIL;

        /* Shaping time: tauHG[7:4] = 0x4 (80 ns), tauLG[3:0] = 0x4
         * Combined byte: 0x44
         * CsI(Tl) decay time ~1 µs — use slowShaping if needed     */
        i2c_status = radioroc2_write(ch, SC_SUBADD_TAU, 0x44);
        if (i2c_status != HAL_OK) return INIT_ERR_I2C_FAIL;
    }

    /* ----------------------------------------------------------
     * Step 7: Reset read register to channel 0
     * ---------------------------------------------------------- */
    system_assert_rstn_read();

    /* ----------------------------------------------------------
     * Step 8: Start 1-second tick timer
     * ---------------------------------------------------------- */
    timers_start_1s_tick();

    /* ----------------------------------------------------------
     * Step 9: Enable NOR_T1OC interrupt (EXTI0 on PA0)
     *         NOTE: CubeMX already enables this in MX_GPIO_Init().
     *         We disable it here at startup and re-enable only after
     *         full ASIC configuration to avoid spurious interrupts.
     * ---------------------------------------------------------- */
    HAL_NVIC_DisableIRQ(NOR_T1_EXTI_IRQn);   /* was enabled by CubeMX */
    __HAL_GPIO_EXTI_CLEAR_IT(NOR_T1_PIN);     /* clear any pending bit  */
    HAL_NVIC_EnableIRQ(NOR_T1_EXTI_IRQn);    /* now safe to enable      */

    return INIT_OK;
}

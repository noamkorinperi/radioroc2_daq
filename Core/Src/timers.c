/* ================================================================
 * timers.c
 * ================================================================ */

#include "timers.h"

/* 1-second tick flag — set by TIM3 ISR, cleared by timers_1s_elapsed() */
static volatile bool s_1s_flag = false;

/* ----------------------------------------------------------------
 * timers_start_clk_sm_i2c
 * Starts TIM1_CH1 PWM output on PA8 → 2 MHz continuous.
 * Must be called before the first I2C transaction to RADIOROC2.
 * ---------------------------------------------------------------- */
void timers_start_clk_sm_i2c(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void timers_stop_clk_sm_i2c(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

/* ----------------------------------------------------------------
 * timers_ck_read_pulse
 *
 * Fires one 30 µs pulse on PA1 (TIM2_CH2, One Pulse Mode).
 *
 * Sequence:
 *   1. Ensure counter is reset
 *   2. Start PWM in One Pulse Mode → pulse fires automatically
 *   3. Wait for pulse to complete (poll TIM2 counter back to 0)
 *
 * Total blocking time: ~30 µs (ARR=29 at 1 µs resolution)
 * ---------------------------------------------------------------- */
void timers_ck_read_pulse(void)
{
    /* Reset counter to ensure clean pulse */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* Start one pulse */
    HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2);

    /* Wait until pulse is complete — counter stops at ARR in OPM */
    while (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) == RESET) {}
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
}

/* ----------------------------------------------------------------
 * timers_start_1s_tick
 *
 * Configures TIM3 for a 1-second periodic interrupt.
 *
 * APB1 Timer clock = 108 MHz (APB1 = 54 MHz, x2 for timers)
 * Prescaler = 10799 → tick = 108MHz / (10799+1) = 10 kHz
 * ARR       = 9999  → period = 10000 / 10kHz = 1.000 s
 * ---------------------------------------------------------------- */
void timers_start_1s_tick(void)
{
    TIM_ClockConfigTypeDef  clock_cfg  = {0};
    TIM_MasterConfigTypeDef master_cfg = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 10799;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 9999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);

    clock_cfg.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &clock_cfg);

    master_cfg.MasterOutputTrigger = TIM_TRGO_RESET;
    master_cfg.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &master_cfg);

    /* Enable TIM3 interrupt in NVIC — priority 5 (lowest) */
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    HAL_TIM_Base_Start_IT(&htim3);
}

bool timers_1s_elapsed(void)
{
    if (s_1s_flag) {
        s_1s_flag = false;
        return true;
    }
    return false;
}

/* ----------------------------------------------------------------
 * TIM3 IRQ Handler — called every 1 second
 * Sets the flag for the main loop.
 * Place this in stm32f7xx_it.c or call from there.
 * ---------------------------------------------------------------- */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

/* ----------------------------------------------------------------
 * HAL_TIM_PeriodElapsedCallback
 *
 * IMPORTANT: CubeMX also generates this callback in main.c for
 * SysTick (htim->Instance == TIM1 used by HAL timebase).
 * This implementation handles TIM3 only.
 *
 * In main.c, CubeMX generates:
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *     if (htim->Instance == TIM1) { HAL_IncTick(); }
 *   }
 *
 * MERGE THESE TWO into one function in main.c:
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *     if (htim->Instance == TIM1) { HAL_IncTick(); }
 *     if (htim->Instance == TIM3) { timers_on_tim3_elapsed(); }
 *   }
 * ---------------------------------------------------------------- */
void timers_on_tim3_elapsed(void)
{
    s_1s_flag = true;
}

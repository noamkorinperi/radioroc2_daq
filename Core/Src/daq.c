/* ================================================================
 * daq.c
 * ================================================================ */

#include "daq.h"
#include "channel_map.h"
#include "timers.h"
#include "system_init.h"
#include "compensation.h"
#include "usb_comm.h"
#include "main.h"

/* ---------------------------------------------------------------
 * Global state
 * --------------------------------------------------------------- */
volatile bool      g_event_pending = false;
ChannelResult_t    g_results[NUM_ACTIVE_CHANNELS];

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* ----------------------------------------------------------------
 * daq_exti0_irq_handler
 *
 * Called from EXTI0_IRQHandler in stm32f7xx_it.c.
 * Only sets the flag — all processing in main loop.
 * ---------------------------------------------------------------- */
void daq_exti0_irq_handler(void)
{
    /* Clear EXTI pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(NOR_T1_PIN);

    /* Set flag for main loop */
    g_event_pending = true;
}

/* ----------------------------------------------------------------
 * daq_read_adc
 * Helper: trigger one ADC conversion and return the result.
 * ---------------------------------------------------------------- */
static uint16_t daq_read_adc_hg(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

static uint16_t daq_read_adc_lg(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return val;
}

/* ----------------------------------------------------------------
 * daq_process_event
 *
 * Full readout sequence for one radiation event.
 * Called from main loop when g_event_pending == true.
 * ---------------------------------------------------------------- */
void daq_process_event(void)
{
    /* Clear flag immediately so we don't re-enter */
    g_event_pending = false;

    uint8_t last_ch   = channel_map_last();
    uint8_t mux_pos   = 0;   /* current MUX position (0-63) */
    uint8_t result_idx = 0;

    /* Clear previous results */
    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        g_results[i].valid = false;
    }

    /* ----------------------------------------------------------
     * Step 1: Assert HOLDEXT (external hold mode only)
     *
     * NOTE: NOR_T1OC fires AFTER the ASIC's internal delay box
     * has already asserted hold. In external mode (selHoldExt=1),
     * the ASIC ignores its internal delay and waits for HOLDEXT.
     * We assert HOLDEXT here to keep the peak frozen during readout.
     * Release only after all ADC conversions are complete.
     * ---------------------------------------------------------- */
#if HOLD_MODE == HOLD_MODE_EXTERNAL
    HAL_GPIO_WritePin(HOLDEXT_PORT, HOLDEXT_PIN, GPIO_PIN_SET);
#endif

    /* ----------------------------------------------------------
     * Step 2: Reset MUX to channel 0
     * ---------------------------------------------------------- */
    system_assert_rstn_read();

    /* MUX settle after reset (~2 µs at 216 MHz = 432 cycles) */
    for (volatile uint32_t i = 0; i < 432; i++) { __NOP(); }

    /* ----------------------------------------------------------
     * Step 3: Scan channels 0 → last_ch
     *         Send CK_READ pulse for each position.
     *         Only sample ADC on active channels.
     * ---------------------------------------------------------- */
    for (mux_pos = 0; mux_pos <= last_ch; mux_pos++) {

        /* Fire one CK_READ pulse — advances MUX to mux_pos */
        timers_ck_read_pulse();

        /* Only read ADC if this channel is active */
        if (channel_map_active(mux_pos)) {

            /* ADC sampling happens during the 30µs pulse window.
             * After the pulse, the MUX output is stable → read. */
            uint16_t hg = daq_read_adc_hg();
            uint16_t lg = daq_read_adc_lg();

            /* Store result */
            g_results[result_idx].channel  = mux_pos;
            g_results[result_idx].adc_hg   = hg;
            g_results[result_idx].adc_lg   = lg;
            g_results[result_idx].valid    = true;
            result_idx++;
        }
    }

    /* ----------------------------------------------------------
     * Step 4: Release hold and reset peak detectors
     * ---------------------------------------------------------- */
#if HOLD_MODE == HOLD_MODE_EXTERNAL
    HAL_GPIO_WritePin(HOLDEXT_PORT, HOLDEXT_PIN, GPIO_PIN_RESET);
#endif

    system_assert_rstn_read();   /* reset MUX back to ch0        */
    system_assert_reset_n();     /* reset peak detectors         */

    /* ----------------------------------------------------------
     * Step 5: Apply temperature correction and convert to keV
     * ---------------------------------------------------------- */
    float T = compensation_get_temperature();

    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        if (g_results[i].valid) {
            g_results[i].energy_kev =
                compensation_apply(g_results[i].adc_hg, T);
        }
    }

    /* ----------------------------------------------------------
     * Step 6: Send results to PC via USB
     * ---------------------------------------------------------- */
    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        if (g_results[i].valid) {
            usb_comm_send_event(g_results[i].channel,
                                g_results[i].energy_kev);
        }
    }
}

/* ----------------------------------------------------------------
 * daq_channel_index
 * ---------------------------------------------------------------- */
int8_t daq_channel_index(uint8_t ch)
{
    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        if (g_active_channels[i] == ch) return (int8_t)i;
    }
    return -1;
}

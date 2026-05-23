/* ================================================================
 * compensation.c
 *
 * PATH 1: PI feedback loop → InDAC (locks SiPM gain)
 * PATH 2: LY(T) polynomial correction → keV conversion
 * ================================================================ */

#include "compensation.h"
#include "radioroc2_i2c.h"
#include "channel_map.h"
#include "daq.h"
#include "usb_comm.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;

/* g_s_t_ref is defined in usb_comm.c and runtime-updatable via USB */
extern float g_s_t_ref;

/* ---------------------------------------------------------------
 * Per-channel PI state
 * --------------------------------------------------------------- */
typedef struct {
    float    integral;
    uint8_t  dac_val;
    bool     converged;
    uint32_t warmup_ticks;
} ChannelCompState_t;

static ChannelCompState_t s_state[NUM_ACTIVE_CHANNELS];

/* ---------------------------------------------------------------
 * LY(T) polynomial (normalised at T_REF = 27°C)
 * Source: Saint-Gobain CsI(Tl) datasheet + user-provided fit
 * LY(T) = 0.923 + 3.95e-3*T - 5.56e-5*T^2
 *              - 4.43e-7*T^3 - 7.35e-10*T^4
 * LY_REF = LY(27°C) ≈ 0.9999
 * --------------------------------------------------------------- */
static float ly_normalized(float T)
{
    return (0.923f
          + 3.95e-3f  * T
          - 5.56e-5f  * T * T
          - 4.43e-7f  * T * T * T
          - 7.35e-10f * T * T * T * T) / LY_REF;
}

/* ----------------------------------------------------------------
 * compensation_init
 * ---------------------------------------------------------------- */
void compensation_init(void)
{
    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        s_state[i].integral      = 0.0f;
        s_state[i].dac_val       = DAC_BASELINE;
        s_state[i].converged     = false;
        s_state[i].warmup_ticks  = 0;

        /* Write baseline DAC to ASIC */
        radioroc2_write(g_active_channels[i],
                        SC_SUBADD_INDAC,
                        DAC_BASELINE);
    }
}

/* ----------------------------------------------------------------
 * compensation_get_temperature
 *
 * Reads the STM32F722 internal temperature sensor.
 * Formula from STM32F7 datasheet:
 *   T(°C) = (V_SENSE - V30) / Avg_Slope + 30
 * where:
 *   V30       = 0.76 V   (calibration value at 30°C)
 *   Avg_Slope = 2.5 mV/°C
 *   V_SENSE   = ADC_raw * (3.3 / 4096)  [V]
 * ---------------------------------------------------------------- */
float compensation_get_temperature(void)
{
    /* Switch ADC1 to temperature channel */
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = ADC_CHANNEL_TEMPSENSOR;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLING_CYCLES_TEMP;
    HAL_ADC_ConfigChannel(&hadc1, &cfg);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* Restore ADC1 to IN4 (OUT_AMUXHG) for normal operation */
    cfg.Channel      = ADC_CHANNEL_4;
    cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &cfg);

    float v_sense = raw * (3.3f / 4096.0f);
    float temp    = ((v_sense - 0.76f) / 0.0025f) + 30.0f;
    return temp;
}

/* ----------------------------------------------------------------
 * compensation_update_dac — PATH 1
 *
 * PI feedback loop: drives InDAC until Vpeak_raw ≈ V_SETPOINT.
 * Called once per second from main loop.
 * Uses the most recent ADC HG reading from g_results[].
 * ---------------------------------------------------------------- */
void compensation_update_dac(void)
{
    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {

        /* Skip channels with no valid measurement yet */
        if (!g_results[i].valid) continue;

        s_state[i].warmup_ticks++;

        /* Warmup period: don't apply correction yet */
        if (s_state[i].warmup_ticks < WARMUP_TICKS) {
            s_state[i].converged = false;
            continue;
        }

        float error = (float)(V_SETPOINT - g_results[i].adc_hg);

        /* PI update */
        s_state[i].integral += error;

        /* Anti-windup: clamp integral */
        if (s_state[i].integral >  5000.0f) s_state[i].integral =  5000.0f;
        if (s_state[i].integral < -5000.0f) s_state[i].integral = -5000.0f;

        float correction = KP * error + KI * s_state[i].integral;
        int new_dac = (int)s_state[i].dac_val + (int)correction;

        /* Clamp to hardware range */
        if (new_dac < DAC_MIN) new_dac = DAC_MIN;
        if (new_dac > DAC_MAX) new_dac = DAC_MAX;

        /* Only write to ASIC if value changed */
        if ((uint8_t)new_dac != s_state[i].dac_val) {
            s_state[i].dac_val = (uint8_t)new_dac;
            radioroc2_write(g_active_channels[i],
                            SC_SUBADD_INDAC,
                            s_state[i].dac_val);
        }

        /* Update convergence flag */
        s_state[i].converged =
            (fabsf(error) < (float)CONVERGENCE_THR);
    }
}

/* ----------------------------------------------------------------
 * compensation_apply — PATH 2
 *
 * 1. Check convergence — if loop not settled, return -1 (invalid)
 * 2. Apply LY(T) correction: adc_corrected = adc_raw / LY_norm(T)
 * 3. Convert to keV: energy = adc_corrected / S_T_REF
 * ---------------------------------------------------------------- */
float compensation_apply(uint16_t adc_raw, float T)
{
    /* Do not output corrected values until DAC loop has converged */
    if (!compensation_is_converged()) {
        return -1.0f;   /* caller should mark event as invalid */
    }

    float ly = ly_normalized(T);

    /* Guard against division by zero or extreme temperature */
    if (ly < 0.1f) ly = 0.1f;

    float adc_corrected = (float)adc_raw / ly;
    float energy_kev    = adc_corrected / g_s_t_ref;

    return energy_kev;
}

/* ----------------------------------------------------------------
 * compensation_is_converged
 * Returns true only if ALL active channels have converged.
 * ---------------------------------------------------------------- */
bool compensation_is_converged(void)
{
    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        if (!s_state[i].converged) return false;
    }
    return true;
}

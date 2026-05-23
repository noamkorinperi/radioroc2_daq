/* ================================================================
 * radioroc2_i2c.c
 *
 * Implementation of the RADIOROC2 non-standard I2C protocol.
 *
 * Protocol summary (per transaction):
 *   R0: sends channel number   (address LSB)
 *   R1: sends sub-address      (address MSB / register select)
 *   R2: sends or receives data
 *
 * Each phase is a FULL separate I2C transaction:
 *   START → address byte → data byte → STOP
 *
 * References: RADIOROC2 Datasheet v2.0, Section "I2C configuration"
 * ================================================================ */

#include "radioroc2_i2c.h"
#include "channel_map.h"

/* Private handle — set once by radioroc2_i2c_init() */
static I2C_HandleTypeDef *s_hi2c = NULL;

/* ----------------------------------------------------------------
 * radioroc2_i2c_init
 * ---------------------------------------------------------------- */
void radioroc2_i2c_init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
}

/* ----------------------------------------------------------------
 * radioroc2_write
 *
 * Three separate HAL_I2C_Master_Transmit calls, each producing
 * its own START + address + data + STOP sequence.
 *
 * The RADIOROC2 encodes the phase in the lower bits of the I2C
 * address field, which is why we use three different addr values.
 * ---------------------------------------------------------------- */
HAL_StatusTypeDef radioroc2_write(uint8_t channel,
                                   uint8_t subadd,
                                   uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t buf;

    /* --- Phase R0: send channel number (full address LSB) --- */
    buf = channel;
    status = HAL_I2C_Master_Transmit(s_hi2c,
                                      RADIOROC2_ADDR_R0_W,
                                      &buf, 1,
                                      RADIOROC2_I2C_TIMEOUT);
    if (status != HAL_OK) return status;

    /* --- Phase R1: send sub-address (full address MSB) --- */
    buf = subadd;
    status = HAL_I2C_Master_Transmit(s_hi2c,
                                      RADIOROC2_ADDR_R1_W,
                                      &buf, 1,
                                      RADIOROC2_I2C_TIMEOUT);
    if (status != HAL_OK) return status;

    /* --- Phase R2: send data byte --- */
    buf = data;
    status = HAL_I2C_Master_Transmit(s_hi2c,
                                      RADIOROC2_ADDR_R2_W,
                                      &buf, 1,
                                      RADIOROC2_I2C_TIMEOUT);
    return status;
}

/* ----------------------------------------------------------------
 * radioroc2_read
 *
 * R0 and R1 are write phases (same as write transaction).
 * R2 switches to a read: HAL_I2C_Master_Receive with R2_R address.
 * ---------------------------------------------------------------- */
HAL_StatusTypeDef radioroc2_read(uint8_t  channel,
                                  uint8_t  subadd,
                                  uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t buf;

    /* --- Phase R0: send channel number --- */
    buf = channel;
    status = HAL_I2C_Master_Transmit(s_hi2c,
                                      RADIOROC2_ADDR_R0_W,
                                      &buf, 1,
                                      RADIOROC2_I2C_TIMEOUT);
    if (status != HAL_OK) return status;

    /* --- Phase R1: send sub-address --- */
    buf = subadd;
    status = HAL_I2C_Master_Transmit(s_hi2c,
                                      RADIOROC2_ADDR_R1_W,
                                      &buf, 1,
                                      RADIOROC2_I2C_TIMEOUT);
    if (status != HAL_OK) return status;

    /* --- Phase R2: read data byte --- */
    status = HAL_I2C_Master_Receive(s_hi2c,
                                     RADIOROC2_ADDR_R2_R,
                                     data, 1,
                                     RADIOROC2_I2C_TIMEOUT);
    return status;
}

/* ----------------------------------------------------------------
 * radioroc2_write_global
 * ---------------------------------------------------------------- */
HAL_StatusTypeDef radioroc2_write_global(uint8_t subadd, uint8_t data)
{
    return radioroc2_write(RADIOROC2_GLOBAL_ADDR, subadd, data);
}

/* ----------------------------------------------------------------
 * radioroc2_read_global
 * ---------------------------------------------------------------- */
HAL_StatusTypeDef radioroc2_read_global(uint8_t subadd, uint8_t *data)
{
    return radioroc2_read(RADIOROC2_GLOBAL_ADDR, subadd, data);
}

/* ----------------------------------------------------------------
 * radioroc2_write_all_channels
 *
 * Iterates over active channels from channel_map and writes
 * the same value to each one.
 * ---------------------------------------------------------------- */
HAL_StatusTypeDef radioroc2_write_all_channels(uint8_t subadd,
                                                uint8_t data)
{
    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t i = 0; i < NUM_ACTIVE_CHANNELS; i++) {
        uint8_t ch = g_active_channels[i];
        HAL_StatusTypeDef s = radioroc2_write(ch, subadd, data);
        if (s != HAL_OK) {
            status = s;   /* record error but continue writing others */
        }
    }

    return status;
}

/* ----------------------------------------------------------------
 * radioroc2_is_alive
 *
 * Sends R0 phase only and checks for ACK.
 * Used during init to verify the ASIC is responding.
 * ---------------------------------------------------------------- */
bool radioroc2_is_alive(void)
{
    uint8_t buf = 0x00;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
                                    s_hi2c,
                                    RADIOROC2_ADDR_R0_W,
                                    &buf, 1,
                                    RADIOROC2_I2C_TIMEOUT);
    return (status == HAL_OK);
}

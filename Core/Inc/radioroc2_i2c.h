#ifndef RADIOROC2_I2C_H
#define RADIOROC2_I2C_H

/* ================================================================
 * radioroc2_i2c.h
 *
 * Low-level I2C driver for the RADIOROC2 ASIC.
 *
 * The RADIOROC2 uses a NON-STANDARD I2C protocol.
 * Every read/write consists of THREE separate I2C transactions,
 * each with its own START and STOP condition:
 *
 *   R0: START | ChipID + 000 + W | channel (addr LSB)  | STOP
 *   R1: START | ChipID + 001 + W | sub-address (MSB)   | STOP
 *   R2: START | ChipID + 010 + W | data byte (write)   | STOP
 *        or
 *   R2: START | ChipID + 010 + R | read data byte      | STOP
 *
 * CHIP_ID<0:3> = 0001 (hardwired on the RADIOROC2 board)
 * 7-bit base address = 0b0001_000 = 0x08
 *
 * The lower 3 bits of the address byte select the phase:
 *   R0 write: 0x08 << 1 | 0x00 = 0x10
 *   R1 write: 0x08 << 1 | 0x02 = 0x12  (bit1 set)
 *   R2 write: 0x08 << 1 | 0x04 = 0x14  (bit2 set)
 *   R2 read:  0x08 << 1 | 0x05 = 0x15  (bit2 set + R/W=1)
 *
 * References: RADIOROC2 Datasheet v2.0, Figure 3, Figure 4
 * ================================================================ */

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------
 * I2C address constants
 * --------------------------------------------------------------- */
#define RADIOROC2_CHIP_ID        0x08        /* 7-bit base address  */
#define RADIOROC2_ADDR_R0_W      (0x10)      /* Phase R0 write      */
#define RADIOROC2_ADDR_R1_W      (0x12)      /* Phase R1 write      */
#define RADIOROC2_ADDR_R2_W      (0x14)      /* Phase R2 write      */
#define RADIOROC2_ADDR_R2_R      (0x15)      /* Phase R2 read       */

/* Global register address (used instead of channel number) */
#define RADIOROC2_GLOBAL_ADDR    65

/* ---------------------------------------------------------------
 * Per-channel slow control sub-addresses
 * (Table 4, RADIOROC2 Datasheet v2.0)
 * --------------------------------------------------------------- */
#define SC_SUBADD_INDAC          0    /* InDAC[7:0] — SiPM bias trim        */
#define SC_SUBADD_PATGAIN        1    /* patGain[5:0] — time preamp gain     */
#define SC_SUBADD_GAIN           2    /* hgGain[3:0] lgGain[7:4]            */
#define SC_SUBADD_TAU            3    /* tauHG[7:4] tauLG[3:0]              */
#define SC_SUBADD_DAC1_LOW       1    /* DAC1[7:0] — time trigger low        */
#define SC_SUBADD_DAC1_HIGH      2    /* DAC1[9:8] DAC2[5:0]                */
#define SC_SUBADD_DACQ           3    /* DacQ[3:0] charge trigger           */

/* ---------------------------------------------------------------
 * Global slow control sub-addresses (address = 65)
 * --------------------------------------------------------------- */
#define SC_GLOBAL_VREF           7    /* vref[3:0] EN_th1 EN_th2 EN_thQ     */
#define SC_GLOBAL_DELAY          8    /* delay[7:0]                          */
#define SC_GLOBAL_SLOPE          9    /* slopeTrim[7:4] ibi_discri[3:0]     */
#define SC_GLOBAL_SELTRIG        12   /* selHoldExt[4] selTrig[3:0]         */

/* ---------------------------------------------------------------
 * I2C timeout
 * --------------------------------------------------------------- */
#define RADIOROC2_I2C_TIMEOUT    10   /* ms */

/* ---------------------------------------------------------------
 * Function prototypes
 * --------------------------------------------------------------- */

/**
 * @brief  Initialise the driver with the HAL I2C handle.
 *         Must be called once before any other function.
 * @param  hi2c  Pointer to the HAL I2C handle (hi2c1)
 */
void radioroc2_i2c_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Write one byte to a per-channel slow control register.
 * @param  channel  ASIC channel number (0-63) or RADIOROC2_GLOBAL_ADDR
 * @param  subadd   Sub-address (register within the channel)
 * @param  data     Byte to write
 * @retval HAL_OK on success, HAL_ERROR or HAL_TIMEOUT on failure
 */
HAL_StatusTypeDef radioroc2_write(uint8_t channel,
                                   uint8_t subadd,
                                   uint8_t data);

/**
 * @brief  Read one byte from a per-channel slow control register.
 * @param  channel  ASIC channel number (0-63) or RADIOROC2_GLOBAL_ADDR
 * @param  subadd   Sub-address
 * @param  data     Pointer to store the received byte
 * @retval HAL_OK on success, HAL_ERROR or HAL_TIMEOUT on failure
 */
HAL_StatusTypeDef radioroc2_read(uint8_t  channel,
                                  uint8_t  subadd,
                                  uint8_t *data);

/**
 * @brief  Write one byte to a global slow control register.
 *         Equivalent to radioroc2_write(RADIOROC2_GLOBAL_ADDR, ...)
 */
HAL_StatusTypeDef radioroc2_write_global(uint8_t subadd, uint8_t data);

/**
 * @brief  Read one byte from a global slow control register.
 */
HAL_StatusTypeDef radioroc2_read_global(uint8_t subadd, uint8_t *data);

/**
 * @brief  Write the same value to all active channels.
 *         Uses the active channel list from channel_map.
 * @param  subadd  Sub-address
 * @param  data    Byte to write
 * @retval HAL_OK if all writes succeeded, HAL_ERROR if any failed
 */
HAL_StatusTypeDef radioroc2_write_all_channels(uint8_t subadd, uint8_t data);

/**
 * @brief  Check if the ASIC I2C slave is responding.
 * @retval true if ACK received, false otherwise
 */
bool radioroc2_is_alive(void);

#endif /* RADIOROC2_I2C_H */

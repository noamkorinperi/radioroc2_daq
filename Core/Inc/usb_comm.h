#ifndef USB_COMM_H
#define USB_COMM_H

/* ================================================================
 * usb_comm.h / usb_comm.c
 *
 * USB CDC communication between STM32 and PC.
 *
 * PACKET FORMAT (STM32 → PC, per event):
 * ┌────────┬─────────┬──────────────┬──────────┐
 * │ Byte 0 │ Byte 1  │ Bytes 2-5    │ Byte 6   │
 * │ 0xAA   │ channel │ keV (float)  │ checksum │
 * │ START  │ (0-63)  │ little-endian│ XOR 1-5  │
 * └────────┴─────────┴──────────────┴──────────┘
 * Total: 7 bytes per event.
 * keV = -1.0f (0xBF800000) signals an invalid/unconverged event.
 *
 * COMMAND FORMAT (PC → STM32):
 * ┌────────┬──────────┬────────────┬──────────┐
 * │ Byte 0 │ Byte 1   │ Bytes 2-5  │ Byte 6   │
 * │ 0xBB   │ cmd_type │ parameter  │ checksum │
 * └────────┴──────────┴────────────┴──────────┘
 *
 * Command types (Byte 1):
 *   0x01 = SET_COUNT_MODE  — param (uint32) = N events
 *   0x02 = SET_TIME_MODE   — param (uint32) = T seconds
 *   0x03 = SET_CALIB       — param (float)  = new S_T_REF
 *   0x04 = RESET           — param ignored, reset system
 * ================================================================ */

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------
 * Packet constants
 * --------------------------------------------------------------- */
#define USB_PKT_START_EVENT   0xAA
#define USB_PKT_START_CMD     0xBB
#define USB_PKT_SIZE          7

/* ---------------------------------------------------------------
 * Command types
 * --------------------------------------------------------------- */
#define USB_CMD_SET_COUNT     0x01
#define USB_CMD_SET_TIME      0x02
#define USB_CMD_SET_CALIB     0x03
#define USB_CMD_RESET         0x04

/* ---------------------------------------------------------------
 * Histogram mode
 * --------------------------------------------------------------- */
typedef enum {
    HIST_MODE_COUNT = 0,   /* display after N events  */
    HIST_MODE_TIME  = 1,   /* display after T seconds */
} HistMode_t;

extern HistMode_t g_hist_mode;
extern uint32_t   g_hist_param;   /* N events or T seconds */
extern float      g_s_t_ref;      /* calibration factor, runtime-updatable */

/* ---------------------------------------------------------------
 * Function prototypes
 * --------------------------------------------------------------- */

/**
 * @brief  Initialise USB comm state. Call once during system_init.
 */
void usb_comm_init(void);

/**
 * @brief  Send one event packet (7 bytes) to PC.
 * @param  channel     ASIC channel number (0-63)
 * @param  energy_kev  Corrected energy in keV (-1.0f = invalid)
 */
void usb_comm_send_event(uint8_t channel, float energy_kev);

/**
 * @brief  Send a histogram-ready notification to PC.
 *         PC uses this as the signal to display the histogram.
 */
void usb_comm_send_histogram_ready(void);

/**
 * @brief  Poll for incoming commands from PC.
 *         Call from main loop. Processes one command per call.
 */
void usb_comm_poll_commands(void);

/**
 * @brief  Returns true if the histogram trigger condition is met
 *         (N events reached or T seconds elapsed).
 *         Resets the counter/timer after returning true.
 */
bool usb_comm_histogram_due(void);

#endif /* USB_COMM_H */

#ifndef MAIN_APP_H
#define MAIN_APP_H

/* ================================================================
 * main_app.h
 *
 * Declarations for the application entry points defined in
 * main_app.c.  Include this in main.c (USER CODE BEGIN Includes)
 * so the compiler sees the prototypes before the call sites.
 * ================================================================ */

/**
 * @brief  One-time application initialisation.
 *         Call from main() after all CubeMX peripheral inits and
 *         before the infinite loop.
 *         Runs: channel_map_init, radioroc2_i2c_init,
 *               compensation_init, usb_comm_init, system_init.
 *         Blocks forever (LED blink) on ASIC init failure.
 */
void app_init(void);

/**
 * @brief  Application main loop — never returns.
 *         Call from main() in place of the while(1) body.
 *         Handles: event acquisition, 1-second thermal feedback,
 *                  histogram trigger, and USB command processing.
 */
void app_run(void);

#endif /* MAIN_APP_H */

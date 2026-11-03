/** @file app_main.h
 *
 * @brief Application entry point for the AAI03 robot car.
 *
 * @par
 * micro T-Kernel calls usermain() (a weak default in the kernel) from its
 * initial task once the kernel and the port's device services run.  This
 * module brings up the board, the shared kernel objects and every
 * subsystem, then reports which subsystems failed to start.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>
#include <tk/tkernel.h>

// Bits of app_main_boot_failures(), one per start-up step.
#define APP_MAIN_FAIL_IPC           0x0001u
#define APP_MAIN_FAIL_COMMS         0x0002u
#define APP_MAIN_FAIL_TELEMETRY     0x0004u
#define APP_MAIN_FAIL_MOTION        0x0008u
#define APP_MAIN_FAIL_LINE          0x0010u
#define APP_MAIN_FAIL_BARCODE       0x0020u
#define APP_MAIN_FAIL_IMU           0x0040u
#define APP_MAIN_FAIL_OBSTACLE      0x0080u
#define APP_MAIN_FAIL_VEHICLE       0x0100u
#define APP_MAIN_FAIL_GPIO_IRQ      0x0200u
#define APP_MAIN_FAIL_TICK          0x0400u
#define APP_MAIN_FAIL_TEST          0x0800u

INT      usermain(void);
uint32_t app_main_boot_failures(void);

#endif /* APP_MAIN_H */

/*** end of file ***/


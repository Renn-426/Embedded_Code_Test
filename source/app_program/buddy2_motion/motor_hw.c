/** @file motor_hw.c
 *
 * @brief Robo Pico dual DC-motor driver on RP2040 PWM.
 *
 * @par
 * The Robo Pico H-bridge takes two inputs per motor: IN_A high with IN_B low
 * drives forward, the reverse drives backward, both low coasts and both high
 * brakes.  Speed is set by PWM on the active input at MOTOR_PWM_HZ.
 * Reference: Cytron Robo Pico datasheet, motor driver section.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "robot_cfg.h"
#include "tune.h"
#include "motor_hw.h"

// clk_sys runs at 125 MHz (port cpu_clock.c), so one PWM count is 8 ns.
#define CLK_SYS_HZ              125000000u
#define PWM_TOP                 ((CLK_SYS_HZ / MOTOR_PWM_HZ) - 1u)
#define PWM_COUNTS              (PWM_TOP + 1u)
#define PERMILLE_SCALE          1000u

static void drive_pair(uint32_t pin_a, uint32_t pin_b, int32_t permille,
                       bool b_reversed);
static void set_level(uint32_t pin, uint32_t level);
static void setup_pin(uint32_t pin);


/*!
 * @brief Configure the four motor pins for PWM and stop both motors.
 */
void
motor_hw_init (void)
{
    setup_pin(PIN_MOTOR_LEFT_A);
    setup_pin(PIN_MOTOR_LEFT_B);
    setup_pin(PIN_MOTOR_RIGHT_A);
    setup_pin(PIN_MOTOR_RIGHT_B);
    motor_hw_set(0, 0);
}


/*!
 * @brief Set both motor drives.
 *
 * @param[in] left_permille   -1000 (full reverse) to +1000 (full forward).
 * @param[in] right_permille  -1000 (full reverse) to +1000 (full forward).
 *
 * @par
 * Zero coasts the motor.  Values beyond MOTOR_MAX_PERMILLE are clamped.
 */
void
motor_hw_set (int32_t left_permille, int32_t right_permille)
{
    drive_pair(PIN_MOTOR_LEFT_A, PIN_MOTOR_LEFT_B, left_permille,
               (0u != MOTOR_LEFT_REVERSED));
    drive_pair(PIN_MOTOR_RIGHT_A, PIN_MOTOR_RIGHT_B, right_permille,
               (0u != MOTOR_RIGHT_REVERSED));
}


/*!
 * @brief Short both motors (both inputs high) for an active stop.
 */
void
motor_hw_brake (void)
{
    set_level(PIN_MOTOR_LEFT_A, PWM_COUNTS);
    set_level(PIN_MOTOR_LEFT_B, PWM_COUNTS);
    set_level(PIN_MOTOR_RIGHT_A, PWM_COUNTS);
    set_level(PIN_MOTOR_RIGHT_B, PWM_COUNTS);
}


/*!
 * @brief Drive one motor's input pair.
 *
 * @param[in] pin_a       Forward input pin.
 * @param[in] pin_b       Reverse input pin.
 * @param[in] permille    Signed drive, clamped to +/-MOTOR_MAX_PERMILLE.
 * @param[in] b_reversed  true if the motor is mounted the other way round.
 */
static void
drive_pair (uint32_t pin_a, uint32_t pin_b, int32_t permille,
            bool b_reversed)
{
    int32_t const max_drive = tune_get(TUNE_MOTOR_MAX);  // Live setting.
    int32_t       drive     = permille;

    if (b_reversed)
    {
        drive = -drive;
    }

    // Never above the speed limit (MOTOR_MAX_PERMILLE, tunable live).
    if (drive > max_drive)
    {
        drive = max_drive;
    }
    else if (drive < -max_drive)
    {
        drive = -max_drive;
    }
    else
    {
        // Inside the valid range.
    }

    if (drive >= 0)
    {
        // drive is in [0, 1000], so the conversion is exact.
        set_level(pin_a, (PWM_COUNTS * (uint32_t) drive) / PERMILLE_SCALE);
        set_level(pin_b, 0u);
    }
    else
    {
        // -drive is in [1, 1000], so the conversion is exact.
        set_level(pin_a, 0u);
        set_level(pin_b, (PWM_COUNTS * (uint32_t) (-drive)) /
                           PERMILLE_SCALE);
    }
}


/*!
 * @brief Set the compare level of the PWM channel driving a pin.
 *
 * @param[in] pin    Motor input pin.
 * @param[in] level  Compare value; PWM_COUNTS holds the output high.
 */
static void
set_level (uint32_t pin, uint32_t level)
{
    // level never exceeds PWM_COUNTS (12500), so it fits the 16-bit counter.
    pwm_set_gpio_level(pin, (uint16_t) level);
}


/*!
 * @brief Route a pin to its PWM slice and start the slice.
 *
 * @param[in] pin  Motor input pin.
 */
static void
setup_pin (uint32_t pin)
{
    uint32_t const slice = pwm_gpio_to_slice_num(pin);

    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_clkdiv_int_frac(slice, 1u, 0u);
    pwm_set_wrap(slice, (uint16_t) PWM_TOP);  // 12499 fits in 16 bits.
    pwm_set_gpio_level(pin, 0u);
    pwm_set_enabled(slice, true);
}

/*** end of file ***/


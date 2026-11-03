/** @file servo_hw.c
 *
 * @brief Hobby servo driver: 50 Hz PWM with a 500-2500 us pulse.
 *
 * @par
 * clk_sys (125 MHz) is divided by 125 so one PWM count is 1 us; a wrap of
 * 20000 counts gives the 20 ms servo frame, and the compare level is the
 * pulse width in microseconds.  The servo uses its own PWM slice (7), so
 * it never disturbs the motor PWM on slices 4 and 5.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "robot_cfg.h"
#include "servo_hw.h"

#define CLOCK_DIVIDER           125u
#define FRAME_COUNTS            20000u


/*!
 * @brief Start the 50 Hz servo signal at the centre position.
 */
void
servo_hw_init (void)
{
    uint32_t const slice = pwm_gpio_to_slice_num(PIN_SERVO);

    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    pwm_set_clkdiv_int_frac(slice, CLOCK_DIVIDER, 0u);
    pwm_set_wrap(slice, (uint16_t) (FRAME_COUNTS - 1u));  // 19999 fits.
    servo_hw_set_angle(SERVO_CENTRE_DEG);
    pwm_set_enabled(slice, true);
}


/*!
 * @brief Command the servo to an angle.
 *
 * @param[in] angle_deg  0 to 180 degrees; larger values are clamped.
 */
void
servo_hw_set_angle (uint32_t angle_deg)
{
    uint32_t const angle    = (angle_deg > SERVO_HW_MAX_DEG) ?
                              SERVO_HW_MAX_DEG : angle_deg;
    uint32_t const pulse_us = SERVO_MIN_US +
                              (((SERVO_MAX_US - SERVO_MIN_US) * angle) /
                               SERVO_HW_MAX_DEG);

    // pulse_us is at most 2500, so it fits the 16-bit compare register.
    pwm_set_gpio_level(PIN_SERVO, (uint16_t) pulse_us);
}

/*** end of file ***/


/** @file lsm303_hw.h
 *
 * @brief LSM303DLHC accelerometer and magnetometer on I2C0 (GP0 SDA,
 *        GP1 SCL).
 *
 * @par
 * Owner: Buddy 4.  Only the IMU Task may call these functions (Week 6:
 * "Only IMU acquires this sensor").  The sensor has no gyroscope.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef LSM303_HW_H
#define LSM303_HW_H

#include <stdint.h>
#include <stdbool.h>

#define LSM303_HW_AXES              3u
#define LSM303_HW_AXIS_X            0u
#define LSM303_HW_AXIS_Y            1u
#define LSM303_HW_AXIS_Z            2u

bool lsm303_hw_init(void);
bool lsm303_hw_read_accel(int32_t * p_accel_mg);
bool lsm303_hw_read_mag(int32_t * p_field);

#endif /* LSM303_HW_H */

/*** end of file ***/


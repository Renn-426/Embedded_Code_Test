/** @file tune.h
 *
 * @brief Live-tunable settings, changed over MQTT while the robot runs.
 *
 * @par
 * Each setting starts at its value in robot_cfg.h.  An MQTT TUNE command
 * (tools/tlm_decode.py tune ...) changes it immediately; the change is lost
 * on reset, so copy a good value back into robot_cfg.h afterwards.
 *
 * @par
 * Concurrency: the Communication side is the only writer (one aligned
 * 32-bit store per setting) and every reader takes one 32-bit load, so no
 * lock is needed on the single-core RP2040.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef TUNE_H
#define TUNE_H

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief Tunable settings.  The numbers are the MQTT parameter ids.
 */
typedef enum
{
    TUNE_MOTOR_MAX = 0,         // Motor power limit, permille (250..1000).
    TUNE_SPEED_FOLLOW,          // Line-following speed, mm/s (30..600).
    TUNE_SPEED_SLOW,            // Straight moves (incl. reverse), mm/s.
    TUNE_SPEED_TURN,            // On-the-spot turns, mm/s (30..600).
    TUNE_STEER_KP,              // Steering P gain, milli (0..3000).
    TUNE_STEER_KD,              // Steering D gain, milli (0..3000).
    TUNE_STEER_DIR,             // Steering sign, -1 or 1.
    TUNE_ENC_UM_PER_EDGE,       // Wheel travel per encoder edge, um.
    TUNE_TRACK_MM,              // Wheel-to-wheel distance, mm (turn angle).
    TUNE_REVERSE_MM,            // Reverse distance on REVERSE_RETRY, mm.
    TUNE_COUNT
} tune_param_t;

int32_t tune_get(tune_param_t param);
bool    tune_set(uint32_t param, int32_t value);

#endif /* TUNE_H */

/*** end of file ***/


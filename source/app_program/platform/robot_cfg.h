/** @file robot_cfg.h
 *
 * @brief Board wiring and tuning constants for the AAI03 robot car.
 *
 * @par
 * This header is the single place where pins, mechanical dimensions, task
 * periods and tuning values are defined.  It is shared by every subsystem and
 * deliberately contains constants only, so it can be included from both the
 * kernel-side modules and the Pico SDK hardware modules.
 *
 * @par
 * Wiring source: "current wiring record 2026-10-20" plus the GP26 conflict
 * fix agreed after Tribunal 1 (centre IR AO moved to GP27, ultrasonic moved to
 * the GP4/GP5 Grove port).  Values marked "TBC" must be measured on the
 * finished chassis before they are trusted.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef ROBOT_CFG_H
#define ROBOT_CFG_H

/* ------------------------------------------------------------------------ */
/* Pin map (Raspberry Pi Pico W on Cytron Robo Pico).                      */
/* ------------------------------------------------------------------------ */

// Buddy 4: LSM303DLHC accelerometer/magnetometer on I2C0 (Grove port 1).
#define PIN_IMU_SDA                 0u
#define PIN_IMU_SCL                 1u

// Buddy 3: IR1 digital output (left line sensor).  IR1 AO on GP2 is not an
// ADC pin and is left unused with its input buffer disabled.
#define PIN_IR_LEFT_DO              3u
#define PIN_IR1_AO_UNUSED           2u

// Buddy 5: HC-SR04 on the GP4/GP5 Grove port.  NOTE: confirm which wire is
// TRIG and which is ECHO; swap these two values if the trace says otherwise.
#define PIN_SONAR_ECHO              4u
#define PIN_SONAR_TRIG              5u

// Buddy 2: right wheel encoder output on GP6.  Channel B is only read when
// ENC_QUADRATURE is 1; GP16 is free on this robot and stands in for it.
#define PIN_ENC_RIGHT_A             6u
#define PIN_ENC_RIGHT_B             16u

// Buddy 3: IR3 digital output (dedicated barcode sensor).  IR3 AO on GP7 is
// not an ADC pin and is left unused with its input buffer disabled.
#define PIN_IR_BARCODE_DO           28u
#define PIN_IR3_AO_UNUSED           7u

// Buddy 2: Robo Pico motor driver.  M1 = GP8/GP9, M2 = GP10/GP11.  TBC which
// motor is on the left; swap the pairs below if the robot spins on FORWARD.
#define PIN_MOTOR_LEFT_A            8u
#define PIN_MOTOR_LEFT_B            9u
#define PIN_MOTOR_RIGHT_A           10u
#define PIN_MOTOR_RIGHT_B           11u

// Buddy 5: scanning servo (Robo Pico servo header GP15).
#define PIN_SERVO                   15u

// Buddy 2: left wheel encoder output on GP26 (Grove port 6, shared with the
// centre IR AO on GP27), found by turning the wheel and watching the GPIO
// status LEDs.  Channel B (quadrature only) uses the free GP17.
#define PIN_ENC_LEFT_A              26u
#define PIN_ENC_LEFT_B              17u

// Integration: Robo Pico on-board push button GP20 (active low) = START.
#define PIN_START_BUTTON            20u

// Buddy 3: centre IR2 analogue output on GP27 (ADC input 1).
#define PIN_IR_CENTRE_AO            27u
#define ADC_INPUT_IR_CENTRE         1u

// Buddy 3: barcode IR module.  Its digital output cannot see a bar narrower
// than the sensor's spot on the paper, so its analogue output is wired to
// GP28 (ADC input 2) instead and the firmware finds the edges itself.  Set
// BARCODE_SENSOR_ANALOGUE to 0u to go back to the digital output on GP28.
#ifndef BARCODE_SENSOR_ANALOGUE
#define BARCODE_SENSOR_ANALOGUE     1u
#endif
#define PIN_IR_BARCODE_AO           28u
#define ADC_INPUT_IR_BARCODE        2u

/* ------------------------------------------------------------------------ */
/* Sensor polarity.                                                        */
/* ------------------------------------------------------------------------ */

// TCRT5000 modules: black tape reflects little IR, so the comparator output
// (DO) is high and the analogue output (AO) is high over black.  Set to 0u if
// a module on this robot behaves the other way round.
#define IR_DO_BLACK_IS_HIGH         1u
#define IR_AO_BLACK_IS_HIGH         1u

// Encoders: 0u = one channel counted, direction taken from the motor command
// (slotted-disc LM393 module); 1u = quadrature A/B with direction from B.
#define ENC_QUADRATURE              0u

/* ------------------------------------------------------------------------ */
/* Mechanical dimensions (TBC: measure on the finished chassis).           */
/* ------------------------------------------------------------------------ */

// Encoder resolution.  Measured on the robot on 2026-11-02 by pushing it
// 300 mm: 1693 edges on each wheel, so 177 um per edge.  The encoders are
// on the motor shaft behind the gearbox, about 1150 edges per wheel
// revolution, not the 40 of a 20-slot wheel disc.  Re-measure with the
// live tune (enc_um_per_edge) if wheels or motors change.
#define ENC_EDGES_PER_REV           1150u
#define ENC_UM_PER_EDGE             177u

// Distance between the two wheel contact points.
#define TRACK_WIDTH_MM              120u

// How far the robot backs up before retrying (REVERSE_RETRY).
#define REVERSE_DISTANCE_MM         100u

/* ------------------------------------------------------------------------ */
/* Task periods.  The kernel tick is CNF_TIMER_PERIOD = 10 ms.             */
/* ------------------------------------------------------------------------ */

#define PERIOD_BASE_MS              10u
#define PERIOD_ENCODER_TICKS        1u
#define PERIOD_LINE_TICKS           1u
#define PERIOD_MOTION_TICKS         1u
#define PERIOD_IMU_TICKS            2u
#define PERIOD_TELEMETRY_TICKS      20u
#define HEARTBEAT_EVERY_FRAMES      5u

/* ------------------------------------------------------------------------ */
/* Task priorities (1 is the highest).  The port's USB-CDC service runs at */
/* 3 and the CYW43 radio-owner task (Buddy 1 network poll) runs at 4, so   */
/* every control and sensing task sits above them (Week 6, NFR17).         */
/* ------------------------------------------------------------------------ */

#define PRIO_VEHICLE                1
#define PRIO_BARCODE                1
#define PRIO_ENCODER                2
#define PRIO_LINE                   2
#define PRIO_MOTION                 2
#define PRIO_OBSTACLE               2
#define PRIO_IMU                    5
#define PRIO_TELEMETRY              8

// Stack sizes in bytes.  Single-core build; trimmed after measurement.
#define STACK_SMALL                 1024u
#define STACK_MEDIUM                2048u

/* ------------------------------------------------------------------------ */
/* Buddy 2: motion control tuning (fixed point, TBC on the chassis).       */
/* ------------------------------------------------------------------------ */

#define MOTOR_PWM_HZ                10000u
#define MOTOR_LEFT_REVERSED         1u
#define MOTOR_RIGHT_REVERSED        0u
#define MOTOR_MIN_PERMILLE          250u
// Speed limit: the most PWM either motor is ever given, in permille of
// full power (1000).  Lower it to make the robot slower; keep it above
// MOTOR_MIN_PERMILLE or the wheels may not turn at all.
#define MOTOR_MAX_PERMILLE          450u
#define MOTOR_FF_PERMILLE_PER_M_S   1300u
#define SPEED_FOLLOW_MM_S           130u    // Tuned live 2026-11-02.
#define SPEED_SLOW_MM_S             100u
#define SPEED_TURN_MM_S             100u
#define SPEED_MAX_MM_S              600u
#define PID_KP_MILLI                1200
#define PID_KI_MILLI                400
#define PID_KD_MILLI                50
#define STEER_KP_MILLI              65      // Tuned live 2026-11-02.
#define STEER_KD_MILLI              110     // Tuned live 2026-11-02.
// Steering sign: 1 or -1.  Flip it if the robot steers away from the line
// (which edge it follows depends on where the centre sensor is mounted).
#define STEER_DIRECTION             1
#define STALL_TIMEOUT_MS            300u
#define ENC_EDGE_TIMEOUT_MS         150u
// Encoder noise filter: edges closer than this are ignored.  At the top
// speed (600 mm/s) real edges are 8.5 ms apart, so 5 ms only drops noise,
// such as bursts picked up from the motor wires.
// Edges closer than this are noise.  At 0.18 mm per edge the wheel gives an
// edge every 1.2 ms at line-following speed, so this must stay well below.
#define ENC_MIN_EDGE_US             200u
#define STOPPED_SPEED_MM_S          20u
#define LINE_STALE_MS               50u

/* ------------------------------------------------------------------------ */
/* Buddy 3: line and barcode tuning.                                       */
/* ------------------------------------------------------------------------ */

#define LINE_MIN_CONTRAST           400u
#define LINE_JUNCTION_MS            20u
#define LINE_LOST_MS                150u
#define LINE_FOUND_MS               30u
// Longest gap allowed between edges before a capture ends.  It must exceed
// the widest element at the slowest reading speed: 400 ms lets a 17 mm bar
// pass at 45 mm/s (about a third of line-following speed), and once some
// elements are in the gap grows with them (see barcode.c).
#define BARCODE_QUIET_MS            400u
#define BARCODE_FIFO_SLOTS          64u

// Analogue barcode sensing.  Both IR analogue inputs are converted in turn,
// IR_SAMPLE_HZ times a second each.  A bar needs the black-white contrast
// seen so far to be at least BARCODE_MIN_SPAN ADC counts.  It is noticed when
// the reading rises above the white level by that contrast divided by
// 2^BARCODE_ENTER_SHIFT and ends when the reading falls below half of the
// bar's own height.  Both of its edges are then placed at that half-height
// level (the start is found in the last BARCODE_HISTORY samples), so a bar
// narrower than the sensor's spot, which never reaches full black, still
// measures at its true width.  The contrast envelope decays by
// 1/2^BARCODE_DECAY_SHIFT per sample (about 0.5 s).
#define IR_SAMPLE_HZ                2000u
#ifndef BARCODE_MIN_SPAN
#define BARCODE_MIN_SPAN            300u
#endif
#ifndef BARCODE_ENTER_SHIFT
#define BARCODE_ENTER_SHIFT         3u
#endif
#define BARCODE_DECAY_SHIFT         10u
#define BARCODE_HISTORY             1024u
// Readings pass through a 3-sample median, which removes single-sample
// spikes, then a BARCODE_AVERAGE-sample running average (16 samples = 8 ms,
// 4 ms of delay; a narrow bar lasts 37 ms at line-following speed).  A bar
// is only accepted if it lasted at least BARCODE_MIN_BAR_MS and reached a
// quarter of the contrast above white.  Overridable for host tests.
#ifndef BARCODE_AVERAGE
#define BARCODE_AVERAGE             16u
#endif
#ifndef BARCODE_MIN_BAR_MS
#define BARCODE_MIN_BAR_MS          8u
#endif
#ifndef BARCODE_ACCEPT_SHIFT
#define BARCODE_ACCEPT_SHIFT        2u
#endif

/* ------------------------------------------------------------------------ */
/* Buddy 4: IMU tuning.                                                    */
/* ------------------------------------------------------------------------ */

#define IMU_I2C_HZ                  400000u
#define IMU_FORWARD_AXIS            0u
#define IMU_FORWARD_SIGN            1
#define IMU_CALIB_SAMPLES           50u
#define IMU_FAULT_ERRORS            3u
#define TURNING_MDEG_S              30000
#define ACCEL_EVENT_MM_S2           300
#define HUMP_ENTER_MDEG             6000
#define HUMP_EXIT_MDEG              3000
#define HUMP_HOLD_MS                100u
#define IMPACT_MG                   1500u
#define IMPACT_COOLDOWN_MS          500u

/* ------------------------------------------------------------------------ */
/* Buddy 5: obstacle tuning.                                               */
/* ------------------------------------------------------------------------ */

#define OBSTACLE_DETECT_MM          250u
#define OBSTACLE_RELEASE_MM         350u
#define MONITOR_PERIOD_MS           60u
#define SONAR_TIMEOUT_MS            40u
#define SERVO_MIN_US                500u
#define SERVO_MAX_US                2500u
#define SERVO_LEFT_IS_HIGH          1u
#define SCAN_FINE_STEP_DEG          5u
// Settling time per fine-scan step.  Was 60 ms: too fast for the servo
// plus the echo of the previous ping to die away, so readings were noisy.
#define SCAN_FINE_SETTLE_MS         100u
#define CLEARANCE_MAX_MM            1000u
#define OBSTACLE_PROFILE_MM         600u
#define SONAR_MAX_MM                2000u
#define SERVO_SETTLE_MS             150u
#define SERVO_CENTRE_DEG            90u
#define VEHICLE_WIDTH_MM            150u
#define CLEARANCE_MARGIN_MM         60u
// Half-angle of the HC-SR04 beam that still returns an echo from a box.
// The sensor reports the obstacle while any part of the beam touches it, so
// a raw scan makes it look d * sin(this) wider on each side; the profile
// takes that off.  TBC: scan a box of known width and adjust.
#define SONAR_BEAM_HALF_DEG         10u
#define OBSTACLE_MIN_WIDTH_MM       30u
// Sideways clearance kept from the obstacle edge while passing it (the
// side-choice check above still uses CLEARANCE_MARGIN_MM), and the limits on
// the sideways move, so a bad scan cannot send the robot off the course.
#define BYPASS_MARGIN_MM            40u
#define BYPASS_LATERAL_MIN_MM       80u
#define BYPASS_LATERAL_MAX_MM       250u
// The detour turn is chosen from the sideways move and the distance to the
// obstacle (atan(sideways / ahead)), within these limits.  Was a fixed 45.
#define BYPASS_ANGLE_MIN_DEG        25
#define BYPASS_ANGLE_MAX_DEG        60

/* ------------------------------------------------------------------------ */
/* Integration: Vehicle Control.                                           */
/* ------------------------------------------------------------------------ */

#define REACQUIRE_TIMEOUT_MS        5000u
#define SEQUENCE_TIMEOUT_MS         15000u
#define PROFILE_TIMEOUT_MS          10000u
#define INIT_SETTLE_MS              1500u
#define OBSTACLE_DEPTH_EST_MM       150u
#define VEHICLE_LENGTH_MM           180u
#define BYPASS_RETRIES_MAX          2u
#define JUNCTION_ADVANCE_MM         60u

/* ------------------------------------------------------------------------ */
/* Buddy 1: MQTT.  MQTT_BROKER_IP is in config/wifi_credentials.h.         */
/* ------------------------------------------------------------------------ */

#define MQTT_BROKER_PORT            1883u
#define MQTT_CLIENT_ID              "aai03-robot"
#define MQTT_TOPIC_ROOT             "inf2004/aai03/"
#define MQTT_KEEPALIVE_S            10u

#endif /* ROBOT_CFG_H */

/*** end of file ***/


/** @file test_main.c
 *
 * @brief Host unit tests for the hardware-independent robot modules.
 *
 * @par
 * Build and run on a PC (see tests/Makefile):  make -C tests
 * The modules under test are compiled unchanged from app_program/.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "bitpack.h"
#include "fix_math.h"
#include "code39.h"
#include "hump.h"
#include "obs_profile.h"
#include "tlm_codec.h"
#include "tune.h"
#include "nav_map.h"
#include "pid_ctrl.h"
#include "sys_status.h"
#include "robot_cfg.h"

#define NARROW_TICKS            40u
#define WIDE_TICKS              120u
#define FRAME_CHARS             3u
#define LETTERS                 26u
#define STRAY_ELEMENTS          3u
#define BLUR_WIDE_TICKS         90u     // Printed wide, blur test.
#define BLUR_TICKS              28u     // Bars gain, spaces lose this.

static uint32_t g_checks   = 0u;
static uint32_t g_failures = 0u;

static void check(bool b_ok, char const * p_what, int32_t line);
static bool near(int32_t value, int32_t expected, int32_t tolerance);
static uint32_t build_frame(uint8_t data_symbol, uint32_t * p_widths);
static void test_bitpack(void);
static void test_fix_math(void);
static void test_code39(void);
static void test_hump(void);
static void test_obs_profile(void);
static void test_tlm_codec(void);
static void test_nav_map(void);
static void test_pid(void);

#define CHECK(COND)             check((COND), #COND, __LINE__)


/*!
 * @brief Run every test and report.
 *
 * @return 0 if every check passed, 1 otherwise.
 */
int
main (void)
{
    test_bitpack();
    test_fix_math();
    test_code39();
    test_hump();
    test_obs_profile();
    test_tlm_codec();
    test_nav_map();
    test_pid();

    printf("%lu checks, %lu failures\n", (unsigned long) g_checks,
           (unsigned long) g_failures);

    return ((0u == g_failures) ? 0 : 1);
}


/*!
 * @brief Record one check.
 *
 * @param[in] b_ok    Outcome.
 * @param[in] p_what  Expression text.
 * @param[in] line    Source line.
 */
static void
check (bool b_ok, char const * p_what, int32_t line)
{
    g_checks++;

    if (!b_ok)
    {
        g_failures++;
        printf("FAIL line %ld: %s\n", (long) line, p_what);
    }
}


/*!
 * @brief Absolute-tolerance comparison.
 *
 * @param[in] value      Measured value.
 * @param[in] expected   Expected value.
 * @param[in] tolerance  Allowed difference.
 *
 * @return true if within tolerance.
 */
static bool
near (int32_t value, int32_t expected, int32_t tolerance)
{
    return (fix_math_abs(value - expected) <= tolerance);
}


/*!
 * @brief Element widths of "*X*" for a data symbol, with small jitter.
 *
 * @param[in]  data_symbol  ASCII letter.
 * @param[out] p_widths     29 widths.
 *
 * @return Number of elements written.
 */
static uint32_t
build_frame (uint8_t data_symbol, uint32_t * p_widths)
{
    uint8_t const symbols[FRAME_CHARS] = { '*', data_symbol, '*' };
    uint32_t      count = 0u;

    for (uint32_t chr = 0u; chr < FRAME_CHARS; chr++)
    {
        uint32_t pattern = 0u;

        for (uint32_t candidate = 0u; candidate < 512u; candidate++)
        {
            if (symbols[chr] == code39_symbol_for(candidate))
            {
                pattern = candidate;
            }
        }

        for (uint32_t idx = 0u; idx < CODE39_ELEMENTS_PER_CHAR; idx++)
        {
            bool const     b_wide = (0u != ((pattern >> (8u - idx)) & 1u));
            uint32_t const jitter = (count % 3u) * 3u;

            p_widths[count] = (b_wide ? WIDE_TICKS : NARROW_TICKS) + jitter;
            count++;
        }

        if ((chr + 1u) < FRAME_CHARS)
        {
            p_widths[count] = NARROW_TICKS;
            count++;
        }
    }

    return (count);
}


/*!
 * @brief bitpack: word fields, excess-K, saturation and bit streams.
 */
static void
test_bitpack (void)
{
    uint8_t   buf[8];
    bitpack_t pack;
    uint32_t  word = 0u;

    word = bitpack_word_set(word, 3u, 5u, 0x1Fu);
    CHECK(0xF8u == word);
    CHECK(0x1Fu == bitpack_word_get(word, 3u, 5u));
    word = bitpack_word_set(word, 4u, 2u, 0u);
    CHECK(0x1Fu != bitpack_word_get(word, 3u, 5u));
    CHECK(1024u == bitpack_from_signed(0, 11u));
    CHECK(0u == bitpack_from_signed(-5000, 11u));
    CHECK(2047u == bitpack_from_signed(5000, 11u));
    CHECK(-300 == bitpack_to_signed(bitpack_from_signed(-300, 11u), 11u));
    CHECK(255u == bitpack_saturate(1000u, 8u));
    CHECK(0xFFFFFFFFu == bitpack_word_get(0xFFFFFFFFu, 0u, 32u));

    bitpack_init(&pack, buf, sizeof(buf));
    bitpack_put(&pack, 0x5u, 3u);
    bitpack_put(&pack, 0x1ABu, 9u);
    bitpack_put(&pack, 0xDEADBEEFu, 32u);
    CHECK(6u == bitpack_bytes_used(&pack));
    bitpack_put(&pack, 0xFFFFu, 32u);
    CHECK(bitpack_is_overflow(&pack));

    bitpack_init(&pack, buf, sizeof(buf));
    bitpack_put(&pack, 0x5u, 3u);
    bitpack_put(&pack, 0x1ABu, 9u);
    bitpack_put(&pack, 0xDEADBEEFu, 32u);
    pack.bit_pos = 0u;
    CHECK(0x5u == bitpack_get(&pack, 3u));
    CHECK(0x1ABu == bitpack_get(&pack, 9u));
    CHECK(0xDEADBEEFu == bitpack_get(&pack, 32u));
}


/*!
 * @brief fix_math: square root and trigonometry accuracy.
 */
static void
test_fix_math (void)
{
    CHECK(0u == fix_math_isqrt(0u));
    CHECK(1000u == fix_math_isqrt(1000000u));
    CHECK(65535u == fix_math_isqrt(0xFFFFFFFFu));
    CHECK(near(fix_math_sin_q15(30000), 16384, 40));
    CHECK(near(fix_math_sin_q15(90000), 32768, 1));
    CHECK(near(fix_math_sin_q15(-90000), -32768, 1));
    CHECK(near(fix_math_sin_q15(210000), -16384, 40));
    CHECK(near(fix_math_cos_q15(60000), 16384, 40));
    CHECK(near(fix_math_cos_q15(180000), -32768, 1));
    CHECK(near(fix_math_atan2_mdeg(1000, 1000), 45000, 300));
    CHECK(near(fix_math_atan2_mdeg(1000, -1000), 135000, 300));
    CHECK(near(fix_math_atan2_mdeg(-1000, -1000), -135000, 300));
    CHECK(near(fix_math_atan2_mdeg(500, 866), 30000, 300));
    CHECK(near(fix_math_atan2_mdeg(0, -1000), 180000, 300));
    CHECK(near(fix_math_asin_mdeg(16384), 30000, 300));
    CHECK(near(fix_math_asin_mdeg(-32768), -90000, 300));
    CHECK(-170000 == fix_math_wrap_mdeg(190000));
    CHECK(near(fix_math_abs(INT32_MIN), INT32_MAX, 0));
}


/*!
 * @brief code39: every letter, both directions, and rejections.
 */
static void
test_code39 (void)
{
    uint32_t widths[CODE39_FRAME_ELEMENTS];
    uint32_t reversed[CODE39_FRAME_ELEMENTS];
    uint8_t  symbol     = 0u;
    bool     b_reversed = false;
    uint32_t passed     = 0u;

    for (uint32_t letter = 0u; letter < LETTERS; letter++)
    {
        uint32_t const count = build_frame((uint8_t) ('A' + letter), widths);

        for (uint32_t idx = 0u; idx < count; idx++)
        {
            reversed[idx] = widths[(count - 1u) - idx];
        }

        if ((CODE39_OK == code39_decode(widths, count, &symbol,
                                        &b_reversed)) &&
            (('A' + letter) == symbol) && (!b_reversed) &&
            (CODE39_OK == code39_decode(reversed, count, &symbol,
                                        &b_reversed)) &&
            (('A' + letter) == symbol) && b_reversed)
        {
            passed++;
        }
    }

    CHECK(LETTERS == passed);
    CHECK(CODE39_BAD_COUNT == code39_decode(widths, 28u, &symbol,
                                            &b_reversed));

    (void) build_frame('B', widths);
    widths[3] = NARROW_TICKS + 30u;    // A narrow element nearly wide.
    widths[1] = WIDE_TICKS - 50u;      // And a wide one nearly narrow.
    CHECK(CODE39_OK != code39_decode(widths, CODE39_FRAME_ELEMENTS, &symbol,
                                     &b_reversed));

    (void) build_frame('1', widths);
    CHECK(CODE39_BAD_FRAME == code39_decode(widths, CODE39_FRAME_ELEMENTS,
                                            &symbol, &b_reversed));

    // Stray edges around the frame (paper edges): every letter, both ways.
    passed = 0u;

    for (uint32_t letter = 0u; letter < LETTERS; letter++)
    {
        uint32_t padded[CODE39_FRAME_ELEMENTS + STRAY_ELEMENTS];

        (void) build_frame((uint8_t) ('A' + letter), widths);
        padded[0] = WIDE_TICKS * 5u;
        padded[CODE39_FRAME_ELEMENTS + 1u] = WIDE_TICKS * 4u;
        padded[CODE39_FRAME_ELEMENTS + 2u] = NARROW_TICKS * 7u;

        for (uint32_t idx = 0u; idx < CODE39_FRAME_ELEMENTS; idx++)
        {
            padded[idx + 1u] = widths[idx];
        }

        if ((CODE39_OK == code39_decode(padded,
                                        CODE39_FRAME_ELEMENTS + STRAY_ELEMENTS,
                                        &symbol, &b_reversed)) &&
            (('A' + letter) == symbol) && (!b_reversed))
        {
            passed++;
        }

        for (uint32_t idx = 0u; idx < CODE39_FRAME_ELEMENTS; idx++)
        {
            padded[idx + 1u] = widths[(CODE39_FRAME_ELEMENTS - 1u) - idx];
        }

        if ((CODE39_OK == code39_decode(padded,
                                        CODE39_FRAME_ELEMENTS + STRAY_ELEMENTS,
                                        &symbol, &b_reversed)) &&
            (('A' + letter) == symbol) && b_reversed)
        {
            passed++;
        }
    }

    CHECK((2u * LETTERS) == passed);

    // IR blur: every bar reads wider and every space narrower than printed,
    // so a narrow bar can end up wider than a wide space.  Printed 40/90,
    // blurred by 28: narrow bar 68 > wide space 62.  Bars and spaces are
    // classed separately, so every letter still reads, both ways.
    passed = 0u;

    for (uint32_t letter = 0u; letter < LETTERS; letter++)
    {
        uint32_t const count = build_frame((uint8_t) ('A' + letter), widths);

        for (uint32_t idx = 0u; idx < count; idx++)
        {
            bool const     b_wide = (widths[idx] >= WIDE_TICKS);
            bool const     b_bar  = (0u == (idx % 2u));  // Bar first.
            uint32_t const print  = b_wide ? BLUR_WIDE_TICKS : NARROW_TICKS;

            widths[idx] = b_bar ? (print + BLUR_TICKS) : (print - BLUR_TICKS);
        }

        for (uint32_t idx = 0u; idx < count; idx++)
        {
            reversed[idx] = widths[(count - 1u) - idx];
        }

        if ((CODE39_OK == code39_decode(widths, count, &symbol,
                                        &b_reversed)) &&
            (('A' + letter) == symbol) && (!b_reversed) &&
            (CODE39_OK == code39_decode(reversed, count, &symbol,
                                        &b_reversed)) &&
            (('A' + letter) == symbol) && b_reversed)
        {
            passed++;
        }
    }

    CHECK(LETTERS == passed);
}


/*!
 * @brief hump: a 10-degree ramp up and down reports about 17 mm.
 */
static void
test_hump (void)
{
    hump_t   hump;
    int32_t  distance = 0;
    uint32_t height   = 0u;
    bool     b_done   = false;

    hump_init(&hump, HUMP_ENTER_MDEG, HUMP_EXIT_MDEG, HUMP_HOLD_MS);

    // Level, then 100 mm climbing at 10 degrees, 100 mm descending, level.
    for (uint32_t step = 0u; step < 60u; step++)
    {
        int32_t pitch = 0;

        if ((step >= 10u) && (step < 30u))
        {
            pitch = 10000;
        }
        else if ((step >= 30u) && (step < 50u))
        {
            pitch = -10000;
        }
        else
        {
            pitch = 0;
        }

        distance += 5;

        if (hump_update(&hump, pitch, distance, true, 20u, &height))
        {
            b_done = true;
        }
    }

    CHECK(b_done);
    CHECK(near((int32_t) height, 17, 2));
    CHECK(near((int32_t) hump_max_mm(&hump), 17, 2));
    CHECK(1u == hump_count(&hump));

    // An invalid sample in the middle abandons the hump.
    hump_init(&hump, HUMP_ENTER_MDEG, HUMP_EXIT_MDEG, HUMP_HOLD_MS);
    b_done = false;

    for (uint32_t step = 0u; step < 60u; step++)
    {
        int32_t const pitch = ((step >= 10u) && (step < 30u)) ? 10000 :
                              (((step >= 30u) && (step < 50u)) ? -10000 : 0);

        distance += 5;

        if (hump_update(&hump, pitch, distance, (25u != step), 20u, &height))
        {
            b_done = true;
        }
    }

    CHECK(!b_done);
    CHECK(0u == hump_max_mm(&hump));
}


/*!
 * @brief obs_profile: plans for clear, one-side and blocked scenes.
 */
static void
test_obs_profile (void)
{
    obs_profile_point_t   points[OBS_PROFILE_MAX_POINTS];
    obs_profile_t profile;
    obs_profile_t decoded;
    uint8_t       msg[OBS_PROFILE_MSG_BYTES];
    uint32_t      count = 0u;

    // Scene 1: nothing within range.
    for (uint32_t angle = 30u; angle <= 150u; angle += 30u)
    {
        points[count].angle_deg   = angle;
        points[count].distance_mm = 1500u;
        points[count].b_echo      = true;
        count++;
    }

    obs_profile_compute(points, count, &profile);
    CHECK(OBS_PLAN_CONTINUE == profile.plan);

    // Scene 2: a 100 mm box 250 mm ahead, open on both sides.  The box
    // spans 80..100 degrees; the 10-degree beam half-angle makes it echo
    // from 70 to 110, and the profile must take that spread off again.
    count = 0u;

    for (uint32_t angle = 30u; angle <= 150u; angle += 5u)
    {
        points[count].angle_deg   = angle;
        points[count].distance_mm = ((angle >= 70u) && (angle <= 110u)) ?
                                    250u : 1500u;
        points[count].b_echo      = ((angle >= 70u) && (angle <= 110u));
        count++;
    }

    obs_profile_compute(points, count, &profile);
    CHECK((OBS_PLAN_PASS_LEFT == profile.plan) ||
          (OBS_PLAN_PASS_RIGHT == profile.plan));
    CHECK(near((int32_t) profile.width_mm, 87, 30));

    // Sideways move: half the box (about 44) + half the vehicle (75) +
    // BYPASS_MARGIN_MM (40), not the raw-width figure of about 250.
    CHECK(near((int32_t) profile.lateral_mm, 159, 30));

    obs_profile_pack(&profile, msg);
    obs_profile_unpack(msg, &decoded);
    CHECK(decoded.plan == profile.plan);
    CHECK((profile.distance_mm / 10u) == (decoded.distance_mm / 10u));
    CHECK((profile.lateral_mm / 10u) == (decoded.lateral_mm / 10u));

    // A box well to the right of the line needs a smaller move to pass it
    // on the left than its width suggests.
    for (uint32_t idx = 0u; idx < count; idx++)
    {
        uint32_t const angle = points[idx].angle_deg;

        points[idx].distance_mm = ((angle >= 55u) && (angle <= 95u)) ?
                                  250u : 1500u;
        points[idx].b_echo      = ((angle >= 55u) && (angle <= 95u));
    }

    obs_profile_compute(points, count, &profile);
    CHECK(OBS_PLAN_PASS_LEFT == profile.plan);
    CHECK(profile.lateral_mm < 140u);

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        uint32_t const angle = points[idx].angle_deg;

        points[idx].distance_mm = ((angle >= 70u) && (angle <= 110u)) ?
                                  250u : 1500u;
        points[idx].b_echo      = ((angle >= 70u) && (angle <= 110u));
    }

    // Scene 3: the same box with a wall close on the left: pass right.
    for (uint32_t idx = 0u; idx < count; idx++)
    {
        if (points[idx].angle_deg >= 120u)
        {
            points[idx].distance_mm = 260u;
            points[idx].b_echo      = true;
        }
    }

    obs_profile_compute(points, count, &profile);
    CHECK(OBS_PLAN_PASS_RIGHT == profile.plan);

    // Scene 4: walls on both sides: reverse and retry.
    for (uint32_t idx = 0u; idx < count; idx++)
    {
        if (points[idx].angle_deg <= 60u)
        {
            points[idx].distance_mm = 260u;
            points[idx].b_echo      = true;
        }
    }

    obs_profile_compute(points, count, &profile);
    CHECK(OBS_PLAN_REVERSE == profile.plan);

    // No usable samples at all: stop.
    obs_profile_compute(points, 0u, &profile);
    CHECK(OBS_PLAN_STOP == profile.plan);
}


/*!
 * @brief tlm_codec: frame sizes and command validation.
 */
static void
test_tlm_codec (void)
{
    uint32_t      words[SYS_STATUS_WORD_COUNT];
    uint8_t       frame[TLM_CODEC_MAX_FRAME];
    uint32_t      word = 0u;
    tlm_codec_summary_t summary = { 17u, 2u, 5u, 1u, 1u, 1234u, 95u, 0u };
    uint8_t const start[]   = { 0x18u, 0x00u };
    uint8_t const map_set[] = { 0x18u, 0x34u, 0x22u };
    uint8_t const bad_ver[] = { 0x28u, 0x00u };
    uint8_t const bad_map[] = { 0x18u, 0x04u, 0x1Fu };
    uint8_t const tune_edge[] = { 0x18u, 0x75u, 0xF1u, 0x13u };
    uint8_t const tune_dir[]  = { 0x18u, 0x65u, 0xFFu, 0xFFu };

    for (uint32_t idx = 0u; idx < (uint32_t) SYS_STATUS_WORD_COUNT; idx++)
    {
        words[idx] = 0xFFFFFFFFu;
    }

    CHECK(288u == (16u + sys_status_total_bits()));
    CHECK(36u == tlm_codec_telemetry(7u, words, frame, sizeof(frame)));
    CHECK(0x10u == frame[0]);
    CHECK(7u == frame[1]);
    CHECK(6u == tlm_codec_heartbeat(1u, 60u, 3u, 0u, frame, sizeof(frame)));
    CHECK(6u == tlm_codec_event(2u, 0x12345678u, frame, sizeof(frame)));
    CHECK(11u == tlm_codec_summary(&summary, frame, sizeof(frame)));
    CHECK(2u == tlm_codec_ack(4u, true, frame, sizeof(frame)));
    CHECK(0u == tlm_codec_telemetry(0u, words, frame, 10u));

    CHECK(TLM_CMD_START == tlm_codec_parse_command(start, 2u, &word));
    CHECK(TLM_CMD_MAP_SET == tlm_codec_parse_command(map_set, 3u, &word));
    CHECK(2u == bitpack_word_get(word, TLM_CMD_LETTER_SHIFT,
                                 TLM_CMD_LETTER_WIDTH));
    CHECK(1u == bitpack_word_get(word, TLM_CMD_ACTION_SHIFT,
                                 TLM_CMD_ACTION_WIDTH));
    CHECK(3u == bitpack_word_get(word, TLM_CMD_VERSION_SHIFT,
                                 TLM_CMD_VERSION_WIDTH));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(bad_ver, 2u, &word));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(bad_map, 3u, &word));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(start, 1u, &word));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(map_set, 2u, &word));

    // TUNE: setting 7 (encoder edge) = 5105, and setting 6 (steering) = -1.
    CHECK(TLM_CMD_TUNE == tlm_codec_parse_command(tune_edge, 4u, &word));
    CHECK(7u == bitpack_word_get(word, TLM_CMD_PARAM_SHIFT,
                                 TLM_CMD_PARAM_WIDTH));
    CHECK(5105 == bitpack_to_signed(bitpack_word_get(word,
                                        TLM_CMD_VALUE_SHIFT,
                                        TLM_CMD_VALUE_WIDTH),
                                    TLM_CMD_VALUE_WIDTH));
    CHECK(TLM_CMD_TUNE == tlm_codec_parse_command(tune_dir, 4u, &word));
    CHECK(-1 == bitpack_to_signed(bitpack_word_get(word,
                                      TLM_CMD_VALUE_SHIFT,
                                      TLM_CMD_VALUE_WIDTH),
                                  TLM_CMD_VALUE_WIDTH));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(tune_edge, 3u, &word));

    // Settings accept only in-range values.
    CHECK(tune_set(TUNE_MOTOR_MAX, 350));
    CHECK(350 == tune_get(TUNE_MOTOR_MAX));
    CHECK(!tune_set(TUNE_MOTOR_MAX, 100));
    CHECK(350 == tune_get(TUNE_MOTOR_MAX));
    CHECK(!tune_set(TUNE_STEER_DIR, 0));
    CHECK(tune_set(TUNE_STEER_DIR, -1));
    CHECK(!tune_set(TUNE_COUNT, 1));
}


/*!
 * @brief nav_map: default actions and updates.
 */
static void
test_nav_map (void)
{
    nav_map_t map;

    nav_map_init(&map);
    CHECK(NAV_ACTION_LEFT == nav_map_get(&map, 0u));
    CHECK(NAV_ACTION_RIGHT == nav_map_get(&map, 1u));
    CHECK(NAV_ACTION_STRAIGHT == nav_map_get(&map, 2u));
    CHECK(NAV_ACTION_UTURN == nav_map_get(&map, 3u));
    CHECK(NAV_ACTION_STRAIGHT == nav_map_get(&map, 25u));
    CHECK(nav_map_set(&map, 25u, NAV_ACTION_UTURN, 7u));
    CHECK(NAV_ACTION_UTURN == nav_map_get(&map, 25u));
    CHECK(7u == nav_map_version(&map));
    CHECK(!nav_map_set(&map, 26u, NAV_ACTION_LEFT, 8u));
    CHECK(7u == nav_map_version(&map));
    CHECK(NAV_ACTION_LEFT == nav_map_get(&map, 0u));
}


/*!
 * @brief pid_ctrl: sign, clamping and anti-windup.
 */
static void
test_pid (void)
{
    pid_ctrl_t pid;
    int32_t    output = 0;

    pid_ctrl_init(&pid, 1000, 500, 0, 300);
    CHECK(pid_ctrl_update(&pid, 100, 0) > 0);
    CHECK(pid_ctrl_update(&pid, 0, 100) < 0);

    for (uint32_t step = 0u; step < 100u; step++)
    {
        output = pid_ctrl_update(&pid, 1000, 0);
    }

    CHECK(300 == output);
    CHECK(pid.integral <= pid.integral_limit);

    // After saturation, a reversed error must pull the output back quickly.
    output = pid_ctrl_update(&pid, 0, 50);
    CHECK(output < 300);
}

/*** end of file ***/


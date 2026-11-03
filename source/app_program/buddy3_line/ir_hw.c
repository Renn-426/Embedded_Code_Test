/** @file ir_hw.c
 *
 * @brief TCRT5000 IR sensor inputs on the RP2040 ADC and GPIO.
 *
 * @par
 * Reference: RP2040 Datasheet section 4.9 (ADC); Pico SDK hardware_adc.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "robot_cfg.h"
#include "ir_hw.h"

#define ADC_CLOCK_HZ            48000000u
#define INPUTS_SAMPLED          2u          // Centre and barcode.
#define FIFO_PAIR               2u          // Interrupt once both are in.
#define SAMPLE_MASK             0x0FFFu     // 12-bit conversion result.

#if BARCODE_SENSOR_ANALOGUE
_Static_assert(IR_HW_ADC_IRQ_NUM == ADC_IRQ_FIFO,
               "IR_HW_ADC_IRQ_NUM must match the SDK ADC_IRQ_FIFO");

// Latest conversions, written by the ADC interrupt (single aligned stores).
static uint32_t volatile g_centre_raw  = 0u;
static uint32_t volatile g_barcode_raw = 0u;

// Diagnostics: interrupts served and FIFO overruns seen.
static uint32_t volatile g_take_calls  = 0u;
static uint32_t volatile g_overruns    = 0u;

static uint32_t other_input(uint32_t input);
#endif

static bool level_means_black(bool b_level);


/*!
 * @brief Configure the ADC input and the two digital IR inputs.
 */
void
ir_hw_init (void)
{
    adc_init();
    adc_gpio_init(PIN_IR_CENTRE_AO);
    adc_select_input(ADC_INPUT_IR_CENTRE);

    // The modules drive DO actively, so no pad pull is needed.
    gpio_init(PIN_IR_LEFT_DO);
    gpio_set_dir(PIN_IR_LEFT_DO, GPIO_IN);
    gpio_disable_pulls(PIN_IR_LEFT_DO);

#if BARCODE_SENSOR_ANALOGUE
    adc_gpio_init(PIN_IR_BARCODE_AO);
    adc_set_round_robin((1u << ADC_INPUT_IR_CENTRE) |
                        (1u << ADC_INPUT_IR_BARCODE));

    // One conversion every (DIV.INT + 1) ADC clocks, shared by both inputs.
    adc_hw->div = ((ADC_CLOCK_HZ / (IR_SAMPLE_HZ * INPUTS_SAMPLED)) - 1u)
                  << ADC_DIV_INT_LSB;
    adc_fifo_setup(true, false, FIFO_PAIR, false, false);
    adc_fifo_drain();
    adc_irq_set_enabled(true);  // The NVIC side is enabled by the kernel.
    adc_run(true);
#else
    gpio_init(PIN_IR_BARCODE_DO);
    gpio_set_dir(PIN_IR_BARCODE_DO, GPIO_IN);
    gpio_disable_pulls(PIN_IR_BARCODE_DO);
#endif
}


/*!
 * @brief One conversion of the centre sensor.
 *
 * @return Raw 12-bit reading, 0 to 4095.
 */
uint32_t
ir_hw_read_centre (void)
{
#if BARCODE_SENSOR_ANALOGUE
    return (g_centre_raw);
#else
    adc_select_input(ADC_INPUT_IR_CENTRE);

    return (adc_read());
#endif
}


/*!
 * @brief Report whether the left sensor is over black.
 *
 * @return true over the line.
 */
bool
ir_hw_is_left_black (void)
{
    return (level_means_black(gpio_get(PIN_IR_LEFT_DO)));
}


/*!
 * @brief Report whether the barcode sensor is over black.
 *
 * @return true over a bar.
 */
bool
ir_hw_is_barcode_black (void)
{
#if BARCODE_SENSOR_ANALOGUE
    // Rough indication for the bench display only: the Barcode task finds
    // the real edges from the contrast it has seen.
    bool const b_high = g_barcode_raw > (IR_HW_ADC_MAX / 2u);

    return (b_high == (0u != IR_AO_BLACK_IS_HIGH));
#else
    return (level_means_black(gpio_get(PIN_IR_BARCODE_DO)));
#endif
}


#if BARCODE_SENSOR_ANALOGUE
/*!
 * @brief Take the finished conversions out of the ADC FIFO.
 *
 * @param[out] p_barcode_raw  Latest barcode conversion, if any arrived.
 *
 * @return true if a barcode conversion was taken.
 *
 * @par
 * Runs in the ADC interrupt.  The FIFO does not say which input a result
 * came from, but round robin alternates the two inputs and CS.AINSEL names
 * the input of the conversion now in progress, so the entries are assigned
 * by walking back from it.  If results were lost (FIFO overrun) that walk
 * is no longer reliable, so the FIFO is emptied and the batch dropped.
 */
bool
ir_hw_sampler_take (uint32_t * p_barcode_raw)
{
    bool b_barcode = false;

    g_take_calls++;

    if (0u != (adc_hw->fcs & ADC_FCS_OVER_BITS))
    {
        g_overruns++;
        adc_fifo_drain();
        // OVER and UNDER clear when written with 1; the other fields are
        // written back unchanged.
        adc_hw->fcs = adc_hw->fcs | ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS;
    }
    else
    {
        uint32_t level = 0u;
        uint32_t again = 0u;
        uint32_t next  = 0u;
        uint32_t input = 0u;

        // A conversion may finish between the two reads: retry until the
        // level and the selected input belong to the same moment.
        level = adc_fifo_get_level();
        next  = (adc_hw->cs & ADC_CS_AINSEL_BITS) >> ADC_CS_AINSEL_LSB;
        again = adc_fifo_get_level();

        while (level != again)
        {
            level = again;
            next  = (adc_hw->cs & ADC_CS_AINSEL_BITS) >> ADC_CS_AINSEL_LSB;
            again = adc_fifo_get_level();
        }

        input = (0u == (level % INPUTS_SAMPLED)) ? next : other_input(next);

        for (; level > 0u; level--)
        {
            // Widening cast of the 16-bit FIFO entry; only 12 bits are data.
            uint32_t const value = (uint32_t) adc_fifo_get() & SAMPLE_MASK;

            if (ADC_INPUT_IR_CENTRE == input)
            {
                g_centre_raw = value;
            }
            else
            {
                g_barcode_raw  = value;
                *p_barcode_raw = value;
                b_barcode      = true;
            }

            input = other_input(input);
        }
    }

    return (b_barcode);
}


/*!
 * @brief Sampler diagnostics for the debug monitor.
 *
 * @param[out] p_calls     Interrupts served since boot.
 * @param[out] p_overruns  FIFO overruns seen since boot.
 */
void
ir_hw_sampler_stats (uint32_t * p_calls, uint32_t * p_overruns)
{
    *p_calls    = g_take_calls;
    *p_overruns = g_overruns;
}


/*!
 * @brief The other input of the round-robin pair.
 *
 * @param[in] input  ADC input number.
 *
 * @return The barcode input for the centre input, and vice versa.
 */
static uint32_t
other_input (uint32_t input)
{
    return ((ADC_INPUT_IR_CENTRE == input) ? ADC_INPUT_IR_BARCODE :
                                             ADC_INPUT_IR_CENTRE);
}
#else
/*!
 * @brief Digital barcode input: there are no conversions to take.
 *
 * @param[out] p_barcode_raw  Left unchanged.
 *
 * @return false.
 */
bool
ir_hw_sampler_take (uint32_t * p_barcode_raw)
{
    (void) p_barcode_raw;  // Unused: the digital output has no samples.

    return (false);
}


/*!
 * @brief Digital barcode input: no sampler, so no statistics.
 *
 * @param[out] p_calls     Set to 0.
 * @param[out] p_overruns  Set to 0.
 */
void
ir_hw_sampler_stats (uint32_t * p_calls, uint32_t * p_overruns)
{
    *p_calls    = 0u;
    *p_overruns = 0u;
}
#endif


/*!
 * @brief Translate a DO level into "black" using the configured polarity.
 *
 * @param[in] b_level  Pin level.
 *
 * @return true if the level means black.
 */
static bool
level_means_black (bool b_level)
{
    return (b_level == (0u != IR_DO_BLACK_IS_HIGH));
}

/*** end of file ***/


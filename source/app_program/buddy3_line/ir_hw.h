/** @file ir_hw.h
 *
 * @brief TCRT5000 IR sensor inputs: centre analogue (GP27, ADC1), left
 *        digital (GP3) and barcode analogue (GP28, ADC2) or digital (GP28).
 *
 * @par
 * Owner: Buddy 3.  With BARCODE_SENSOR_ANALOGUE the ADC free-runs over the
 * centre and barcode inputs and raises IR_HW_ADC_IRQ_NUM when a pair of
 * conversions is ready; ir_hw_sampler_take() is called from that interrupt
 * and ir_hw_read_centre() returns the latest centre conversion.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef IR_HW_H
#define IR_HW_H

#include <stdint.h>
#include <stdbool.h>

// Largest raw ADC reading (12-bit converter).
#define IR_HW_ADC_MAX               4095u

// RP2040 ADC FIFO interrupt (checked against the SDK in ir_hw.c).
#define IR_HW_ADC_IRQ_NUM           22u

void     ir_hw_init(void);
uint32_t ir_hw_read_centre(void);
bool     ir_hw_is_left_black(void);
bool     ir_hw_is_barcode_black(void);
bool     ir_hw_sampler_take(uint32_t * p_barcode_raw);
void     ir_hw_sampler_stats(uint32_t * p_calls, uint32_t * p_overruns);

#endif /* IR_HW_H */

/*** end of file ***/


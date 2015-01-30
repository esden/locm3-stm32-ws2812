/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

 /* This is the driver for controlling WS2812 RGB leds with built in pwm driver. */

#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/cortex.h>

#include "ws2812.h"

#define WS2812_BUFFER_SIZE 24*4
#define WS2812_CLOCK 800000
#define WS2812_T0H 0.32
#define WS2812_T1H 0.64
#define WS2812_TRESET 50e-6

extern uint32_t rcc_ppre1_frequency;

static volatile enum {
	ws2812_state_uninit = 0,
	ws2812_state_reset_blank,
	ws2812_state_idle,
	ws2812_state_sending
} ws2812_state = ws2812_state_uninit;

uint8_t ws2812_dma_buffer[(WS2812_BUFFER_SIZE)];
bool ws2812_buffer_half;
const uint8_t *ws2812_send_begin;
const uint8_t *ws2812_send_end;

void ws2812_timer_init(void);
uint32_t ws2812_timer_input_frequency(void);
uint8_t ws2812_timer_period(void);
uint16_t ws2812_timer_reset_period(void);
void ws2812_set_outputs_low(void);
void ws2812_timer_start_reset(void);
void ws2812_timer_start_main(void);
uint8_t ws2812_timer_one_high(void);
uint8_t ws2812_timer_zero_high(void);
void ws2812_dma_disable(void);
void ws2812_fill_next_buffer(uint8_t high, uint8_t low);
void ws2812_dma_init(void);
void ws2812_mode_initialize(const void *data, uint16_t length);
void ws2812_init(void);
void ws2812_send(const void *data, uint16_t length);
bool ws2812_busy(void);
bool ws2812_transmitting(void);

void ws2812_timer_init(void)
{
	/* Initialize peripheral clocks. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_TIM4);

 	/* Initialize Timer. */
	rcc_periph_reset_pulse(RCC_TIM4);
	TIM_CR1(TIM4) = 0;
	TIM_SR(TIM4) = 0;

	nvic_clear_pending_irq(NVIC_TIM4_IRQ);
	nvic_enable_irq(NVIC_TIM4_IRQ);
	nvic_set_priority(NVIC_TIM4_IRQ, 0);

	TIM_CR1(TIM4) = 0;
	TIM_PSC(TIM4) = 0;
}

uint32_t ws2812_timer_input_frequency(void)
{
	return rcc_ppre1_frequency;
}

uint8_t ws2812_timer_period(void)
{
	return (uint16_t)((ws2812_timer_input_frequency() + (uint32_t)((WS2812_CLOCK)/2)) /
		(uint32_t)(WS2812_CLOCK));
}

uint16_t ws2812_timer_reset_period(void)
{
	return (uint16_t)(ws2812_timer_input_frequency() /
		(uint32_t)(1.0 / (WS2812_TRESET) + 0.5)) + 2;
}

void ws2812_set_outputs_low(void)
{
	TIM_CCR1(TIM4) = 0;
}

void ws2812_timer_start_reset(void)
{
	ws2812_state = ws2812_state_reset_blank;
	TIM_CR1(TIM4) = 0;
	TIM_ARR(TIM4) = (uint16_t)ws2812_timer_reset_period() - 1;
    TIM_CNT(TIM4) = 0;
   	ws2812_set_outputs_low();
    TIM_EGR(TIM4) = TIM_EGR_UG;
    TIM_SR(TIM4) = 0;
    TIM_DIER(TIM4) = TIM_DIER_UIE;
    TIM_CR1(TIM4) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
        TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
}

void ws2812_timer_start_main(void)
{
	TIM_CR1(TIM4) = 0;
	TIM_ARR(TIM4) = (uint16_t)ws2812_timer_period() - 1;
    TIM_CNT(TIM4) = 0;
    TIM_SR(TIM4) = 0;
    TIM_CR1(TIM4) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
        TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
}

uint8_t ws2812_timer_zero_high(void)
{
	uint32_t high = (uint32_t)((WS2812_T0H) * (WS2812_CLOCK) + 0.5);
	return (uint8_t)((high * (uint32_t)ws2812_timer_period() + (uint32_t)((WS2812_CLOCK) / 2)) /
		(uint32_t)(WS2812_CLOCK));
}

uint8_t ws2812_timer_one_high(void)
{
	uint32_t high = (uint32_t)((WS2812_T1H) * (WS2812_CLOCK) + 0.5);
	return (uint8_t)((high * (uint32_t)ws2812_timer_period() + (uint32_t)((WS2812_CLOCK) / 2)) /
		(uint32_t)(WS2812_CLOCK));
}

void tim4_isr(void)
{
	TIM_CR1(TIM4) = 0;
	TIM_DIER(TIM4) = 0;
	TIM_SR(TIM4) = 0;
	ws2812_dma_disable();
	ws2812_state = ws2812_state_idle;
}

void ws2812_fill_next_buffer(uint8_t high, uint8_t low)
{
	uint8_t *target;
    if (ws2812_buffer_half) {
        target = &ws2812_dma_buffer[(WS2812_BUFFER_SIZE)/2];
    } else {
        target = &ws2812_dma_buffer[0];
    }
    ws2812_buffer_half = !ws2812_buffer_half;
    for (int total = 0; total < (WS2812_BUFFER_SIZE)/(8*2) &&
            ws2812_send_begin != ws2812_send_end; ++ws2812_send_begin, ++total) {
        uint8_t source = *ws2812_send_begin;
        for (int bit=0; bit<8; bit++, source <<= 1, ++target) {
            if (source & 0x80) {
                *target = high;
            } else {
                *target = low;
            }
        }
    }
}

void ws2812_dma_init(void)
{
	rcc_periph_clock_enable(RCC_DMA1);

	TIM_CCER(TIM4) = (TIM_CCER_CC1E);
	TIM_CCMR1(TIM4) = (TIM_CCMR1_OC1M_PWM1 |
					   TIM_CCMR1_OC1PE);

	//DMA_CCR(DMA1, DMA_CHANNEL7) = 0;
	//DMA_CNDTR(DMA1, DMA_CHANNEL7) = WS2812_BUFFER_SIZE;
	//DMA_CPAR(DMA1, DMA_CHANNEL7) = (uint32_t)(&TIM_CCR1(TIM4));
    //DMA_CMAR(DMA1, DMA_CHANNEL7) = (uint32_t)(&ws2812_dma_buffer[0]);
    //DMA_IFCR(DMA1) = DMA_IFCR_CGIF(DMA_CHANNEL7);

    //nvic_clear_pending_irq(NVIC_DMA1_CHANNEL7_IRQ);
    //nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
    //nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0);
}

void ws2812_dma_disable(void)
{
	//DMA_CCR(DMA1, DMA_CHANNEL7) = 0;
	ws2812_set_outputs_low();
}

void ws2812_mode_initialize(const void *data, uint16_t length)
{
	/* Initialize both buffers and start a send (the request won't
     * actually come in until the timer is started). */
    ws2812_send_begin = (const uint8_t *)data;
    ws2812_send_end = ws2812_send_begin + length;        
    ws2812_buffer_half = false;
    ws2812_fill_next_buffer(ws2812_timer_one_high(), ws2812_timer_zero_high());
    ws2812_fill_next_buffer(ws2812_timer_one_high(), ws2812_timer_zero_high());
    
    /* Enable the DMA request at the update event */
    /*DMA_CCR(DMA1, DMA_CHANNEL7) = (DMA_CCR_PL_HIGH |
    							   DMA_CCR_MSIZE_8BIT |
            					   DMA_CCR_PSIZE_16BIT |
            					   DMA_CCR_MINC |
            					   DMA_CCR_CIRC |
            					   DMA_CCR_DIR |
            					   DMA_CCR_TCIE |
            					   DMA_CCR_HTIE |
            					   DMA_CCR_EN);*/
    TIM_DIER(TIM4) = TIM_DIER_UDE;
    
    /* This means that output won't be updated until the second
     * clock cycle (since the DMA request won't come in until the
     * update at the end of the first, and the register itself won't
     * take the shadow value until the one after that).  This is fine,
     * however, as we just hold it low and this gives us plenty of
     * time to finish up handling. */
    TIM_CCR1(TIM4) = 0;
}

void dma1_stream6_isr(void)
{
	//DMA_IFCR(DMA1) = DMA_IFCR_CGIF(DMA_CHANNEL7);
        
    /* Not done yet, so fill the next buffer and wait for the send to
     * complete. */
    if (ws2812_send_begin != ws2812_send_end) {
            ws2812_fill_next_buffer(ws2812_timer_one_high(), ws2812_timer_zero_high());
        return;
    }

    /* Just clock out one extra buffer to make sure everything is
     * completely out before we start the reset sequence. */
    if (ws2812_send_begin != 0) {
        ws2812_send_begin = 0;
        ws2812_send_end = 0;
        return;
    }
    
    /* May already have clocked something out, but we don't care at this
     * point since it's past the end of the chain. */
    ws2812_dma_disable();
    ws2812_timer_start_reset();
}

void ws2812_init(void)
{
	/* disable interrupts */
	cm_disable_interrupts();

	ws2812_timer_init();
	ws2812_dma_init();
	ws2812_timer_start_reset();

	/* reenable interrupts */
	cm_enable_interrupts();
}

void ws2812_send(const void *data, uint16_t length)
{
	if (ws2812_state == ws2812_state_uninit){
		ws2812_init();
	}
	while (ws2812_busy()) { __asm__("nop"); }
	if (length == 0)
		return;

	cm_enable_interrupts();
	ws2812_state = ws2812_state_sending;
	ws2812_timer_init();
	ws2812_dma_init();
	ws2812_mode_initialize(data, length);
	ws2812_timer_start_main();
	cm_disable_interrupts();
}

bool ws2812_busy(void)
{
	return ws2812_state != ws2812_state_idle;
}

bool ws2812_transmitting(void)
{
	return ws2812_state == ws2812_state_sending;
}
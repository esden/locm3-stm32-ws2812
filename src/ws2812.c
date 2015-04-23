/*
 *
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2015 Jack Ziesing <jziesing@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

#include "ws2812.h"

#define WS2812_BIT_BUFFER_SIZE (24*6)

enum ws2812_stage {
	ws2812_idle,
	ws2812_sending,
	ws2812_done,
	ws2812_reset
};

struct ws2812_status {
	ws2812_led_t *leds;
	int led_count;
	int leds_sent;
	volatile enum ws2812_stage stage;
	uint16_t bit_buffer[WS2812_BIT_BUFFER_SIZE];
} ws2812_status;

/* Private function declarations. */
void ws2812_gpio_init(void);
void ws2812_tim_init(void);
void ws2812_dma_init(void);
void ws2812_init_bit_buffer(void);
void ws2812_fill_low_bit_buffer(void);
void ws2812_fill_high_bit_buffer(void);
void ws2812_fill_bit_buffer(bool low_high);
void ws2812_clear_bit_buffer(void);

/* API functions. */
void ws2812_init(void)
{
	ws2812_gpio_init();
	ws2812_tim_init();
	ws2812_dma_init();

	ws2812_status.stage = ws2812_idle;
}

void ws2812_dma_start(void)
{
	dma_enable_stream(DMA1, DMA_STREAM6);
}

void ws2812_dma_stop(void)
{
	dma_disable_stream(DMA1, DMA_STREAM6);
}

void ws2812_send(ws2812_led_t *leds, int led_count)
{
	ws2812_status.leds = leds;
	ws2812_status.led_count = led_count;
	ws2812_status.leds_sent = 0;

	ws2812_init_bit_buffer();
	ws2812_dma_start();
}

bool ws2812_is_sending(void)
{
	return (ws2812_status.stage != ws2812_idle);
}

/* Private function implementations. */
void ws2812_init_bit_buffer(void)
{
	ws2812_fill_low_bit_buffer();
	ws2812_fill_high_bit_buffer();
}

void ws2812_fill_low_bit_buffer(void)
{
	ws2812_fill_bit_buffer(false);
}

void ws2812_fill_high_bit_buffer(void)
{
	ws2812_fill_bit_buffer(true);
}

void ws2812_fill_bit_buffer(bool low_high)
{
	int offset = 0;
	int bitcount = WS2812_BIT_BUFFER_SIZE / 2;
	int led = ws2812_status.leds_sent;
	int i;

	ws2812_status.stage = ws2812_sending;

	if(low_high) {
		offset = bitcount;
	}

	/*
	 * 60 = 1
	 * 29 = 0
	 */
	for(i = 0; i < bitcount; i++) {
		if (i < ((ws2812_status.led_count - ws2812_status.leds_sent) * 24)) {
			if (((ws2812_status.leds[ws2812_status.leds_sent + (i/24)].grbu >> (31 - (i % 24)))
				 & 0x00000001) != 0) {
				ws2812_status.bit_buffer[offset + i] = 60;
			} else {
				ws2812_status.bit_buffer[offset + i] = 29;
			}
			led = ws2812_status.leds_sent + ((i + 0) / 24);
		} else {
			ws2812_status.stage = ws2812_done;
			break;
		}
	}

	for(; i < bitcount; i++) {
			ws2812_status.bit_buffer[offset + i] = 0;
	}

	ws2812_status.leds_sent = led + 1;
}

void ws2812_clear_bit_buffer(void) {
	for(int i = 0; i < WS2812_BIT_BUFFER_SIZE; i++) {
		ws2812_status.bit_buffer[i] = 0;
	}
}

/* Hardware dependent code. */
void ws2812_gpio_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 | GPIO14);
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12);
    /* Not sure why this does not really do the job if we have a pullup to 5V */
    /* gpio_set_output_options(GPIOD, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO12); */
}

void ws2812_tim_init(void)
{
	rcc_periph_clock_enable(RCC_TIM4);
    timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 0);
    timer_continuous_mode(TIM4);
    timer_set_period(TIM4, 104); /* 168000000 / 2 / 800000 (800khz pwm) */
    timer_disable_oc_output(TIM4, TIM_OC1);
    timer_disable_oc_clear(TIM4, TIM_OC1);
    timer_enable_oc_preload(TIM4, TIM_OC1);
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    timer_set_oc_value(TIM4, TIM_OC1, 0);
    timer_enable_oc_output(TIM4, TIM_OC1);
    timer_enable_preload(TIM4);

    timer_enable_irq(TIM4, TIM_DIER_UDE);

    timer_enable_counter(TIM4);
}

void ws2812_dma_init(void)
{
	rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
	dma_stream_reset(DMA1, DMA_STREAM6);
    dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_VERY_HIGH);
    dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_16BIT);
    dma_enable_circular_mode(DMA1, DMA_STREAM6);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
    dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&TIM4_CCR1);
    dma_set_memory_address(DMA1, DMA_STREAM6, (uint32_t)(&ws2812_status.bit_buffer[0]));
    dma_set_number_of_data(DMA1, DMA_STREAM6, WS2812_BIT_BUFFER_SIZE);
    dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM6);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
    nvic_clear_pending_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
}

/* Interrupt handlers. */
void dma1_stream6_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_HTIF) != 0) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_HTIF);

        gpio_toggle(GPIOD, GPIO13);

        if(ws2812_status.stage != ws2812_idle){
        	if(ws2812_status.stage == ws2812_done) {
        		ws2812_fill_low_bit_buffer();
        		ws2812_status.stage = ws2812_idle;
        	} else {
        		ws2812_fill_low_bit_buffer();
        	}
        } else {
        	ws2812_clear_bit_buffer();
        	ws2812_dma_stop();
        	timer_set_oc_value(TIM4, TIM_OC1, 0);
        }

    }
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF) != 0) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);

        gpio_toggle(GPIOD, GPIO14);

        if(ws2812_status.stage != ws2812_idle){
            if(ws2812_status.stage == ws2812_done) {
        		ws2812_fill_high_bit_buffer();
        	    ws2812_status.stage = ws2812_idle;
            } else {
        	    ws2812_fill_high_bit_buffer();
            }
        } else {
        	ws2812_clear_bit_buffer();
        	ws2812_dma_stop();
        	timer_set_oc_value(TIM4, TIM_OC1, 0);
        }

    }
}
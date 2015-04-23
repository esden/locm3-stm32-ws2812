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

#ifndef WS2812_H
#define WS2812_H

typedef union {
      struct __attribute__ ((__packed__)) {
	uint8_t _unused;
	uint8_t b;
	uint8_t r;
	uint8_t g;
      } colors;
      uint32_t grbu;
} ws2812_led_t;

void ws2812_init(void);
void ws2812_dma_start(void);
void ws2812_dma_stop(void);
void ws2812_send(ws2812_led_t *leds, int led_count);
bool ws2812_is_sending(void);

#endif /* WS2812_H */
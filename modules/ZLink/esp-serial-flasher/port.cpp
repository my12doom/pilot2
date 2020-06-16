/* Copyright 2020 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "serial_io.h"
#include <HAL/Interface/IUART.h>
#include <HAL/Interface/IGPIO.h>
#include <HAL/Interface/ISysTimer.h>

#include <stdio.h>
#include <string.h>

extern HAL::IUART *esp_uart;
extern HAL::IGPIO *esp_io0;
extern HAL::IGPIO *esp_rst;

//#define SERIAL_DEBUG_ENABLE

#ifdef SERIAL_DEBUG_ENABLE

static void dec_to_hex_str(const uint8_t dec, uint8_t hex_str[3])
{
    static const uint8_t dec_to_hex[] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
    };

    hex_str[0] = dec_to_hex[(dec >> 4)];
    hex_str[1] = dec_to_hex[(dec & 0xF)];
    hex_str[2] = '\0';
}

int count = 0;
static void serial_debug_print(const uint8_t *data, uint16_t size, bool write)
{
    static bool write_prev = false;
    uint8_t hex_str[3];

    if(write_prev != write) {
        write_prev = write;
        printf("(%d bytes)\n--- %s ---\n", count, write ? "WRITE" : "READ");
		count = 0;
    }

    for(uint32_t i = 0; i < size; i++) {
        dec_to_hex_str(data[i], hex_str);
        printf("%s ", hex_str);
    }

	count += size;
}

#else

static void serial_debug_print(const uint8_t *data, uint16_t size, bool write) { }

#endif

esp_loader_error_t loader_port_serial_init(const loader_serial_config_t *config)
{
    return ESP_LOADER_SUCCESS;
}


esp_loader_error_t loader_port_serial_write(const uint8_t *data, uint16_t size, uint32_t timeout)
{
	serial_debug_print(data, size, true);
		esp_uart->write(data, size);
    return ESP_LOADER_SUCCESS;
}


esp_loader_error_t loader_port_serial_read(uint8_t *data, uint16_t size, uint32_t timeout)
{
	memset(data, 0, size);

	uint32_t timeout_tick = systimer->gettime() + timeout*1000;
	while (esp_uart->available() < size && systimer->gettime() < timeout_tick)
		;
	
	if (esp_uart->available() < size)
		return ESP_LOADER_ERROR_TIMEOUT;
	
	int read = esp_uart->read(data, size);

	if (read > 0)
		serial_debug_print(data, read, false);

    if (read < 0) {
        return ESP_LOADER_ERROR_FAIL;
    } else if (read < size) {
        return ESP_LOADER_ERROR_TIMEOUT;
    } else {
        return ESP_LOADER_SUCCESS;
    }
}


// Set GPIO0 LOW, then
// assert reset pin for 50 milliseconds.
void loader_port_enter_bootloader(void)
{
	esp_io0->set_mode(HAL::MODE_OUT_PushPull);
	esp_rst->set_mode(HAL::MODE_OUT_PushPull);
	esp_io0->write(0);
	esp_rst->write(0);
	systimer->delayms(50);
	esp_rst->write(1);
}


void loader_port_reset_target(void)
{
	esp_io0->set_mode(HAL::MODE_OUT_PushPull);
	esp_rst->set_mode(HAL::MODE_OUT_PushPull);
	esp_io0->write(1);
	esp_rst->write(0);
	systimer->delayms(50);
	esp_rst->write(1);
}


void loader_port_delay_ms(uint32_t ms)
{
	systimer->delayms(ms);
}

static uint32_t t;

void loader_port_start_timer(uint32_t ms)
{
	t = systimer->gettime() + ms * 1000;
}


uint32_t loader_port_remaining_time(void)
{
	int remaining = (t - systimer->gettime())/1000;
	return (remaining > 0) ? remaining : 0;
}


void loader_port_debug_print(const char *str)
{
}

esp_loader_error_t loader_port_change_baudrate(uint32_t baudrate)
{
	esp_uart->set_baudrate(baudrate);

	return ESP_LOADER_SUCCESS;
}

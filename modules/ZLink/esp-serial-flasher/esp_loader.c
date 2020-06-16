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

#include "serial_comm_prv.h"
#include "serial_comm.h"
#include "serial_io.h"
#include "esp_loader.h"
#include <string.h>

#ifndef MAX
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
#endif

static const uint32_t DEFAULT_TIMEOUT = 500;
static const uint32_t SPI_PIN_CONFIG_DEFAULT = 0;
static const uint32_t DEFAULT_FLASH_TIMEOUT = 5000;       // timeout for most flash operations
static const uint32_t ERASE_REGION_TIMEOUT_PER_MB = 5000; // timeout (per megabyte) for erasing a region
static const uint8_t  PADDING_PATTERN = 0xFF;
static const uint32_t MD5_TIMEOUT_PER_MB = 800;


static uint32_t timeout_per_mb(uint32_t size_bytes, uint32_t time_per_mb)
{
    uint32_t timeout = ERASE_REGION_TIMEOUT_PER_MB * (size_bytes / 1e6);
    return MAX(timeout, DEFAULT_FLASH_TIMEOUT);
}


esp_loader_error_t esp_loader_connect(esp_loader_connect_args_t *connect_args)
{
    esp_loader_error_t err;
    int32_t trials = connect_args->trials;

    do {
        loader_port_start_timer(connect_args->sync_timeout);
        err = loader_sync_cmd();
        if (err == ESP_LOADER_ERROR_TIMEOUT) {
            if (--trials == 0) {
                return ESP_LOADER_ERROR_TIMEOUT;
            }
            loader_port_delay_ms(100);
        } else if (err != ESP_LOADER_SUCCESS) {
            return err;
        }
    } while (err != ESP_LOADER_SUCCESS);

	return ESP_LOADER_SUCCESS;
}

esp_loader_error_t esp_loader_flash_start(uint32_t offset, uint32_t image_size, uint32_t block_size)
{
    uint32_t blocks_to_write = (image_size + block_size - 1) / block_size;
    uint32_t erase_size = block_size * blocks_to_write;

    loader_port_start_timer(timeout_per_mb(erase_size, ERASE_REGION_TIMEOUT_PER_MB));

    return loader_flash_begin_cmd(offset, erase_size, block_size, blocks_to_write);
}

esp_loader_error_t esp_loader_flash_write(void *payload, uint32_t size)
{
    uint8_t *data = (uint8_t *)payload;

    loader_port_start_timer(DEFAULT_TIMEOUT*10);

    return loader_flash_data_cmd(data, size);
}


esp_loader_error_t esp_loader_flash_finish(bool reboot)
{
    loader_port_start_timer(DEFAULT_TIMEOUT);

    return loader_flash_end_cmd(!reboot);
}


esp_loader_error_t esp_loader_memory_start(uint32_t offset, uint32_t image_size, uint32_t block_size)
{
	uint32_t blocks_to_write = (image_size + block_size - 1) / block_size;
	uint32_t erase_size = block_size * blocks_to_write;
	
	loader_port_start_timer(timeout_per_mb(erase_size, ERASE_REGION_TIMEOUT_PER_MB));

	return loader_memory_begin_cmd(offset, erase_size, block_size, blocks_to_write);
}


esp_loader_error_t esp_loader_memory_write(void *payload, uint32_t size)
{
	uint8_t *data = (uint8_t *)payload;
	
	loader_port_start_timer(DEFAULT_TIMEOUT);

	return loader_memory_data_cmd(data, size);
}


esp_loader_error_t esp_loader_memory_finish(uint32_t run_flag, uint32_t entry_address)
{
	loader_port_start_timer(DEFAULT_TIMEOUT);

	return loader_memory_end_cmd(run_flag, entry_address);
}


esp_loader_error_t esp_loader_read_register(uint32_t address, uint32_t *reg_value)
{
    loader_port_start_timer(DEFAULT_TIMEOUT);

    return loader_read_reg_cmd(address, reg_value);
}


esp_loader_error_t esp_loader_write_register(uint32_t address, uint32_t reg_value)
{
    loader_port_start_timer(DEFAULT_TIMEOUT);

    return loader_write_reg_cmd(address, reg_value, 0xFFFFFFFF, 0);
}

esp_loader_error_t esp_loader_change_baudrate(uint32_t baudrate, uint32_t old_baudrate)
{
    loader_port_start_timer(DEFAULT_TIMEOUT);

    return loader_change_baudrate_cmd(baudrate, old_baudrate);
}

esp_loader_error_t esp_loader_read_md5(uint8_t md5_out[16], uint32_t address, int byte_count)
{
    uint8_t received_md5[MD5_SIZE + 1];

    loader_port_start_timer(timeout_per_mb(byte_count, MD5_TIMEOUT_PER_MB));

    RETURN_ON_ERROR( loader_md5_cmd(address, byte_count, received_md5) );

	memcpy(md5_out, received_md5, 16);
	
    return ESP_LOADER_SUCCESS;
}

void esp_loader_reset_target(void)
{
    loader_port_reset_target();
}
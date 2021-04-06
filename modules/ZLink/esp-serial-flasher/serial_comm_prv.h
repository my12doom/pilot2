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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

#define STATUS_FAILURE  1
#define STATUS_SUCCESS  0

#define READ_DIRECTION  1
#define WRITE_DIRECTION 0

#define MD5_SIZE 32

typedef enum
{
    FLASH_BEGIN = 0x02,
    FLASH_DATA  = 0x03,
    FLASH_END   = 0x04,
    MEM_BEGIN   = 0x05,
    MEM_END     = 0x06,
    MEM_DATA    = 0x07,
    SYNC        = 0x08,
    WRITE_REG   = 0x09,
    READ_REG    = 0x0a,

    SPI_SET_PARAMS   = 0x0b,
    SPI_ATTACH       = 0x0d,
    CHANGE_BAUDRATE  = 0x0f,
    FLASH_DEFL_BEGIN = 0x10,
    FLASH_DEFL_DATA  = 0x11,
    FLASH_DEFL_END   = 0x12,
    SPI_FLASH_MD5    = 0x13,
} command_t;

typedef enum
{
    RESPONSE_OK     = 0x00,
    INVALID_COMMAND = 0x05, // parameters or length field is invalid
    COMMAND_FAILED  = 0x06, // Failed to act on received message
    INVALID_CRC     = 0x07, // Invalid CRC in message
    FLASH_WRITE_ERR = 0x08, // After writing a block of data to flash, the ROM loader reads the value back and the 8-bit CRC is compared to the data read from flash. If they don't match, this error is returned.
    FLASH_READ_ERR  = 0x09, // SPI read failed
    READ_LENGTH_ERR = 0x0a, // SPI read request length is too long
    DEFLATE_ERROR   = 0x0b, // ESP32 compressed uploads only
} error_code_t;

typedef struct
{
    uint8_t direction;
    uint8_t command;    // One of command_t
    uint16_t size;
    uint32_t checksum;
} command_common_t;

typedef struct
{
    command_common_t common;
    uint32_t erase_size;
    uint32_t packet_count;
    uint32_t packet_size;
    uint32_t offset;
} begin_command_t;

typedef struct
{
    command_common_t common;
    uint32_t data_size;
    uint32_t sequence_number;
    uint32_t zero_0;
    uint32_t zero_1;
} data_command_t;

typedef struct
{
    command_common_t common;
    uint32_t stay_in_loader;
} flash_end_command_t;

typedef struct
{
    command_common_t common;
    uint32_t run_flag;
    uint32_t entry_point_address;
} mem_end_command_t;

typedef struct
{
    command_common_t common;
    uint8_t sync_sequence[36];
} sync_command_t;

typedef struct
{
    command_common_t common;
    uint32_t address;
    uint32_t value;
    uint32_t mask;
    uint32_t delay_us;
} write_reg_command_t;

typedef struct
{
    command_common_t common;
    uint32_t address;
} read_reg_command_t;

typedef struct
{
    command_common_t common;
    uint32_t configuration;
    //uint32_t zero; // ESP32 ROM only
} spi_attach_command_t;

typedef struct
{
    command_common_t common;
    uint32_t new_baudrate;
    uint32_t old_baudrate;
} change_baudrate_command_t;

typedef struct
{
    command_common_t common;
    uint32_t address;
    uint32_t size;
    uint32_t reserved_0;
    uint32_t reserved_1;
} spi_flash_md5_command_t;

typedef struct
{
    uint8_t direction;
    uint8_t command;    // One of command_t
    uint16_t size;
    uint32_t value;
} common_response_t;

typedef struct
{
    uint8_t failed;
    uint8_t error;
    //uint8_t reserved_0; // ESP32 ROM only
    //uint8_t reserved_1; // ESP32 ROM only
} response_status_t;

typedef struct
{
    common_response_t common;
    response_status_t status;
} response_t;

typedef struct
{
    common_response_t common;
    uint8_t md5[16];     // ROM only
    response_status_t status;
} rom_md5_response_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif
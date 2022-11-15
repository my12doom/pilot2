#include "board.h"
#include <Protocol/common.h>

HAL::ISPI *spi = NULL;
HAL::IGPIO *cs = NULL;
HAL::IGPIO *ce = NULL;
HAL::IGPIO *irq = NULL;
HAL::IGPIO *dbg = NULL;
HAL::IGPIO *dbg2 = NULL;
HAL::IInterrupt *interrupt = NULL;
HAL::ITimer *timer = NULL;

HAL::IGPIO *bind_button = NULL;
HAL::IGPIO *vibrator = NULL;
HAL::IGPIO *downlink_led = NULL;

HAL::IGPIO *SCL = NULL;
HAL::IGPIO *SDA = NULL;
HAL::IRCOUT *ppm = NULL;
HAL::IUART *ebus = NULL;
HAL::IUART *sbus = NULL;

HAL::IUART *telemetry = NULL;
bool is_tx = false;

WEAK void select_ant(uint32_t *randomizer, bool tx)
{
}

WEAK void read_keys(uint8_t* key, int max_keys)
{
}
WEAK void custom_output(uint8_t * payload, int payload_size, int latency)
{
}
#include "board.h"

extern HAL::ISPI *spi = NULL;
extern HAL::IGPIO *cs = NULL;
extern HAL::IGPIO *ce = NULL;
extern HAL::IGPIO *irq = NULL;
extern HAL::IGPIO *dbg = NULL;
extern HAL::IGPIO *dbg2 = NULL;
extern HAL::IInterrupt *interrupt = NULL;
extern HAL::ITimer *timer = NULL;

extern HAL::IGPIO *bind_button = NULL;
extern HAL::IGPIO *vibrator = NULL;

extern HAL::IGPIO *SCL = NULL;
extern HAL::IGPIO *SDA = NULL;
extern HAL::IRCOUT *ppm = NULL;
HAL::IUART *uart = NULL;
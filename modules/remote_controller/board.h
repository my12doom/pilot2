#pragma once

#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>
#include <HAL/Interface/IInterrupt.h>
#include <HAL/Interface/ITimer.h>
#include <HAL/Interface/IRCOUT.h>

extern HAL::ISPI *spi;
extern HAL::IGPIO *cs;
extern HAL::IGPIO *ce;
extern HAL::IGPIO *irq;
extern HAL::IGPIO *dbg;
extern HAL::IGPIO *dbg2;
extern HAL::IInterrupt *interrupt;
extern HAL::ITimer *timer;

extern HAL::IGPIO *SCL;
extern HAL::IGPIO *SDA;
extern HAL::IRCOUT *ppm;

int board_init();
void read_channels(int16_t *channel, int max_channel_count);
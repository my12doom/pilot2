#pragma once

#include <stdint.h>
#include "stm32f4xx_gpio.h"

#define max_adc_data_count 1024

class dma_adc
{
public:
    typedef void (*dma_full_cb)(void *parameter);
    int16_t adc_data[max_adc_data_count];

public:
    dma_adc();
    ~dma_adc();

    int set_cb(dma_full_cb cb, void *user);
    int init(GPIO_TypeDef *GPIOx, int channel);
    int begin();
    int stop();
    bool full();
    int pos();

    // for irq
    void irq();
protected:
    bool _full;
    dma_full_cb cb;
    void *user;
};

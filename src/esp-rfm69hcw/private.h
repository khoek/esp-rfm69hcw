#ifndef __LIB__RFM69HCW_PRIVATE_H
#define __LIB__RFM69HCW_PRIVATE_H

#include <driver/gpio.h>
#include <driver/spi_master.h>

#include "rfm69hcw.h"

static const char* TAG = "rfm69hcw";

struct rfm69hcw {
    gpio_num_t pin_rst;
    gpio_num_t pin_irq;

    spi_device_handle_t spi;
};

#endif
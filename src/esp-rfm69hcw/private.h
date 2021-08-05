#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>

#include "device/rfm69hcw.h"

static const char* TAG = "rfm69hcw";

struct rfm69hcw {
    gpio_num_t pin_rst;

    spi_device_handle_t spi;
};

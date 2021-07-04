#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <libesp.h>
#include <rfm69hcw.h>
#include <string.h>

static const char* TAG = "rfm69hcw";

#define ALWAYS_INLINE __attribute__((always_inline)) inline

void rfm69hcw_init(spi_host_device_t host, gpio_num_t pin_cs, gpio_num_t pin_rst, gpio_num_t pin_irq, rfm69hcw_handle_t* out_dev) {
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // FIXME has been divided by 10
        .mode = 0,
        .spics_io_num = pin_cs,
        .queue_size = 1,
        .input_delay_ns = 0,
    };

    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << pin_rst);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(pin_rst, 0);

    rfm69hcw_handle_t dev = malloc(sizeof(rfm69hcw_t));
    dev->pin_rst = pin_rst;
    dev->pin_irq = pin_irq;
    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, &dev->spi));

    dev->irq_sem = xSemaphoreCreateBinary();

    rfm69hcw_reset(dev);

    *out_dev = dev;
}

static void isr_irq(void* arg) {
    rfm69hcw_t* dev = arg;

    // First, since this interrupt is level-triggered (so we don't miss any) disable further
    // interrupts. If the `irq_loop()` happens to enable this interrupt again asynchronously
    // with this, we don't mind, since `sleep_until_irq_high()` clears spurious pending
    // messages.
    gpio_intr_disable(dev->pin_irq);

    // Only now give the semaphore, which will ensure that the `irq_loop()` will always
    // eventually take the semaphore and call `gpio_intr_enable()` for us. (We don't care
    // about the return value, i.e. whether we were able to.)
    xSemaphoreGiveFromISR(dev->irq_sem, NULL);
}

static ALWAYS_INLINE void sleep_until_irq_high(const rfm69hcw_t* dev) {
    // Clear spurious pending messages.
    xSemaphoreTake(dev->dreq_sem, 0);

    // If we are here, IRQ is low. First, enable the IRQ ISR. Since the ISR disables
    // itself before it is possible to take `player->irq_sem`, a race condition cannot
    // occur here.
    gpio_intr_enable(dev->pin_irq);

    // Wait until the ISR tells us IRQ has gone high again.
    while (xSemaphoreTake(dev->irq_sem, portMAX_DELAY) == pdFALSE)
        ;

    // The concurrency "invariant" at this point is that we are guarenteed that the ISR
    // handler is disabled.
}

static void irq_loop(rfm69hcw_t* dev) {
}

static void irq_task(void* arg) {
    rfm69hcw_t* dev = arg;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_HIGH_LEVEL,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << dev->pin_irq),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Prevent the level trigger interrupt on the IRQ pin
    // from hanging up the core when we enable interrupts.
    gpio_intr_disable(dev->pin_irq);

    // TODO Install in a way which doesn't collide with other code.
    // (Note that we need `gpio_install_isr_service()` to have been
    // called without the flag which requires IRAM ISRs.)
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin_irq, isr_irq, dev);

    irq_loop(dev);

    UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
    if (stack_remaining <= 0) {
        ESP_LOGE(TAG, "stack overflow detected!");
    }

    ESP_LOGD(TAG, "stack words remaining: %u", stack_remaining);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_isr_handler_remove(dev->pin_irq);
    gpio_uninstall_isr_service();

    gpio_intr_enable(dev->pin_irq);
}

void rfm69hcw_reset(rfm69hcw_handle_t dev) {
    ESP_LOGI(TAG, "resetting");

    gpio_set_level(dev->pin_rst, 1);
    util_wait_micros(100);
    gpio_set_level(dev->pin_rst, 0);
    vTaskDelay(1 + (5 / portTICK_PERIOD_MS));

    while (!(rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1) & RFM69HCW_IRQ_FLAGS_1_MODE_READY)) {
        vTaskDelay(1);
    }
}

void rfm69hcw_set_sync(rfm69hcw_handle_t dev, const uint8_t* sync_value, uint8_t sync_tol) {
    uint8_t len = 0;
    while (sync_value && sync_value[len] && len <= 8) {
        len++;
    }

    if (len > 8) {
        ESP_LOGE(TAG, "sync value too long! (8 bytes max)");
        len = 8;
    }

    if (!len) {
        rfm69hcw_reg_write(dev, RFM69HCW_REG_SYNC_CONFIG, MK_RFM69HCW_SYNC_CONFIG(false, true, 0, sync_tol));
    } else {
        rfm69hcw_reg_write(dev, RFM69HCW_REG_SYNC_CONFIG, MK_RFM69HCW_SYNC_CONFIG(true, false, len - 1, sync_tol));
    }

    for (uint8_t reg = RFM69HCW_REG_SYNC_VALUE_1; reg <= RFM69HCW_REG_SYNC_VALUE_8 && len; reg++) {
        rfm69hcw_reg_write(dev, reg, *sync_value++);
        len--;
    }
}

void rfm69hcw_set_power_level(rfm69hcw_handle_t dev, rfm69hcw_power_level_t level) {
    switch (level) {
        case RFM69HCW_POWER_RX_or_PA0: {
            // Correct values for PA0 only or RX mode
            rfm69hcw_reg_write(dev, RFM69HCW_REG_PA_LEVEL, 0x9F);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_OCP, 0x1A);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_TEST_PA1, 0x55);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_TEST_PA2, 0x70);
            break;
        }
        case RFM69HCW_POWER_TX_NORMAL: {
            rfm69hcw_reg_write(dev, RFM69HCW_REG_PA_LEVEL, 0x7F);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_OCP, 0x0F);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_TEST_PA1, 0x55);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_TEST_PA2, 0x70);
            break;
        }
        case RFM69HCW_POWER_TX_HIGH: {
            // Note max 1% duty cycle, and MUST be off when using PA0 and/or RX mode.
            rfm69hcw_reg_write(dev, RFM69HCW_REG_PA_LEVEL, 0x7F);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_OCP, 0x0F);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_TEST_PA1, 0x5D);
            rfm69hcw_reg_write(dev, RFM69HCW_REG_TEST_PA2, 0x7C);
            break;
        }
        default: {
            ESP_LOGE(TAG, "unknown power level: %d", level);
            return;
        }
    }
}

void rfm69hcw_enter_rx(rfm69hcw_handle_t dev, const rfm69hcw_rx_config_t* cfg) {
    rfm69hcw_reg_write(dev, RFM69HCW_REG_OP_MODE, RFM69HCW_OP_MODE_MODE_STANDBY);
    while (!(rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1) & RFM69HCW_IRQ_FLAGS_1_MODE_READY)) {
        vTaskDelay(1);
    }

    rfm69hcw_set_power_level(dev, RFM69HCW_POWER_RX_or_PA0);

    uint8_t shaping = cfg->type == RFM69HCW_MODULATION_TYPE_FSK ? cfg->fsk_shaping : cfg->ook_shaping;
    rfm69hcw_reg_write(dev, RFM69HCW_REG_DATA_MODUL, (((uint8_t) cfg->data_mode) << SHIFT_RFM69HCW_DATA_MODE) | (((uint8_t) cfg->type) << SHIFT_RFM69HCW_MODULATION_TYPE) | (shaping << SHIFT_RFM69HCW_MODULATION_SHAPING));

    // Required by specification, ensures no overflow below.
    assert(cfg->freq_khz <= 1020000);
    assert(cfg->bit_period_ns <= 833333);

    // This will never overflow a 32-bit unsigned in if: * `freq_hz` < 2 million.
    //                                                   * `bit_period_ns` < 1 million.
    uint32_t freq_steps = (cfg->freq_khz << 11) / ((uint32_t) 125);
    uint32_t bitrate_divisor = (cfg->bit_period_ns << 2) / ((uint32_t) 125);

    assert(!(freq_steps & ~0x00FFFFFF));
    assert(!(bitrate_divisor & ~0x0000FFFF));

    // Note must write LSB last, this triggers the operation which actually effects the frequency change.
    rfm69hcw_reg_write(dev, RFM69HCW_REG_FRF_MSB, (freq_steps & (0xFFULL << 16)) >> 16);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_FRF_MID, (freq_steps & (0xFFULL << 8)) >> 8);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_FRF_LSB, (freq_steps & (0xFFULL << 0)) >> 0);

    rfm69hcw_reg_write(dev, RFM69HCW_REG_BITRATE_MSB, (bitrate_divisor & (0xFFULL << 8)) >> 8);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_BITRATE_LSB, (bitrate_divisor & (0xFFULL << 0)) >> 0);

    uint8_t rx_bw = (((uint8_t) cfg->dcc_cutoff) << SHIFT_RFM69HCW_DCC_CUTOFF) | (((uint8_t) cfg->rx_bw) << SHIFT_RFM69HCW_RX_BW);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_RX_BW, rx_bw);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_AFC_BW, rx_bw);

    rfm69hcw_reg_write(dev, RFM69HCW_REG_AFC_CTRL, 0x00);  // TODO support `AfcLowBetaOn`
    rfm69hcw_reg_write(dev, RFM69HCW_REG_AFC_FEI, RFM69HCW_AFC_FEI_AFC_AUTO_ON | RFM69HCW_AFC_FEI_AFC_AUTOCLEAR_ON);

    // FIXME OOK register stuff?
    // rfm69hcw_reg_write(dev, RFM69HCW_REG_OOK_PEAK, 0x78);

    rfm69hcw_reg_write(dev, RFM69HCW_REG_DIO_MAPPING_1, RFM69HCW_DIO_MAPPING_1_DIO_0_PAYLOAD_READY);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_DIO_MAPPING_2, RFM69HCW_DIO_MAPPING_2_CLK_OUT_OFF);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_PACKET_CONFIG_1, MK_RFM69HCW_PACKET_CONFIG_1(false, RFM69HCW_DC_FREE_NONE, false, false, RFM69HCW_ADDRESS_FILTERING_NONE));
    rfm69hcw_reg_write(dev, RFM69HCW_REG_PACKET_CONFIG_2, MK_RFM69HCW_PACKET_CONFIG_2(0x0, false, true, false));

    rfm69hcw_set_sync(dev, cfg->sync_value, cfg->sync_bit_tol);
    rfm69hcw_reg_write(dev, RFM69HCW_REG_PAYLOAD_LENGTH, cfg->payload_len);

    // CUSTOM!!!
    // FIXME DO SOME MEASUREMENTS AND CALIBRATE THIS
    rfm69hcw_reg_write(dev, RFM69HCW_REG_RSSI_THRESH, 0xA0);

    rfm69hcw_reg_write(dev, RFM69HCW_REG_OP_MODE, RFM69HCW_OP_MODE_MODE_RX);
    while (!(rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1) & RFM69HCW_IRQ_FLAGS_1_MODE_READY)) {
        vTaskDelay(1);
    }
}

void rfm69hcw_managed_enter_tx(rfm69hcw_handle_t dev /*, const rfm69hcw_rx_config_t* cfg*/) {
    rfm69hcw_reg_write(dev, RFM69HCW_REG_OP_MODE, RFM69HCW_OP_MODE_MODE_STANDBY);
    // rfm69hcw_set_power_level(dev, RFM69HCW_POWER_????);

    // FIXME this.

    // FIXME abstract RFM69HCW_REG_FIFO_THRESH
    // rfm69hcw_reg_write(dev, RFM69HCW_REG_FIFO_THRESH, 0x01);

    {
        // rfm69hcw_reg_write(dev, RFM69HCW_REG_FDEV_MSB, 0x02);
        // rfm69hcw_reg_write(dev, RFM69HCW_REG_FDEV_LSB, 0x3E);
        // rfm69hcw_reg_write(dev, RFM69HCW_REG_PREAMBLE_MSB, 0x00);
        // rfm69hcw_reg_write(dev, RFM69HCW_REG_PREAMBLE_LSB, 0x06);

        // // FIXME how does the arduino library select this?
        // // rfm69hcw_reg_write(dev, RFM69HCW_REG_PA_RAMP, 0x0A);
    }
}

#define CMD_PREFIX_READ 0x00
#define CMD_PREFIX_WRITE 0x80
#define CMD_PREFIX_MASK 0x80

static uint8_t reg_access(rfm69hcw_handle_t dev, uint8_t addr_byte, uint8_t data_byte) {
    spi_transaction_t trans = {
        .length = 16,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .tx_data = {addr_byte, data_byte, 0, 0},
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi, &trans));

    bool is_write = addr_byte & CMD_PREFIX_MASK;

    uint8_t rx = trans.rx_data[1];
    ESP_LOGD(TAG, "reg_access(%c:0x%02X)=0x%02X", is_write ? 'W' : 'R', addr_byte & ~CMD_PREFIX_MASK, is_write ? data_byte : rx);
    return rx;
}

uint8_t rfm69hcw_reg_read(rfm69hcw_handle_t dev, rfm69hcw_reg_t addr) {
    return reg_access(dev, addr | CMD_PREFIX_READ, 0x00);
}

void rfm69hcw_reg_write(rfm69hcw_handle_t dev, rfm69hcw_reg_t addr, uint8_t val) {
    reg_access(dev, addr | CMD_PREFIX_WRITE, val);
}

void rfm69hcw_reg_block_write_fifo(rfm69hcw_handle_t dev, const uint8_t* buff, uint8_t len) {
    uint8_t* dma_buff = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    dma_buff[0] = CMD_PREFIX_MASK | RFM69HCW_REG_FIFO;
    memcpy(dma_buff + 1, buff, len);

    spi_transaction_t trans = {
        .length = 8 * (len + 1),
        .tx_buffer = dma_buff,
        .rx_buffer = dma_buff,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi, &trans));

    free(dma_buff);
}
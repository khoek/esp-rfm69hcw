#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <libesp.h>
#include <string.h>

#include "private.h"

esp_err_t rfm69hcw_init(spi_host_device_t host, gpio_num_t pin_cs,
                        gpio_num_t pin_rst, rfm69hcw_handle_t* out_dev) {
    esp_err_t ret;

    const spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // FIXME has been divided by 10
        .mode = 0,
        .spics_io_num = pin_cs,
        .queue_size = 1,
        .input_delay_ns = 0,
    };

    // As per spec, we should usually have the RST pin in Hi-Z mode, unless we
    // explicitly want to pull it up.
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin_rst);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    // FIXME Add a timeout
    while (gpio_get_level(pin_rst)) {
        vTaskDelay(1);
    }

    // As per spec, after a POR (power-on reset) we must wait 10ms after the
    // chip pulls the RST pin low before it is ready.
    vTaskDelay(1 + (10 / portTICK_PERIOD_MS));

    rfm69hcw_handle_t dev = malloc(sizeof(rfm69hcw_t));
    dev->pin_rst = pin_rst;
    ret = spi_bus_add_device(host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    uint8_t ver;
    ret = rfm69hcw_reg_read(dev, RFM69HCW_REG_VERSION, &ver);
    if (ret != ESP_OK) {
        goto rfm69hcw_init_fail;
    }

    switch (ver) {
        case RFM69HCW_VERSION_DRIVER_SUPPORTED: {
            // Current version supported by the driver.
            break;
        }
        default: {
            ESP_LOGE(TAG,
                     "unknown chip version (0x%02X), are pin numbers correct?",
                     ver);
            ret = ESP_FAIL;
            goto rfm69hcw_init_fail;
        }
    }

    ret = rfm69hcw_reset(dev);
    if (ret != ESP_OK) {
        goto rfm69hcw_init_fail;
    }

    *out_dev = dev;
    return ESP_OK;

rfm69hcw_init_fail:
    rfm69hcw_destroy(dev);
    return ret;
}

void rfm69hcw_destroy(rfm69hcw_handle_t dev) {
    ESP_ERROR_DISCARD(rfm69hcw_reg_write(dev, RFM69HCW_REG_OP_MODE,
                                         RFM69HCW_OP_MODE_MODE_STANDBY));
    spi_bus_remove_device(dev->spi);
    free(dev);
}

static esp_err_t wait_for_mode_ready(rfm69hcw_handle_t dev) {
    esp_err_t ret;

    while (1) {
        uint8_t irq_flags_1;
        ret = rfm69hcw_reg_read(dev, RFM69HCW_REG_IRQ_FLAGS_1, &irq_flags_1);
        if (ret != ESP_OK) {
            return ret;
        }

        if (irq_flags_1 & RFM69HCW_IRQ_FLAGS_1_MODE_READY) {
            break;
        }

        vTaskDelay(1);
    }

    return ESP_OK;
}

esp_err_t rfm69hcw_reset(rfm69hcw_handle_t dev) {
    esp_err_t ret;

    ESP_LOGD(TAG, "resetting");

    gpio_set_level(dev->pin_rst, 1);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << dev->pin_rst);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    io_conf.mode = GPIO_MODE_OUTPUT;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    // Spec says: pull up for 100us.
    util_wait_micros(100);

    // As per spec, the RST pin should usually be left in Hi-Z mode, unless we
    // explicitly want to pull it up during a reset (as we have just done).
    io_conf.mode = GPIO_MODE_INPUT;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    // Spec says: wait 5ms.
    vTaskDelay(1 + (5 / portTICK_PERIOD_MS));

    ret = wait_for_mode_ready(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t rfm69hcw_set_sync(rfm69hcw_handle_t dev, const uint8_t* sync_value,
                            uint8_t sync_tol) {
    esp_err_t ret;

    uint8_t len = 0;
    while (sync_value && sync_value[len] && len <= 8) {
        len++;
    }

    if (len > 8) {
        ESP_LOGE(TAG, "sync value too long! (8 bytes max)");
        len = 8;
    }

    if (!len) {
        ret = rfm69hcw_reg_write(
            dev, RFM69HCW_REG_SYNC_CONFIG,
            MK_RFM69HCW_SYNC_CONFIG(false, true, 0, sync_tol));
    } else {
        ret = rfm69hcw_reg_write(
            dev, RFM69HCW_REG_SYNC_CONFIG,
            MK_RFM69HCW_SYNC_CONFIG(true, false, len - 1, sync_tol));
    }

    if (ret != ESP_OK) {
        return ret;
    }

    for (uint8_t reg = RFM69HCW_REG_SYNC_VALUE_1;
         reg <= RFM69HCW_REG_SYNC_VALUE_8 && len; reg++) {
        ret = rfm69hcw_reg_write(dev, reg, *sync_value++);
        if (ret != ESP_OK) {
            return ret;
        }

        len--;
    }

    return ESP_OK;
}

esp_err_t rfm69hcw_set_power_level(rfm69hcw_handle_t dev,
                                   rfm69hcw_power_level_t level) {
    esp_err_t ret;

    uint8_t pa_level;
    uint8_t ocp;
    uint8_t test_pa1;
    uint8_t test_pa2;

    switch (level) {
        case RFM69HCW_POWER_RX_or_PA0: {
            // Correct values for PA0 only or RX mode
            pa_level = 0x9F;
            ocp = 0x1A;
            test_pa1 = 0x55;
            test_pa2 = 0x70;
            break;
        }
        case RFM69HCW_POWER_TX_NORMAL: {
            pa_level = 0x7F;
            ocp = 0x0F;
            test_pa1 = 0x55;
            test_pa2 = 0x70;
            break;
        }
        case RFM69HCW_POWER_TX_HIGH: {
            // Note max 1% duty cycle, and MUST be off when using PA0 and/or RX
            // mode.
            pa_level = 0x7F;
            ocp = 0x0F;
            test_pa1 = 0x5D;
            test_pa2 = 0x7C;
            break;
        }
        default: {
            ESP_LOGE(TAG, "unknown power level: %d", level);
            return ESP_FAIL;
        }
    }

    ret = rfm69hcw_reg_write(dev, RFM69HCW_REG_PA_LEVEL, pa_level);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = rfm69hcw_reg_write(dev, RFM69HCW_REG_OCP, ocp);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = rfm69hcw_reg_write(dev, RFM69HCW_REG_TEST_PA1, test_pa1);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = rfm69hcw_reg_write(dev, RFM69HCW_REG_TEST_PA2, test_pa2);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

#define RETURN_IF_ERR(expr)          \
    do {                             \
        esp_err_t _esp_err = (expr); \
        if (_esp_err != ESP_OK) {    \
            return _esp_err;         \
        }                            \
    } while (0)

esp_err_t rfm69hcw_configure_rx(rfm69hcw_handle_t dev,
                                const rfm69hcw_rx_config_t* cfg) {
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_OP_MODE,
                                     RFM69HCW_OP_MODE_MODE_STANDBY));

    RETURN_IF_ERR(wait_for_mode_ready(dev));

    RETURN_IF_ERR(rfm69hcw_set_power_level(dev, RFM69HCW_POWER_RX_or_PA0));

    uint8_t shaping = cfg->type == RFM69HCW_MODULATION_TYPE_FSK
                          ? cfg->fsk_shaping
                          : cfg->ook_shaping;
    RETURN_IF_ERR(rfm69hcw_reg_write(
        dev, RFM69HCW_REG_DATA_MODUL,
        MK_RFM69HCW_DATA_MODUL_FSK(cfg->data_mode, cfg->type, shaping)));

    // Required by specification, ensures no overflow below.
    assert(cfg->freq_khz <= 1020000);
    assert(cfg->bit_period_ns <= 833333);

    // This will never overflow a 32-bit unsigned in if: * `freq_hz` < 2
    // million.
    //                                                   * `bit_period_ns` < 1
    //                                                   million.
    uint32_t freq_steps = (cfg->freq_khz << 11) / ((uint32_t) 125);
    uint32_t bitrate_divisor = (cfg->bit_period_ns << 2) / ((uint32_t) 125);

    assert(!(freq_steps & ~0x00FFFFFF));
    assert(!(bitrate_divisor & ~0x0000FFFF));

    // Note must write LSB of FRF last, this triggers the operation which
    // actually effects the frequency change.
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_FRF_MSB,
                                     (freq_steps & (0xFFULL << 16)) >> 16));
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_FRF_MID,
                                     (freq_steps & (0xFFULL << 8)) >> 8));
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_FRF_LSB,
                                     (freq_steps & (0xFFULL << 0)) >> 0));

    // Now write BITRATE.
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_BITRATE_MSB,
                                     (bitrate_divisor & (0xFFULL << 8)) >> 8));
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_BITRATE_LSB,
                                     (bitrate_divisor & (0xFFULL << 0)) >> 0));

    uint8_t rx_bw = MK_RFM69HCW_BW(cfg->rx_bw, cfg->dcc_cutoff);
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_RX_BW, rx_bw));
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_AFC_BW, rx_bw));

    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_AFC_CTRL,
                                     0x00));  // TODO support `AfcLowBetaOn`
    RETURN_IF_ERR(rfm69hcw_reg_write(
        dev, RFM69HCW_REG_AFC_FEI,
        RFM69HCW_AFC_FEI_AFC_AUTO_ON | RFM69HCW_AFC_FEI_AFC_AUTOCLEAR_ON));

    // FIXME OOK register stuff?
    // rfm69hcw_reg_write(dev, RFM69HCW_REG_OOK_PEAK, 0x78);

    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_DIO_MAPPING_1,
                                     RFM69HCW_DIO_MAPPING_1_DIO_0_RSSI));
    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_DIO_MAPPING_2,
                                     RFM69HCW_DIO_MAPPING_2_CLK_OUT_OFF));
    RETURN_IF_ERR(rfm69hcw_reg_write(
        dev, RFM69HCW_REG_PACKET_CONFIG_1,
        MK_RFM69HCW_PACKET_CONFIG_1(false, RFM69HCW_DC_FREE_NONE, false, false,
                                    RFM69HCW_ADDRESS_FILTERING_NONE)));
    RETURN_IF_ERR(rfm69hcw_reg_write(
        dev, RFM69HCW_REG_PACKET_CONFIG_2,
        MK_RFM69HCW_PACKET_CONFIG_2(cfg->inter_packet_rx_delay, false, true,
                                    false)));

    RETURN_IF_ERR(rfm69hcw_set_sync(dev, cfg->sync_value, cfg->sync_bit_tol));
    RETURN_IF_ERR(
        rfm69hcw_reg_write(dev, RFM69HCW_REG_PAYLOAD_LENGTH, cfg->payload_len));

    // TODO add a function to measure the background noise floor
    RETURN_IF_ERR(
        rfm69hcw_reg_write(dev, RFM69HCW_REG_RSSI_THRESH,
                           cfg->rssi_thresh ? cfg->rssi_thresh : 0xFF));

    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_RX_TIMEOUT_2,
                                     cfg->timeout_rssi_thresh));

    RETURN_IF_ERR(rfm69hcw_reg_write(dev, RFM69HCW_REG_OP_MODE,
                                     RFM69HCW_OP_MODE_MODE_RX));

    RETURN_IF_ERR(wait_for_mode_ready(dev));

    return ESP_OK;
}

#define CMD_PREFIX_READ 0x00
#define CMD_PREFIX_WRITE 0x80
#define CMD_PREFIX_MASK 0x80

static uint8_t reg_access(rfm69hcw_handle_t dev, uint8_t addr, uint8_t tx,
                          uint8_t* rx) {
    esp_err_t ret;

    spi_transaction_t trans = {
        .length = 16,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .tx_data = {addr, tx, 0, 0},
    };

    ret = spi_device_polling_transmit(dev->spi, &trans);
    if (ret != ESP_OK) {
        return ret;
    }

    bool is_write = addr & CMD_PREFIX_MASK;

    *rx = trans.rx_data[1];
    ESP_LOGD(TAG, "reg_access(%c:0x%02X)=0x%02X", is_write ? 'W' : 'R',
             addr & ~CMD_PREFIX_MASK, is_write ? tx : *rx);

    return ESP_OK;
}

esp_err_t rfm69hcw_reg_read(rfm69hcw_handle_t dev, rfm69hcw_reg_t addr,
                            uint8_t* val) {
    return reg_access(dev, addr | CMD_PREFIX_READ, 0x00, val);
}

esp_err_t rfm69hcw_reg_write(rfm69hcw_handle_t dev, rfm69hcw_reg_t addr,
                             uint8_t val) {
    uint8_t rx;
    return reg_access(dev, addr | CMD_PREFIX_WRITE, val, &rx);
}

esp_err_t rfm69hcw_reg_block_write_fifo(rfm69hcw_handle_t dev,
                                        const uint8_t* buff, uint8_t len) {
    esp_err_t ret;

    uint8_t* dma_buff = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    dma_buff[0] = CMD_PREFIX_MASK | RFM69HCW_REG_FIFO;
    memcpy(dma_buff + 1, buff, len);

    spi_transaction_t trans = {
        .length = 8 * (len + 1),
        .tx_buffer = dma_buff,
        .rx_buffer = dma_buff,
    };
    ret = spi_device_polling_transmit(dev->spi, &trans);

    free(dma_buff);

    return ret;
}

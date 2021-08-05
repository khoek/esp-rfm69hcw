#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>

// The chip version of the RFM69HCW supported by this driver.
#define RFM69HCW_VERSION_DRIVER_SUPPORTED 0x24

typedef enum rfm69hcw_reg {
    RFM69HCW_REG_FIFO = 0x00,
    RFM69HCW_REG_OP_MODE = 0x01,
    RFM69HCW_REG_DATA_MODUL = 0x02,
    RFM69HCW_REG_BITRATE_MSB = 0x03,
    RFM69HCW_REG_BITRATE_LSB = 0x04,

    RFM69HCW_REG_FDEV_MSB = 0x05,
    RFM69HCW_REG_FDEV_LSB = 0x06,

    RFM69HCW_REG_FRF_MSB = 0x07,
    RFM69HCW_REG_FRF_MID = 0x08,
    RFM69HCW_REG_FRF_LSB = 0x09,

    RFM69HCW_REG_AFC_CTRL = 0x0B,

    RFM69HCW_REG_VERSION = 0x10,

    RFM69HCW_REG_PA_LEVEL = 0x11,
    RFM69HCW_REG_PA_RAMP = 0x12,
    RFM69HCW_REG_OCP = 0x13,

    RFM69HCW_REG_LNA = 0x18,
    RFM69HCW_REG_RX_BW = 0x19,
    RFM69HCW_REG_AFC_BW = 0x1A,
    RFM69HCW_REG_OOK_PEAK = 0x1B,
    RFM69HCW_REG_AFC_FEI = 0x1E,
    RFM69HCW_REG_AFC_MSB = 0x1F,
    RFM69HCW_REG_AFC_LSB = 0x20,
    RFM69HCW_REG_RSSI_VALUE = 0x24,

    RFM69HCW_REG_DIO_MAPPING_1 = 0x25,
    RFM69HCW_REG_DIO_MAPPING_2 = 0x26,

    RFM69HCW_REG_IRQ_FLAGS_1 = 0x27,
    RFM69HCW_REG_IRQ_FLAGS_2 = 0x28,

    RFM69HCW_REG_RSSI_THRESH = 0x29,

    RFM69HCW_REG_RX_TIMEOUT_1 = 0x2A,
    RFM69HCW_REG_RX_TIMEOUT_2 = 0x2B,

    RFM69HCW_REG_PREAMBLE_MSB = 0x2C,
    RFM69HCW_REG_PREAMBLE_LSB = 0x2D,

    RFM69HCW_REG_SYNC_CONFIG = 0x2E,
    RFM69HCW_REG_SYNC_VALUE_1 = 0x2F,
    RFM69HCW_REG_SYNC_VALUE_2 = 0x30,
    RFM69HCW_REG_SYNC_VALUE_3 = 0x31,
    RFM69HCW_REG_SYNC_VALUE_4 = 0x32,
    RFM69HCW_REG_SYNC_VALUE_5 = 0x33,
    RFM69HCW_REG_SYNC_VALUE_6 = 0x34,
    RFM69HCW_REG_SYNC_VALUE_7 = 0x35,
    RFM69HCW_REG_SYNC_VALUE_8 = 0x36,

    RFM69HCW_REG_PACKET_CONFIG_1 = 0x37,
    RFM69HCW_REG_PAYLOAD_LENGTH = 0x38,
    RFM69HCW_REG_FIFO_THRESH = 0x3C,
    RFM69HCW_REG_PACKET_CONFIG_2 = 0x3D,

    RFM69HCW_REG_TEST_PA1 = 0x5A,
    RFM69HCW_REG_TEST_PA2 = 0x5C,
    RFM69HCW_REG_TEST_DAGC = 0x6F,
} rfm69hcw_reg_t;

#define RFM69HCW_OP_MODE_MODE_SLEEP 0x00
#define RFM69HCW_OP_MODE_MODE_STANDBY 0x04
#define RFM69HCW_OP_MODE_MODE_TX 0x0C
#define RFM69HCW_OP_MODE_MODE_RX 0x10
#define MASK_RFM69HCW_OP_MODE_MODE 0x1C

#define RFM69HCW_AFC_FEI_FEI_DONE (0x1ULL << 6)
#define RFM69HCW_AFC_FEI_FEI_START (0x1ULL << 5)
#define RFM69HCW_AFC_FEI_AFC_DONE (0x1ULL << 4)
#define RFM69HCW_AFC_FEI_AFC_AUTOCLEAR_ON (0x1ULL << 3)
#define RFM69HCW_AFC_FEI_AFC_AUTO_ON (0x1ULL << 2)
#define RFM69HCW_AFC_FEI_AFC_CLEAR (0x1ULL << 1)
#define RFM69HCW_AFC_FEI_AFC_START (0x1ULL << 0)

#define RFM69HCW_DIO_MAPPING_1_DIO_0_CRC_OK (0b00ULL << 6)
#define RFM69HCW_DIO_MAPPING_1_DIO_0_PAYLOAD_READY (0b01ULL << 6)
#define RFM69HCW_DIO_MAPPING_1_DIO_0_SYNC_ADDRESS (0b10ULL << 6)
#define RFM69HCW_DIO_MAPPING_1_DIO_0_RSSI (0b11ULL << 6)

#define RFM69HCW_DIO_MAPPING_2_CLK_OUT_OFF 0b111

#define RFM69HCW_IRQ_FLAGS_1_MODE_READY (0x1ULL << 7)
#define RFM69HCW_IRQ_FLAGS_1_RX_READY (0x1ULL << 6)
#define RFM69HCW_IRQ_FLAGS_1_TX_READY (0x1ULL << 5)
#define RFM69HCW_IRQ_FLAGS_1_PLL_LOCK (0x1ULL << 4)
#define RFM69HCW_IRQ_FLAGS_1_RSSI (0x1ULL << 3)
#define RFM69HCW_IRQ_FLAGS_1_TIMEOUT (0x1ULL << 2)
#define RFM69HCW_IRQ_FLAGS_1_AUTO_MODE (0x1ULL << 1)
#define RFM69HCW_IRQ_FLAGS_1_SYNC_ADDRESS_MATCH (0x1ULL << 0)

#define RFM69HCW_IRQ_FLAGS_2_FIFO_FULL (0x1ULL << 7)
#define RFM69HCW_IRQ_FLAGS_2_FIFO_NOT_EMPTY (0x1ULL << 6)
#define RFM69HCW_IRQ_FLAGS_2_FIFO_LEVEL (0x1ULL << 5)
#define RFM69HCW_IRQ_FLAGS_2_FIFO_OVERRUN (0x1ULL << 4)
#define RFM69HCW_IRQ_FLAGS_2_PACKET_SENT (0x1ULL << 3)
#define RFM69HCW_IRQ_FLAGS_2_PAYLOAD_READY (0x1ULL << 2)
#define RFM69HCW_IRQ_FLAGS_2_CRC_OK (0x1ULL << 1)

#define RFM69HCW_TEST_DAGC_CONTINUOUS_DAGC 0x30

#define RFM69HCW_SYNC_CONFIG_SYNC_ON (0x1ULL << 7)
#define RFM69HCW_SYNC_CONFIG_FIFO_FILL_CONDITION_1 (0x1ULL << 6)
#define MASK_RFM69HCW_SYNC_CONFIG_SYNC_SIZE 0b111ULL
#define MASK_RFM69HCW_SYNC_CONFIG_SYNC_TOL 0b111ULL
#define SHIFT_RFM69HCW_SYNC_CONFIG_SYNC_SIZE 3
#define SHIFT_RFM69HCW_SYNC_CONFIG_SYNC_TOL 0

// As per spec, `sync_size` is 1 less than the number of sync bytes.
static inline uint8_t MK_RFM69HCW_SYNC_CONFIG(bool sync_on,
                                              bool fifo_fill_condition,
                                              uint8_t sync_size,
                                              uint8_t sync_tol) {
    assert(!(sync_size & ~MASK_RFM69HCW_SYNC_CONFIG_SYNC_SIZE));
    assert(!(sync_tol & ~MASK_RFM69HCW_SYNC_CONFIG_SYNC_TOL));

    uint8_t ret = 0;
    ret |= sync_on ? RFM69HCW_SYNC_CONFIG_SYNC_ON : 0;
    ret |= fifo_fill_condition ? RFM69HCW_SYNC_CONFIG_FIFO_FILL_CONDITION_1 : 0;
    ret |= (sync_size & MASK_RFM69HCW_SYNC_CONFIG_SYNC_SIZE)
           << SHIFT_RFM69HCW_SYNC_CONFIG_SYNC_SIZE;
    ret |= (sync_tol & MASK_RFM69HCW_SYNC_CONFIG_SYNC_TOL)
           << SHIFT_RFM69HCW_SYNC_CONFIG_SYNC_TOL;
    return ret;
}

typedef enum rfm69hcw_data_mode {
    RFM69HCW_DATA_MODE_PACKET_MODE = 0b00,
    RFM69HCW_DATA_MODE_CONTINUOUS_YES_BIT_SYNC = 0b10,
    RFM69HCW_DATA_MODE_CONTINUOUS_NO_BIT_SYNC = 0b11,
} rfm69hcw_data_mode_t;

typedef enum rfm69hcw_modulation_type {
    RFM69HCW_MODULATION_TYPE_FSK = 0b00,
    RFM69HCW_MODULATION_TYPE_OOK = 0b01,
} rfm69hcw_modulation_type_t;

typedef enum rfm69hcw_modulation_shaping_fsk {
    RFM69HCW_MODULATION_SHAPING_FSK_NONE = 0b00,
    RFM69HCW_MODULATION_SHAPING_FSK_GAUSSIAN_BT_1_0 = 0b01,
    RFM69HCW_MODULATION_SHAPING_FSK_GAUSSIAN_BT_0_5 = 0b10,
    RFM69HCW_MODULATION_SHAPING_FSK_GAUSSIAN_BT_0_3 = 0b11,
} rfm69hcw_modulation_shaping_fsk_t;

typedef enum rfm69hcw_modulation_shaping_ook {
    RFM69HCW_MODULATION_SHAPING_OOK_NONE = 0b00,
    RFM69HCW_MODULATION_SHAPING_OOK_FILTERING_CUTOFF_1_BR = 0b01,
    RFM69HCW_MODULATION_SHAPING_OOK_FILTERING_CUTOFF_2_BR = 0b10,
} rfm69hcw_modulation_shaping_ook_t;

#define MASK_RFM69HCW_DATA_MODE 0b111ULL
#define MASK_RFM69HCW_MODULATION_TYPE 0b11ULL
#define MASK_RFM69HCW_SHAPING 0b111ULL
#define SHIFT_RFM69HCW_DATA_MODE 5
#define SHIFT_RFM69HCW_MODULATION_TYPE 3
#define SHIFT_RFM69HCW_MODULATION_SHAPING 0

static inline uint8_t MK_RFM69HCW_DATA_MODUL__INTERNAL(
    rfm69hcw_data_mode_t data_mode, rfm69hcw_modulation_type_t type,
    uint8_t shaping) {
    assert(!(data_mode & ~MASK_RFM69HCW_DATA_MODE));
    assert(!(type & ~MASK_RFM69HCW_MODULATION_TYPE));
    assert(!(shaping & ~MASK_RFM69HCW_SHAPING));

    uint8_t ret = 0;
    ret |= (((uint8_t) data_mode) << SHIFT_RFM69HCW_DATA_MODE);
    ret |= (((uint8_t) type) << SHIFT_RFM69HCW_MODULATION_TYPE);
    ret |= (((uint8_t) shaping) << SHIFT_RFM69HCW_MODULATION_SHAPING);
    return ret;
}

static inline uint8_t MK_RFM69HCW_DATA_MODUL_FSK(
    rfm69hcw_data_mode_t data_mode, rfm69hcw_modulation_type_t type,
    rfm69hcw_modulation_shaping_fsk_t shaping) {
    return MK_RFM69HCW_DATA_MODUL__INTERNAL(data_mode, type, (uint8_t) shaping);
}

static inline uint8_t MK_RFM69HCW_DATA_MODUL_OOK(
    rfm69hcw_data_mode_t data_mode, rfm69hcw_modulation_type_t type,
    rfm69hcw_modulation_shaping_ook_t shaping) {
    return MK_RFM69HCW_DATA_MODUL__INTERNAL(data_mode, type, (uint8_t) shaping);
}

// Assumes `FX_OSC` = 32MHz.
//
// Note that these frequencies as staed are all for FSK; for OOK
// the value in kHz they represent is half of their stated value.
//
// DANGER: as per the spec, the selected bit rate must satisfy
// <bit_rate_freq> < 2 * <rx_bw_freq>.
typedef enum rfm69hcw_rx_bw {
    RFM69HCW_RX_BW_2_6_kHz = 0x0F,
    RFM69HCW_RX_BW_3_91_kHz = 0x07,
    RFM69HCW_RX_BW_5_21_kHz = 0x16,
    RFM69HCW_RX_BW_6_25_kHz = 0x0E,
    RFM69HCW_RX_BW_7_81_kHz = 0x06,
    RFM69HCW_RX_BW_10_4_kHz = 0x15,
    RFM69HCW_RX_BW_12_5_kHz = 0x0D,
    RFM69HCW_RX_BW_15_6_kHz = 0x05,
    RFM69HCW_RX_BW_20_8_kHz = 0x14,
    RFM69HCW_RX_BW_25_0_kHz = 0x0C,
    RFM69HCW_RX_BW_31_3_kHz = 0x04,
    RFM69HCW_RX_BW_41_7_kHz = 0x13,
    RFM69HCW_RX_BW_50_0_kHz = 0x0B,
    RFM69HCW_RX_BW_62_5_kHz = 0x03,
    RFM69HCW_RX_BW_83_3_kHz = 0x12,
    RFM69HCW_RX_BW_100_kHz = 0x0A,
    RFM69HCW_RX_BW_125_kHz = 0x02,
    RFM69HCW_RX_BW_167_kHz = 0x11,
    RFM69HCW_RX_BW_200_kHz = 0x09,
    RFM69HCW_RX_BW_250_kHz = 0x01,
    RFM69HCW_RX_BW_333_kHz = 0x10,
    RFM69HCW_RX_BW_400_kHz = 0x08,
    RFM69HCW_RX_BW_500_kHz = 0x00,
} rfm69hcw_rx_bw_t;

// From the spec: "The default value of DccFreq cutoff frequency is
// typically 4% of the RxBw (channel filter BW). The cutoff frequency
// of the DCC can however be increased to slightly improve the
// sensitivity, under wider modulation conditions.""
typedef enum rfm69hcw_dcc_cutoff {
    RFM69HCW_DCC_CUTOFF_16_PERCENT = 0b000,
    RFM69HCW_DCC_CUTOFF_8_PERCENT = 0b001,
    RFM69HCW_DCC_CUTOFF_4_PERCENT = 0b010,  // Default
    RFM69HCW_DCC_CUTOFF_2_PERCENT = 0b011,
    RFM69HCW_DCC_CUTOFF_1_PERCENT = 0b100,
    RFM69HCW_DCC_CUTOFF_0_5_PERCENT = 0b101,
    RFM69HCW_DCC_CUTOFF_0_25_PERCENT = 0b110,
    RFM69HCW_DCC_CUTOFF_0_125_PERCENT = 0b111,
} rfm69hcw_dcc_cutoff_t;

#define MASK_RFM69HCW_DCC_CUTOFF 0b111ULL
#define MASK_RFM69HCW_RX_BW 0b11111ULL
#define SHIFT_RFM69HCW_DCC_CUTOFF 5
#define SHIFT_RFM69HCW_RX_BW 0

static inline uint8_t MK_RFM69HCW_BW(rfm69hcw_rx_bw_t rx_bw,
                                     rfm69hcw_dcc_cutoff_t dcc_cutoff) {
    assert(!(dcc_cutoff & ~MASK_RFM69HCW_DCC_CUTOFF));
    assert(!(rx_bw & ~MASK_RFM69HCW_RX_BW));

    uint8_t ret = 0;
    ret |= (((uint8_t) dcc_cutoff) << SHIFT_RFM69HCW_DCC_CUTOFF);
    ret |= (((uint8_t) rx_bw) << SHIFT_RFM69HCW_RX_BW);
    return ret;
}

typedef enum rfm69hcw_dc_free {
    RFM69HCW_DC_FREE_NONE = 0b00,
    RFM69HCW_DC_FREE_MANCHESTER = 0b01,
    RFM69HCW_DC_FREE_WHITENING = 0b10,
} rfm69hcw_dc_free_t;

typedef enum rfm69hcw_address_filtering {
    RFM69HCW_ADDRESS_FILTERING_NONE = 0b00,
    RFM69HCW_ADDRESS_FILTERING_MATCH_NODE = 0b01,
    RFM69HCW_ADDRESS_FILTERING_MATCH_NODE_OR_BROADCAST = 0b10,
} rfm69hcw_address_filtering_t;

#define RFM69HCW_PACKET_CONFIG_1_PACKET_FORMAT_VARIABLE (0x1ULL << 7)
#define RFM69HCW_PACKET_CONFIG_1_CRC_ON (0x1ULL << 4)
#define RFM69HCW_PACKET_CONFIG_1_CRC_AUTO_CLEAR_OFF (0x1ULL << 3)
#define SHIFT_RFM69HCW_PACKET_CONFIG_1_DC_FREE 5
#define SHIFT_RFM69HCW_PACKET_CONFIG_1_ADDRESS_FILTERING 1

static inline uint8_t MK_RFM69HCW_PACKET_CONFIG_1(
    bool packet_format_variable, rfm69hcw_dc_free_t dc_free, bool crc_on,
    bool crc_auto_clear_off, rfm69hcw_address_filtering_t address_filtering) {
    uint8_t ret = 0;
    ret |= packet_format_variable
               ? RFM69HCW_PACKET_CONFIG_1_PACKET_FORMAT_VARIABLE
               : 0;
    ret |= ((uint8_t) dc_free) << SHIFT_RFM69HCW_PACKET_CONFIG_1_DC_FREE;
    ret |= crc_on ? RFM69HCW_PACKET_CONFIG_1_CRC_ON : 0;
    ret |= crc_auto_clear_off ? RFM69HCW_PACKET_CONFIG_1_CRC_AUTO_CLEAR_OFF : 0;
    ret |= ((uint8_t) address_filtering)
           << SHIFT_RFM69HCW_PACKET_CONFIG_1_ADDRESS_FILTERING;
    return ret;
}

#define RFM69HCW_PACKET_CONFIG_2_RESTART_RX (0x1ULL << 2)
#define RFM69HCW_PACKET_CONFIG_2_AUTO_RX_RESTART_ON (0x1ULL << 1)
#define RFM69HCW_PACKET_CONFIG_2_AES_ON (0x1ULL << 0)
#define SHIFT_RFM69HCW_PACKET_CONFIG_2_INTER_PACKET_RX_DELAY 4

static inline uint8_t MK_RFM69HCW_PACKET_CONFIG_2(uint8_t inter_packet_rx_delay,
                                                  bool restart_rx,
                                                  bool auto_rx_restart_on,
                                                  bool aes_on) {
    uint8_t ret = 0;
    ret |= (inter_packet_rx_delay & 0b1111ULL)
           << SHIFT_RFM69HCW_PACKET_CONFIG_2_INTER_PACKET_RX_DELAY;
    ret |= restart_rx ? RFM69HCW_PACKET_CONFIG_2_RESTART_RX : 0;
    ret |= auto_rx_restart_on ? RFM69HCW_PACKET_CONFIG_2_AUTO_RX_RESTART_ON : 0;
    ret |= aes_on ? RFM69HCW_PACKET_CONFIG_2_AES_ON : 0;
    return ret;
}

typedef struct rfm69hcw rfm69hcw_t;
typedef rfm69hcw_t* rfm69hcw_handle_t;

// Register the RFM69HCW on the given SPI bus (including managing CS and RST),
// but don't send any traffic yet.
__result_use_check esp_err_t rfm69hcw_init(spi_host_device_t host,
                                           gpio_num_t pin_cs,
                                           gpio_num_t pin_rst,
                                           rfm69hcw_handle_t* out_dev);

// Unregister the RFM69HCW and free the handle;
void rfm69hcw_destroy(rfm69hcw_handle_t dev);

__result_use_check esp_err_t rfm69hcw_reset(rfm69hcw_handle_t dev);
__result_use_check esp_err_t rfm69hcw_reg_read(rfm69hcw_handle_t dev,
                                               rfm69hcw_reg_t addr,
                                               uint8_t* val);
__result_use_check esp_err_t rfm69hcw_reg_write(rfm69hcw_handle_t dev,
                                                rfm69hcw_reg_t addr,
                                                uint8_t val);
__result_use_check esp_err_t rfm69hcw_reg_block_write_fifo(
    rfm69hcw_handle_t dev, const uint8_t* buff, uint8_t len);

typedef enum rfm69hcw_power_level {
    RFM69HCW_POWER_RX_or_PA0,
    RFM69HCW_POWER_TX_NORMAL,
    // Note: Spec requires 1% duty cycle in this mode! (And note that the power
    // amplifiers are running at all times while in TX mode.) Also, MUST be off
    // when in RX mode.
    RFM69HCW_POWER_TX_HIGH,
} rfm69hcw_power_level_t;

// Note zero bytes are illegal sync bytes for the RFM69HCW, and the `sync_value`
// argument must be a zero-terminated byte array of length at most 8.
__result_use_check esp_err_t rfm69hcw_set_sync(rfm69hcw_handle_t dev,
                                               const uint8_t* sync_value,
                                               uint8_t sync_tol);
__result_use_check esp_err_t
rfm69hcw_set_power_level(rfm69hcw_handle_t dev, rfm69hcw_power_level_t level);

typedef struct rfm69hcw_rx_config {
    rfm69hcw_data_mode_t data_mode;
    rfm69hcw_modulation_type_t type;
    union {
        rfm69hcw_modulation_shaping_fsk_t fsk_shaping;
        rfm69hcw_modulation_shaping_ook_t ook_shaping;
    };

    uint32_t freq_khz;
    uint32_t bit_period_ns;
    rfm69hcw_rx_bw_t rx_bw;
    rfm69hcw_dcc_cutoff_t dcc_cutoff;

    uint8_t sync_bit_tol;
    uint8_t sync_value[9];

    uint8_t payload_len;

    // RSSI in dBm below zero in 0.5dBm steps; 1 is -0.5dBm,
    // 2 is -1dBm, etc.
    uint8_t rssi_thresh;

    uint8_t inter_packet_rx_delay;
    uint8_t timeout_rssi_thresh;
} rfm69hcw_rx_config_t;

__result_use_check esp_err_t
rfm69hcw_configure_rx(rfm69hcw_handle_t dev, const rfm69hcw_rx_config_t* cfg);

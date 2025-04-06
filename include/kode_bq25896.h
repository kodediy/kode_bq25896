/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Handle to BQ25896 device
 */
typedef struct bq25896_dev_t* bq25896_handle_t;

/**
 * @brief Initialize the BQ25896 driver
 * 
 * @param i2c_bus I2C bus handle
 * @param handle Pointer to store the device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_init(i2c_master_bus_handle_t i2c_bus, bq25896_handle_t *handle);

/**
 * @brief Delete the BQ25896 driver instance and free resources
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_delete(bq25896_handle_t handle);

/* ####################################################
*                  REGISTER 00h
#################################################### */
/**
 * @brief HIZ mode state
 */
typedef enum {
    BQ25896_HIZ_DISABLE = 0, // High impedance mode disabled
    BQ25896_HIZ_ENABLE = 1   // High impedance mode enabled
} bq25896_hiz_state_t;

/**
 * @brief ILIM pin state
 */
typedef enum {
    BQ25896_ILIM_PIN_DISABLE = 0, // ILIM pin current limit control disabled
    BQ25896_ILIM_PIN_ENABLE = 1   // ILIM pin current limit control enabled
} bq25896_ilim_pin_state_t;

/**
 * @brief Input Current Limit options (in mA) for REG00 [5:0]
 * Range: 100mA to 3250mA with 50mA resolution
 */
typedef enum {
    BQ25896_ILIM_100MA  = 0x00,  // 100mA
    BQ25896_ILIM_150MA  = 0x01,  // 150mA
    BQ25896_ILIM_200MA  = 0x02,  // 200mA
    BQ25896_ILIM_250MA  = 0x03,  // 250mA
    BQ25896_ILIM_300MA  = 0x04,  // 300mA
    BQ25896_ILIM_350MA  = 0x05,  // 350mA
    BQ25896_ILIM_400MA  = 0x06,  // 400mA
    BQ25896_ILIM_450MA  = 0x07,  // 450mA
    BQ25896_ILIM_500MA  = 0x08,  // 500mA (default)
    BQ25896_ILIM_600MA  = 0x0A,  // 600mA
    BQ25896_ILIM_700MA  = 0x0C,  // 700mA
    BQ25896_ILIM_800MA  = 0x0E,  // 800mA
    BQ25896_ILIM_900MA  = 0x10,  // 900mA
    BQ25896_ILIM_1000MA = 0x12,  // 1000mA
    BQ25896_ILIM_1200MA = 0x14,  // 1200mA
    BQ25896_ILIM_1400MA = 0x16,  // 1400mA
    BQ25896_ILIM_1600MA = 0x18,  // 1600mA
    BQ25896_ILIM_1800MA = 0x1A,  // 1800mA
    BQ25896_ILIM_2000MA = 0x1C,  // 2000mA
    BQ25896_ILIM_2200MA = 0x1E,  // 2200mA
    BQ25896_ILIM_2400MA = 0x20,  // 2400mA
    BQ25896_ILIM_2600MA = 0x22,  // 2600mA
    BQ25896_ILIM_2800MA = 0x24,  // 2800mA
    BQ25896_ILIM_3000MA = 0x26,  // 3000mA
    BQ25896_ILIM_3250MA = 0x3F,  // 3250mA (maximum)
} bq25896_ilim_t;

/**
 * @brief Set High Impedance mode (HIZ)
 * When enabled, the input current from the input source is reduced to 0
 * 
 * @param handle Device handle
 * @param state BQ25896_HIZ_ENABLE or BQ25896_HIZ_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_hiz_mode(bq25896_handle_t handle, bq25896_hiz_state_t state);

/**
 * @brief Set ILIM pin current limit control
 * When enabled, the input current limit is determined by ILIM pin
 * When disabled, input current limit is set by I2C register
 * 
 * @param handle Device handle
 * @param state BQ25896_ILIM_PIN_ENABLE or BQ25896_ILIM_PIN_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_ilim_pin(bq25896_handle_t handle, bq25896_ilim_pin_state_t state);

/**
 * @brief Set input current limit using enum value
 * 
 * @param handle Device handle
 * @param ilim Current limit from bq25896_ilim_t enum
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_input_current_limit(bq25896_handle_t handle, bq25896_ilim_t ilim);


/* ####################################################
*                  REGISTER 01h
#################################################### */
/**
 * @brief Boost Mode Hot Temperature Monitor Threshold settings (REG01 [7:6])
 */
typedef enum {
    BQ25896_BHOT_THRESHOLD1 = 0x00,  // VBHOT1 Threshold (34.75%) (default)
    BQ25896_BHOT_THRESHOLD0 = 0x01,  // VBHOT0 Threshold (37.75%)
    BQ25896_BHOT_THRESHOLD2 = 0x02,  // VBHOT2 Threshold (31.25%)
    BQ25896_BHOT_DISABLED   = 0x03,  // Disable boost mode thermal protection
} bq25896_bhot_t;

/**
 * @brief Boost Mode Cold Temperature Monitor Threshold settings (REG01 [5])
 */
typedef enum {
    BQ25896_BCOLD_THRESHOLD0 = 0x00,  // VBCOLD0 Threshold (77%) (default)
    BQ25896_BCOLD_THRESHOLD1 = 0x01,  // VBCOLD1 Threshold (80%)
} bq25896_bcold_t;

/**
 * @brief VINDPM offset values (REG01 [4:0])
 * Range: 0mV to 3100mV in 100mV steps
 */
typedef enum {
    BQ25896_VINDPM_OS_0MV     = 0x00,  // 0mV
    BQ25896_VINDPM_OS_100MV   = 0x01,  // 100mV
    BQ25896_VINDPM_OS_200MV   = 0x02,  // 200mV
    BQ25896_VINDPM_OS_300MV   = 0x03,  // 300mV
    BQ25896_VINDPM_OS_400MV   = 0x04,  // 400mV
    BQ25896_VINDPM_OS_500MV   = 0x05,  // 500mV
    BQ25896_VINDPM_OS_600MV   = 0x06,  // 600mV (default)
    BQ25896_VINDPM_OS_700MV   = 0x07,  // 700mV
    BQ25896_VINDPM_OS_800MV   = 0x08,  // 800mV
    BQ25896_VINDPM_OS_900MV   = 0x09,  // 900mV
    BQ25896_VINDPM_OS_1000MV  = 0x0A,  // 1000mV
    BQ25896_VINDPM_OS_1100MV  = 0x0B,  // 1100mV
    BQ25896_VINDPM_OS_1200MV  = 0x0C,  // 1200mV
    BQ25896_VINDPM_OS_1300MV  = 0x0D,  // 1300mV
    BQ25896_VINDPM_OS_1400MV  = 0x0E,  // 1400mV
    BQ25896_VINDPM_OS_1500MV  = 0x0F,  // 1500mV
    BQ25896_VINDPM_OS_1600MV  = 0x10,  // 1600mV
    BQ25896_VINDPM_OS_1700MV  = 0x11,  // 1700mV
    BQ25896_VINDPM_OS_1800MV  = 0x12,  // 1800mV
    BQ25896_VINDPM_OS_1900MV  = 0x13,  // 1900mV
    BQ25896_VINDPM_OS_2000MV  = 0x14,  // 2000mV
    BQ25896_VINDPM_OS_2100MV  = 0x15,  // 2100mV
    BQ25896_VINDPM_OS_2200MV  = 0x16,  // 2200mV
    BQ25896_VINDPM_OS_2300MV  = 0x17,  // 2300mV
    BQ25896_VINDPM_OS_2400MV  = 0x18,  // 2400mV
    BQ25896_VINDPM_OS_2500MV  = 0x19,  // 2500mV
    BQ25896_VINDPM_OS_2600MV  = 0x1A,  // 2600mV
    BQ25896_VINDPM_OS_2700MV  = 0x1B,  // 2700mV
    BQ25896_VINDPM_OS_2800MV  = 0x1C,  // 2800mV
    BQ25896_VINDPM_OS_2900MV  = 0x1D,  // 2900mV
    BQ25896_VINDPM_OS_3000MV  = 0x1E,  // 3000mV
    BQ25896_VINDPM_OS_3100MV  = 0x1F,  // 3100mV (maximum)
} bq25896_vindpm_os_t;

/**
 * @brief Set boost mode hot temperature monitor threshold
 * 
 * @param handle Device handle
 * @param threshold One of the following:
 *                 BQ25896_BHOT_THRESHOLD1 (34.75%, default)
 *                 BQ25896_BHOT_THRESHOLD0 (37.75%)
 *                 BQ25896_BHOT_THRESHOLD2 (31.25%)
 *                 BQ25896_BHOT_DISABLED (Thermal protection disabled)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_bhot_threshold(bq25896_handle_t handle, bq25896_bhot_t threshold);

/**
 * @brief Set boost mode cold temperature monitor threshold
 * 
 * @param handle Device handle
 * @param threshold One of the following:
 *                  BQ25896_BCOLD_THRESHOLD0 (77%, default)
 *                  BQ25896_BCOLD_THRESHOLD1 (80%)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_bcold_threshold(bq25896_handle_t handle, bq25896_bcold_t threshold);

/**
 * @brief Set input voltage limit offset
 * 
 * This offset is added to the measured VBUS when calculating VINDPM threshold
 * when FORCE_VINDPM = 0 (REG0D[7] = 0). When VBUS at no load is ≤ 6V, this offset
 * is used directly. When VBUS at no load is > 6V, this offset is multiplied by 2.
 * 
 * @param handle Device handle
 * @param offset VINDPM offset from bq25896_vindpm_os_t enum
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vindpm_offset(bq25896_handle_t handle, bq25896_vindpm_os_t offset);


/* ####################################################
*                  REGISTER 02h
#################################################### */
/**
 * @brief ADC conversion control state
 */
typedef enum {
    BQ25896_ADC_CONV_STOP  = 0, // Stop ADC conversion (default)
    BQ25896_ADC_CONV_START = 1  // Start ADC conversion
} bq25896_adc_conv_state_t;

/**
 * @brief ADC Conversion Rate settings
 */
typedef enum {
    BQ25896_ADC_CONV_RATE_ONESHOT = 0x00,  // One shot conversion (default)
    BQ25896_ADC_CONV_RATE_CONTINUOUS = 0x01,  // Continuous conversion
} bq25896_adc_conv_rate_t;

/**
 * @brief Boost Mode Frequency settings
 */
typedef enum {
    BQ25896_BOOST_FREQ_1500KHZ = 0x00,  // 1.5MHz (default)
    BQ25896_BOOST_FREQ_500KHZ  = 0x01,  // 500KHz
} bq25896_boost_freq_t;

/**
 * @brief Input Current Optimizer (ICO) state
 */
typedef enum {
    BQ25896_ICO_DISABLE = 0, // Input Current Optimizer disabled
    BQ25896_ICO_ENABLE = 1   // Input Current Optimizer enabled (default)
} bq25896_ico_state_t;

/**
 * @brief Input Detection Force state
 */
typedef enum {
    BQ25896_FORCE_DPDM_DISABLE = 0, // Not in PSEL detection (default)
    BQ25896_FORCE_DPDM_ENABLE = 1   // Force PSEL detection
} bq25896_force_dpdm_state_t;

/**
 * @brief Automatic Input Detection state
 */
typedef enum {
    BQ25896_AUTO_DPDM_DISABLE = 0, // Disable PSEL detection when VBUS is plugged-in
    BQ25896_AUTO_DPDM_ENABLE = 1   // Enable PSEL detection when VBUS is plugged-in (default)
} bq25896_auto_dpdm_state_t;

/**
 * @brief Set ADC conversion state
 * Starts or stops an ADC conversion
 * Note: This bit is read-only when CONV_RATE = 1
 * 
 * @param handle Device handle
 * @param state BQ25896_ADC_CONV_START or BQ25896_ADC_CONV_STOP
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_adc_conversion(bq25896_handle_t handle, bq25896_adc_conv_state_t state);

/**
 * @brief Set ADC conversion rate
 * Sets one-shot or continuous conversion mode
 * 
 * @param handle Device handle
 * @param rate BQ25896_ADC_CONV_RATE_ONESHOT or BQ25896_ADC_CONV_RATE_CONTINUOUS
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_adc_conversion_rate(bq25896_handle_t handle, bq25896_adc_conv_rate_t rate);

/**
 * @brief Set boost mode frequency
 * Note: Write is ignored when OTG_CONFIG is enabled ----------- TBD, need to check this register before write
 * 
 * @param handle Device handle
 * @param freq BQ25896_BOOST_FREQ_1500KHZ or BQ25896_BOOST_FREQ_500KHZ
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_boost_frequency(bq25896_handle_t handle, bq25896_boost_freq_t freq);

/**
 * @brief Set Input Current Optimizer (ICO) state
 * 
 * @param handle Device handle
 * @param state BQ25896_ICO_ENABLE or BQ25896_ICO_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_ico(bq25896_handle_t handle, bq25896_ico_state_t state);

/**
 * @brief Force input detection (DPDM)
 * 
 * @param handle Device handle
 * @param state BQ25896_FORCE_DPDM_ENABLE or BQ25896_FORCE_DPDM_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_force_dpdm(bq25896_handle_t handle, bq25896_force_dpdm_state_t state);

/**
 * @brief Set automatic input detection (Auto DPDM)
 * 
 * @param handle Device handle
 * @param state BQ25896_AUTO_DPDM_ENABLE or BQ25896_AUTO_DPDM_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_auto_dpdm(bq25896_handle_t handle, bq25896_auto_dpdm_state_t state);


/* ####################################################
*                  REGISTER 03h
#################################################### */
/**
 * @brief Battery Load state
 */
typedef enum {
    BQ25896_BAT_LOAD_DISABLE = 0, // Battery load disabled (default)
    BQ25896_BAT_LOAD_ENABLE = 1   // Battery load enabled
} bq25896_bat_load_state_t;

/**
 * @brief Watchdog Timer Reset state
 */
typedef enum {
    BQ25896_WD_RST_NORMAL = 0, // Normal operation (default)
    BQ25896_WD_RST_RESET = 1   // Reset watchdog timer (auto-clears)
} bq25896_wd_rst_state_t;

/**
 * @brief OTG (Boost) Mode state
 */
typedef enum {
    BQ25896_OTG_DISABLE = 0, // OTG mode disabled (default)
    BQ25896_OTG_ENABLE = 1   // OTG mode enabled
} bq25896_otg_state_t;

/**
 * @brief Charge Enable state
 */
typedef enum {
    BQ25896_CHG_DISABLE = 0, // Charging disabled
    BQ25896_CHG_ENABLE = 1   // Charging enabled (default)
} bq25896_chg_state_t;

/**
 * @brief Minimum System Voltage settings
 */
typedef enum {
    BQ25896_SYS_MIN_3000MV = 0x00, // 3.0V
    BQ25896_SYS_MIN_3100MV = 0x01, // 3.1V
    BQ25896_SYS_MIN_3200MV = 0x02, // 3.2V
    BQ25896_SYS_MIN_3300MV = 0x03, // 3.3V
    BQ25896_SYS_MIN_3400MV = 0x04, // 3.4V
    BQ25896_SYS_MIN_3500MV = 0x05, // 3.5V (default)
    BQ25896_SYS_MIN_3600MV = 0x06, // 3.6V
    BQ25896_SYS_MIN_3700MV = 0x07  // 3.7V
} bq25896_sys_min_t;

/**
 * @brief Minimum Battery Voltage Selection for boost mode exit
 */
typedef enum {
    BQ25896_MIN_VBAT_2900MV = 0, // 2.9V (default)
    BQ25896_MIN_VBAT_2500MV = 1  // 2.5V
} bq25896_min_vbat_sel_t;

/**
 * @brief Enable/disable battery load
 * 
 * @param handle Device handle
 * @param state BQ25896_BAT_LOAD_ENABLE or BQ25896_BAT_LOAD_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_bat_load(bq25896_handle_t handle, bq25896_bat_load_state_t state);

/**
 * @brief Reset the watchdog timer
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_reset_watchdog(bq25896_handle_t handle);

/**
 * @brief Set OTG (boost) mode
 * 
 * @param handle Device handle
 * @param state BQ25896_OTG_ENABLE or BQ25896_OTG_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_otg(bq25896_handle_t handle, bq25896_otg_state_t state);

/**
 * @brief Set charging state
 * 
 * @param handle Device handle
 * @param state BQ25896_CHG_ENABLE or BQ25896_CHG_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_charging(bq25896_handle_t handle, bq25896_chg_state_t state);

/**
 * @brief Set minimum system voltage
 * 
 * @param handle Device handle
 * @param sys_min Minimum system voltage from bq25896_sys_min_t enum
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_sys_min(bq25896_handle_t handle, bq25896_sys_min_t sys_min);

/**
 * @brief Set minimum battery voltage for boost mode exit
 * 
 * @param handle Device handle
 * @param vbat_sel BQ25896_MIN_VBAT_2900MV or BQ25896_MIN_VBAT_2500MV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_min_vbat(bq25896_handle_t handle, bq25896_min_vbat_sel_t vbat_sel);


/* ####################################################
*                  REGISTER 04h
#################################################### */
/**
 * @brief Current Pulse Control (PUMPX) state
 */
typedef enum {
    BQ25896_PUMPX_DISABLE = 0, // Disable current pulse control (default)
    BQ25896_PUMPX_ENABLE = 1   // Enable current pulse control
} bq25896_pumpx_state_t;

/**
 * @brief Fast Charge Current Limit settings
 * Note: Values represent current in mA divided by 64
 */
typedef enum {
    BQ25896_ICHG_0MA     = 0x00,  // 0mA (disables charging)
    BQ25896_ICHG_64MA    = 0x01,  // 64mA
    BQ25896_ICHG_128MA   = 0x02,  // 128mA
    BQ25896_ICHG_512MA   = 0x08,  // 512mA
    BQ25896_ICHG_1024MA  = 0x10,  // 1024mA
    BQ25896_ICHG_2048MA  = 0x20,  // 2048mA (default)
    BQ25896_ICHG_3008MA  = 0x2F,  // 3008mA (maximum)
} bq25896_ichg_t;

/**
 * @brief Set current pulse control (PUMPX) state
 * 
 * @param handle Device handle
 * @param state BQ25896_PUMPX_ENABLE or BQ25896_PUMPX_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_pumpx(bq25896_handle_t handle, bq25896_pumpx_state_t state);

/**
 * @brief Set fast charge current limit
 * 
 * @param handle Device handle
 * @param ichg Fast charge current limit from bq25896_ichg_t enum
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_charge_current(bq25896_handle_t handle, bq25896_ichg_t ichg);


/* ####################################################
*                  REGISTER 05h
#################################################### */
/**
 * @brief Precharge Current Limit settings (REG05 [7:4])
 * Value = (enum value + 1) × 64mA
 */
typedef enum {
    BQ25896_PRECHG_64MA    = 0x0,  // 64mA (minimum)
    BQ25896_PRECHG_128MA   = 0x1,  // 128mA (default)
    BQ25896_PRECHG_192MA   = 0x2,  // 192mA
    BQ25896_PRECHG_256MA   = 0x3,  // 256mA
    BQ25896_PRECHG_320MA   = 0x4,  // 320mA
    BQ25896_PRECHG_384MA   = 0x5,  // 384mA
    BQ25896_PRECHG_448MA   = 0x6,  // 448mA
    BQ25896_PRECHG_512MA   = 0x7,  // 512mA
    BQ25896_PRECHG_576MA   = 0x8,  // 576mA
    BQ25896_PRECHG_640MA   = 0x9,  // 640mA
    BQ25896_PRECHG_704MA   = 0xA,  // 704mA
    BQ25896_PRECHG_768MA   = 0xB,  // 768mA
    BQ25896_PRECHG_832MA   = 0xC,  // 832mA
    BQ25896_PRECHG_896MA   = 0xD,  // 896mA
    BQ25896_PRECHG_960MA   = 0xE,  // 960mA
    BQ25896_PRECHG_1024MA  = 0xF,  // 1024mA (maximum)
} bq25896_prechg_current_t;

/**
 * @brief Termination Current Limit settings (REG05 [3:0])
 * Value = (enum value + 1) × 64mA
 */
typedef enum {
    BQ25896_ITERM_64MA    = 0x0,  // 64mA (minimum)
    BQ25896_ITERM_128MA   = 0x1,  // 128mA
    BQ25896_ITERM_192MA   = 0x2,  // 192mA
    BQ25896_ITERM_256MA   = 0x3,  // 256mA (default)
    BQ25896_ITERM_320MA   = 0x4,  // 320mA
    BQ25896_ITERM_384MA   = 0x5,  // 384mA
    BQ25896_ITERM_448MA   = 0x6,  // 448mA
    BQ25896_ITERM_512MA   = 0x7,  // 512mA
    BQ25896_ITERM_576MA   = 0x8,  // 576mA
    BQ25896_ITERM_640MA   = 0x9,  // 640mA
    BQ25896_ITERM_704MA   = 0xA,  // 704mA
    BQ25896_ITERM_768MA   = 0xB,  // 768mA
    BQ25896_ITERM_832MA   = 0xC,  // 832mA
    BQ25896_ITERM_896MA   = 0xD,  // 896mA
    BQ25896_ITERM_960MA   = 0xE,  // 960mA
    BQ25896_ITERM_1024MA  = 0xF,  // 1024mA (maximum)
} bq25896_iterm_current_t;

/**
 * @brief Set precharge current limit
 * Sets the battery charge current during precharge phase (when battery voltage is below BATLOWV)
 * 
 * @param handle Device handle
 * @param current Precharge current setting from bq25896_prechg_current_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_precharge_current(bq25896_handle_t handle, bq25896_prechg_current_t current);

/**
 * @brief Set termination current limit
 * Sets the current threshold for charge termination
 * 
 * @param handle Device handle
 * @param current Termination current setting from bq25896_iterm_current_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_termination_current(bq25896_handle_t handle, bq25896_iterm_current_t current);


/* ####################################################
*                  REGISTER 06h
#################################################### */
/**
 * @brief Charge Voltage Limit settings (REG06 [7:2])
 * Value = 3840mV + (enum value × 16mV)
 */
typedef enum {
    BQ25896_VREG_3840MV = 0x00,  // 3.840V (minimum)
    BQ25896_VREG_3856MV = 0x01,  // 3.856V
    BQ25896_VREG_3872MV = 0x02,  // 3.872V
    BQ25896_VREG_3888MV = 0x03,  // 3.888V
    BQ25896_VREG_3904MV = 0x04,  // 3.904V
    BQ25896_VREG_3920MV = 0x05,  // 3.920V
    BQ25896_VREG_3936MV = 0x06,  // 3.936V
    BQ25896_VREG_3952MV = 0x07,  // 3.952V
    BQ25896_VREG_3968MV = 0x08,  // 3.968V
    BQ25896_VREG_3984MV = 0x09,  // 3.984V
    BQ25896_VREG_4000MV = 0x0A,  // 4.000V
    BQ25896_VREG_4016MV = 0x0B,  // 4.016V
    BQ25896_VREG_4032MV = 0x0C,  // 4.032V
    BQ25896_VREG_4048MV = 0x0D,  // 4.048V
    BQ25896_VREG_4064MV = 0x0E,  // 4.064V
    BQ25896_VREG_4080MV = 0x0F,  // 4.080V
    BQ25896_VREG_4096MV = 0x10,  // 4.096V
    BQ25896_VREG_4112MV = 0x11,  // 4.112V
    BQ25896_VREG_4128MV = 0x12,  // 4.128V
    BQ25896_VREG_4144MV = 0x13,  // 4.144V
    BQ25896_VREG_4160MV = 0x14,  // 4.160V
    BQ25896_VREG_4176MV = 0x15,  // 4.176V
    BQ25896_VREG_4192MV = 0x16,  // 4.192V
    BQ25896_VREG_4208MV = 0x17,  // 4.208V (default)
    BQ25896_VREG_4224MV = 0x18,  // 4.224V
    BQ25896_VREG_4240MV = 0x19,  // 4.240V
    BQ25896_VREG_4256MV = 0x1A,  // 4.256V
    BQ25896_VREG_4272MV = 0x1B,  // 4.272V
    BQ25896_VREG_4288MV = 0x1C,  // 4.288V
    BQ25896_VREG_4304MV = 0x1D,  // 4.304V
    BQ25896_VREG_4320MV = 0x1E,  // 4.320V
    BQ25896_VREG_4336MV = 0x1F,  // 4.336V
    BQ25896_VREG_4352MV = 0x20,  // 4.352V
    BQ25896_VREG_4368MV = 0x21,  // 4.368V
    BQ25896_VREG_4384MV = 0x22,  // 4.384V
    BQ25896_VREG_4400MV = 0x23,  // 4.400V
    BQ25896_VREG_4416MV = 0x24,  // 4.416V
    BQ25896_VREG_4432MV = 0x25,  // 4.432V
    BQ25896_VREG_4448MV = 0x26,  // 4.448V
    BQ25896_VREG_4464MV = 0x27,  // 4.464V
    BQ25896_VREG_4480MV = 0x28,  // 4.480V
    BQ25896_VREG_4496MV = 0x29,  // 4.496V
    BQ25896_VREG_4512MV = 0x2A,  // 4.512V
    BQ25896_VREG_4528MV = 0x2B,  // 4.528V
    BQ25896_VREG_4544MV = 0x2C,  // 4.544V
    BQ25896_VREG_4560MV = 0x2D,  // 4.560V
    BQ25896_VREG_4576MV = 0x2E,  // 4.576V
    BQ25896_VREG_4592MV = 0x2F,  // 4.592V
    BQ25896_VREG_4608MV = 0x30   // 4.608V (maximum)
} bq25896_vreg_t;

/**
 * @brief Battery Precharge to Fast Charge Threshold settings (REG06 [1])
 */
typedef enum {
    BQ25896_BATLOWV_2800MV = 0,  // 2.8V threshold
    BQ25896_BATLOWV_3000MV = 1   // 3.0V threshold (default)
} bq25896_batlowv_t;

/**
 * @brief Battery Recharge Threshold Offset settings (REG06 [0])
 */
typedef enum {
    BQ25896_VRECHG_100MV = 0,  // 100mV below VREG (default)
    BQ25896_VRECHG_200MV = 1   // 200mV below VREG
} bq25896_vrechg_t;

/**
 * @brief Set charge voltage limit
 * Sets the battery charging voltage limit
 * 
 * @param handle Device handle
 * @param vreg Charge voltage setting from bq25896_vreg_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_charge_voltage(bq25896_handle_t handle, bq25896_vreg_t vreg);

/**
 * @brief Set battery precharge to fast charge threshold
 * 
 * @param handle Device handle
 * @param threshold BQ25896_BATLOWV_2800MV or BQ25896_BATLOWV_3000MV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_batlowv(bq25896_handle_t handle, bq25896_batlowv_t threshold);

/**
 * @brief Set battery recharge threshold offset
 * 
 * @param handle Device handle
 * @param vrechg BQ25896_VRECHG_100MV or BQ25896_VRECHG_200MV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vrechg(bq25896_handle_t handle, bq25896_vrechg_t vrechg);


/* ####################################################
*                  REGISTER 07h
#################################################### */
/**
 * @brief Charging Termination Enable settings (REG07 [7])
 */
typedef enum {
    BQ25896_TERM_DISABLE = 0,  // Disable termination
    BQ25896_TERM_ENABLE = 1    // Enable termination (default)
} bq25896_term_state_t;

/**
 * @brief STAT Pin Disable settings (REG07 [6])
 */
typedef enum {
    BQ25896_STAT_ENABLE = 0,   // Enable STAT pin function (default)
    BQ25896_STAT_DISABLE = 1   // Disable STAT pin function
} bq25896_stat_pin_state_t;

/**
 * @brief I2C Watchdog Timer settings (REG07 [5:4])
 */
typedef enum {
    BQ25896_WATCHDOG_DISABLE = 0x0,  // Disable watchdog timer
    BQ25896_WATCHDOG_40S = 0x1,      // 40 seconds (default)
    BQ25896_WATCHDOG_80S = 0x2,      // 80 seconds
    BQ25896_WATCHDOG_160S = 0x3      // 160 seconds
} bq25896_watchdog_t;

/**
 * @brief Charging Safety Timer Enable settings (REG07 [3])
 */
typedef enum {
    BQ25896_SAFETY_TIMER_DISABLE = 0,  // Disable safety timer
    BQ25896_SAFETY_TIMER_ENABLE = 1    // Enable safety timer (default)
} bq25896_safety_timer_state_t;

/**
 * @brief Fast Charge Timer settings (REG07 [2:1])
 */
typedef enum {
    BQ25896_CHG_TIMER_5H = 0x0,   // 5 hours
    BQ25896_CHG_TIMER_8H = 0x1,   // 8 hours
    BQ25896_CHG_TIMER_12H = 0x2,  // 12 hours (default)
    BQ25896_CHG_TIMER_20H = 0x3   // 20 hours
} bq25896_chg_timer_t;

/**
 * @brief JEITA Low Temperature Current settings (REG07 [0])
 */
typedef enum {
    BQ25896_JEITA_ISET_50PCT = 0,  // 50% of ICHG
    BQ25896_JEITA_ISET_20PCT = 1   // 20% of ICHG (default)
} bq25896_jeita_iset_t;

/**
 * @brief Set charging termination state
 * 
 * @param handle Device handle
 * @param state BQ25896_TERM_ENABLE or BQ25896_TERM_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_termination_state(bq25896_handle_t handle, bq25896_term_state_t state);

/**
 * @brief Set STAT pin state
 * 
 * @param handle Device handle
 * @param state BQ25896_STAT_ENABLE or BQ25896_STAT_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_stat_pin_state(bq25896_handle_t handle, bq25896_stat_pin_state_t state);

/**
 * @brief Set I2C watchdog timer
 * 
 * @param handle Device handle
 * @param watchdog Watchdog setting from bq25896_watchdog_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_watchdog_timer(bq25896_handle_t handle, bq25896_watchdog_t watchdog);

/**
 * @brief Set safety timer state
 * 
 * @param handle Device handle
 * @param state BQ25896_SAFETY_TIMER_ENABLE or BQ25896_SAFETY_TIMER_DISABLE
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_safety_timer_state(bq25896_handle_t handle, bq25896_safety_timer_state_t state);

/**
 * @brief Set fast charge timer duration
 * 
 * @param handle Device handle
 * @param timer Fast charge timer setting from bq25896_chg_timer_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_charge_timer(bq25896_handle_t handle, bq25896_chg_timer_t timer);

/**
 * @brief Set JEITA low temperature current percentage
 * 
 * @param handle Device handle
 * @param iset BQ25896_JEITA_ISET_50PCT or BQ25896_JEITA_ISET_20PCT
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_jeita_iset(bq25896_handle_t handle, bq25896_jeita_iset_t iset);


/* ####################################################
*                  REGISTER 08h
#################################################### */
/**
 * @brief IR Compensation Resistor settings (REG08 [7:5])
 */
typedef enum {
    BQ25896_BAT_COMP_0MO     = 0x0,  // 0mΩ (default, disabled)
    BQ25896_BAT_COMP_20MO    = 0x1,  // 20mΩ
    BQ25896_BAT_COMP_40MO    = 0x2,  // 40mΩ
    BQ25896_BAT_COMP_60MO    = 0x3,  // 60mΩ
    BQ25896_BAT_COMP_80MO    = 0x4,  // 80mΩ
    BQ25896_BAT_COMP_100MO   = 0x5,  // 100mΩ
    BQ25896_BAT_COMP_120MO   = 0x6,  // 120mΩ
    BQ25896_BAT_COMP_140MO   = 0x7   // 140mΩ
} bq25896_bat_comp_t;

/**
 * @brief IR Compensation Voltage Clamp settings (REG08 [4:2])
 */
typedef enum {
    BQ25896_VCLAMP_0MV     = 0x0,  // 0mV (default)
    BQ25896_VCLAMP_32MV    = 0x1,  // 32mV
    BQ25896_VCLAMP_64MV    = 0x2,  // 64mV
    BQ25896_VCLAMP_96MV    = 0x3,  // 96mV
    BQ25896_VCLAMP_128MV   = 0x4,  // 128mV
    BQ25896_VCLAMP_160MV   = 0x5,  // 160mV
    BQ25896_VCLAMP_192MV   = 0x6,  // 192mV
    BQ25896_VCLAMP_224MV   = 0x7   // 224mV
} bq25896_vclamp_t;

/**
 * @brief Thermal Regulation Threshold settings (REG08 [1:0])
 */
typedef enum {
    BQ25896_TREG_60C     = 0x0,  // 60°C
    BQ25896_TREG_80C     = 0x1,  // 80°C
    BQ25896_TREG_100C    = 0x2,  // 100°C
    BQ25896_TREG_120C    = 0x3   // 120°C (default)
} bq25896_treg_t;

/**
 * @brief Set IR compensation resistor value
 * 
 * @param handle Device handle
 * @param bat_comp IR compensation resistor setting from bq25896_bat_comp_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_bat_comp(bq25896_handle_t handle, bq25896_bat_comp_t bat_comp);

/**
 * @brief Set IR compensation voltage clamp
 * 
 * @param handle Device handle
 * @param vclamp IR compensation voltage clamp setting from bq25896_vclamp_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vclamp(bq25896_handle_t handle, bq25896_vclamp_t vclamp);

/**
 * @brief Set thermal regulation threshold
 * 
 * @param handle Device handle
 * @param treg Thermal regulation threshold setting from bq25896_treg_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_treg(bq25896_handle_t handle, bq25896_treg_t treg);


/* ####################################################
*                  REGISTER 09h
#################################################### */
/**
 * @brief Force Start Input Current Optimizer (ICO)
 */
typedef enum {
    BQ25896_ICO_NOT_FORCE = 0,  // Not force start Input Current Optimizer (ICO)
    BQ25896_ICO_FORCE = 1    // Force start Input Current Optimizer (ICO)
} bq25896_ico_force_start_t;

/**
 * @brief Safety Timer Extension Setting (REG09 [6])
 */
typedef enum {
    BQ25896_TMR2X_DISABLE = 0,  // Safety timer not slowed during DPM/thermal regulation
    BQ25896_TMR2X_ENABLE = 1    // Safety timer slowed by 2X during DPM/thermal regulation (default)
} bq25896_tmr2x_t;

/**
 * @brief BATFET Control (REG09 [5])
 */
typedef enum {
    BQ25896_BATFET_ENABLE = 0,  // Allow BATFET turn on (default)
    BQ25896_BATFET_DISABLE = 1  // Force BATFET off (ship mode)
} bq25896_batfet_state_t;

/**
 * @brief JEITA High Temperature Voltage Setting (REG09 [4])
 */
typedef enum {
    BQ25896_JEITA_VSET_REDUCED = 0,  // Set Charge Voltage to VREG-200mV during JEITA high temperature (default)
    BQ25896_JEITA_VSET_NORMAL = 1    // Set Charge Voltage to VREG during JEITA high temperature
} bq25896_jeita_vset_t;

/**
 * @brief BATFET Turn Off Delay Control (REG09 [3])
 */
typedef enum {
    BQ25896_BATFET_DLY_DISABLE = 0,  // BATFET turn off immediately when BATFET_DIS bit is set (default)
    BQ25896_BATFET_DLY_ENABLE = 1    // BATFET turn off delay by tsm_dly when BATFET_DIS bit is set
} bq25896_batfet_dly_t;

/**
 * @brief BATFET Full System Reset Enable (REG09 [2])
 */
typedef enum {
    BQ25896_BATFET_RST_DISABLE = 0,  // Disable BATFET full system reset
    BQ25896_BATFET_RST_ENABLE = 1    // Enable BATFET full system reset (default)
} bq25896_batfet_rst_t;

/**
 * @brief Force start Input Current Optimizer (ICO)
 * Note: This bit automatically clears after ICO starts
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_force_ico(bq25896_handle_t handle);

/**
 * @brief Set safety timer extension during DPM or thermal regulation
 * 
 * @param handle Device handle
 * @param state Timer extension state from bq25896_tmr2x_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_timer_extension(bq25896_handle_t handle, bq25896_tmr2x_t state);

/**
 * @brief Control BATFET state (battery connection)
 * 
 * @param handle Device handle
 * @param state BATFET state from bq25896_batfet_state_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_batfet_state(bq25896_handle_t handle, bq25896_batfet_state_t state);

/**
 * @brief Set JEITA high temperature voltage setting
 * 
 * @param handle Device handle
 * @param vset JEITA voltage setting from bq25896_jeita_vset_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_jeita_vset(bq25896_handle_t handle, bq25896_jeita_vset_t vset);

/**
 * @brief Set BATFET turn off delay control
 * 
 * @param handle Device handle
 * @param delay BATFET delay setting from bq25896_batfet_dly_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_batfet_delay(bq25896_handle_t handle, bq25896_batfet_dly_t delay);

/**
 * @brief Set BATFET full system reset enable
 * 
 * @param handle Device handle
 * @param reset BATFET reset setting from bq25896_batfet_rst_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_batfet_reset(bq25896_handle_t handle, bq25896_batfet_rst_t reset);

/**
 * @brief Trigger current pulse control voltage up
 * Note: This bit automatically clears after sequence completes
 * Note: EN_PUMPX must be set in REG04 before calling this function
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_pumpx_up(bq25896_handle_t handle);

/**
 * @brief Trigger current pulse control voltage down
 * Note: This bit automatically clears after sequence completes
 * Note: EN_PUMPX must be set in REG04 before calling this function
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_pumpx_down(bq25896_handle_t handle);


/* ####################################################
*                  REGISTER 0Ah
#################################################### */
/**
 * @brief Boost Mode Voltage Regulation settings (REG0A [7:4])
 * Note: Voltage = 4.55V + (enum value * 64mV)
 */
typedef enum {
    BQ25896_BOOSTV_4614MV = 0x0,  // 4.614V (4.55V + 64mV)
    BQ25896_BOOSTV_4678MV = 0x1,  // 4.678V (4.55V + 128mV)
    BQ25896_BOOSTV_4742MV = 0x2,  // 4.742V (4.55V + 192mV)
    BQ25896_BOOSTV_4806MV = 0x3,  // 4.806V (4.55V + 256mV)
    BQ25896_BOOSTV_4870MV = 0x4,  // 4.870V (4.55V + 320mV)
    BQ25896_BOOSTV_4934MV = 0x5,  // 4.934V (4.55V + 384mV)
    BQ25896_BOOSTV_4998MV = 0x7,  // 4.998V (4.55V + 448mV) (default)
    BQ25896_BOOSTV_5062MV = 0x8,  // 5.062V (4.55V + 512mV)
    BQ25896_BOOSTV_5126MV = 0x9,  // 5.126V (4.55V + 576mV)
    BQ25896_BOOSTV_5190MV = 0xA,  // 5.190V (4.55V + 640mV)
    BQ25896_BOOSTV_5254MV = 0xB,  // 5.254V (4.55V + 704mV)
    BQ25896_BOOSTV_5318MV = 0xC,  // 5.318V (4.55V + 768mV)
    BQ25896_BOOSTV_5382MV = 0xD,  // 5.382V (4.55V + 832mV)
    BQ25896_BOOSTV_5446MV = 0xE,  // 5.446V (4.55V + 896mV)
    BQ25896_BOOSTV_5510MV = 0xF   // 5.510V (4.55V + 960mV)
} bq25896_boostv_t;

/**
 * @brief PFM mode in boost mode settings (REG0A [3])
 */
typedef enum {
    BQ25896_PFM_BOOST_ALLOW = 0,  // Allow PFM in boost mode (default)
    BQ25896_PFM_BOOST_DISABLE = 1 // Disable PFM in boost mode
} bq25896_pfm_boost_t;

/**
 * @brief Boost Mode Current Limit settings (REG0A [2:0])
 */
typedef enum {
    BQ25896_BOOST_LIM_500MA = 0x0,    // 0.5A
    BQ25896_BOOST_LIM_750MA = 0x1,    // 0.75A
    BQ25896_BOOST_LIM_1200MA = 0x2,   // 1.2A
    BQ25896_BOOST_LIM_1400MA = 0x3,   // 1.4A (default)
    BQ25896_BOOST_LIM_1650MA = 0x4,   // 1.65A
    BQ25896_BOOST_LIM_1875MA = 0x5,   // 1.875A
    BQ25896_BOOST_LIM_2150MA = 0x6    // 2.15A
    // 0x7 is Reserved
} bq25896_boost_lim_t;

/**
 * @brief Set boost mode output voltage
 * 
 * @param handle Device handle
 * @param boostv Boost voltage setting from bq25896_boostv_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_boost_voltage(bq25896_handle_t handle, bq25896_boostv_t boostv);

/**
 * @brief Set PFM mode in boost mode
 * 
 * @param handle Device handle
 * @param pfm_state PFM mode setting from bq25896_pfm_boost_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_pfm_boost_mode(bq25896_handle_t handle, bq25896_pfm_boost_t pfm_state);

/**
 * @brief Set boost mode current limit
 * 
 * @param handle Device handle
 * @param boost_lim Current limit setting from bq25896_boost_lim_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_boost_current_limit(bq25896_handle_t handle, bq25896_boost_lim_t boost_lim);


/* ####################################################
*                  REGISTER 0Bh
#################################################### */
/**
 * @brief VBUS Status values (REG0B [7:5])
 */
typedef enum {
    BQ25896_VBUS_STAT_NO_INPUT    = 0x0,  // No Input
    BQ25896_VBUS_STAT_USB_HOST    = 0x1,  // USB Host SDP
    BQ25896_VBUS_STAT_ADAPTER     = 0x2,  // Adapter (3.25A)
    BQ25896_VBUS_STAT_OTG         = 0x7   // OTG
} bq25896_vbus_stat_t;

/**
 * @brief Charging Status values (REG0B [4:3])
 */
typedef enum {
    BQ25896_CHRG_STAT_NOT_CHARGING     = 0x0,  // Not Charging
    BQ25896_CHRG_STAT_PRE_CHARGE       = 0x1,  // Pre-charge (< VBATLOWV)
    BQ25896_CHRG_STAT_FAST_CHARGING    = 0x2,  // Fast Charging
    BQ25896_CHRG_STAT_TERM_DONE        = 0x3   // Charge Termination Done
} bq25896_chrg_stat_t;

/**
 * @brief Power Good Status values (REG0B [2])
 */
typedef enum {
    BQ25896_PG_STAT_NOT_GOOD  = 0x0,  // Not Power Good
    BQ25896_PG_STAT_GOOD      = 0x1   // Power Good
} bq25896_pg_stat_t;

/**
 * @brief VSYS Regulation Status values (REG0B [0])
 */
typedef enum {
    BQ25896_VSYS_STAT_NOT_IN_REG   = 0x0,  // Not in VSYSMIN regulation (BAT > VSYSMIN)
    BQ25896_VSYS_STAT_IN_REG       = 0x1   // In VSYSMIN regulation (BAT < VSYSMIN)
} bq25896_vsys_stat_t;

/**
 * @brief Read VBUS status
 * 
 * @param handle Device handle
 * @param vbus_stat Pointer to store VBUS status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vbus_status(bq25896_handle_t handle, bq25896_vbus_stat_t *vbus_stat);

/**
 * @brief Read charging status
 * 
 * @param handle Device handle
 * @param chrg_stat Pointer to store charging status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charging_status(bq25896_handle_t handle, bq25896_chrg_stat_t *chrg_stat);

/**
 * @brief Read power good status
 * 
 * @param handle Device handle
 * @param pg_stat Pointer to store power good status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_pg_status(bq25896_handle_t handle, bq25896_pg_stat_t *pg_stat);

/**
 * @brief Read VSYS regulation status
 * 
 * @param handle Device handle
 * @param vsys_stat Pointer to store VSYS regulation status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vsys_status(bq25896_handle_t handle, bq25896_vsys_stat_t *vsys_stat);


/* ####################################################
*                  REGISTER 0Ch
#################################################### */
/**
 * @brief Watchdog Fault Status values (REG0C [7])
 */
typedef enum {
    BQ25896_WD_FAULT_NORMAL = 0,    // Normal
    BQ25896_WD_FAULT_EXPIRED = 1    // Watchdog timer expiration
} bq25896_watchdog_fault_t;

/**
 * @brief Boost Mode Fault Status values (REG0C [6])
 */
typedef enum {
    BQ25896_BOOST_FAULT_NORMAL = 0,  // Normal
    BQ25896_BOOST_FAULT_FAULT = 1    // VBUS overloaded in OTG, or VBUS OVP, or battery too low
} bq25896_boost_fault_t;

/**
 * @brief Charge Fault Status values (REG0C [5:4])
 */
typedef enum {
    BQ25896_CHRG_FAULT_NORMAL = 0,         // Normal
    BQ25896_CHRG_FAULT_INPUT_FAULT = 1,    // Input fault (VBUS > VACOV or VBAT < VBUS < VVBUSMIN)
    BQ25896_CHRG_FAULT_THERMAL = 2,        // Thermal shutdown
    BQ25896_CHRG_FAULT_TIMER_EXPIRED = 3   // Charge Safety Timer Expiration
} bq25896_chrg_fault_t;

/**
 * @brief Battery Fault Status values (REG0C [3])
 */
typedef enum {
    BQ25896_BAT_FAULT_NORMAL = 0,    // Normal
    BQ25896_BAT_FAULT_OVERVOLTAGE = 1 // BATOVP (VBAT > VBATOVP)
} bq25896_bat_fault_t;

/**
 * @brief NTC Fault Status values (REG0C [2:0])
 */
typedef enum {
    BQ25896_NTC_FAULT_NORMAL = 0,    // Normal
    BQ25896_NTC_FAULT_TS_WARM = 2,   // TS Warm (Buck mode only)
    BQ25896_NTC_FAULT_TS_COOL = 3,   // TS Cool (Buck mode only)
    BQ25896_NTC_FAULT_TS_COLD = 5,   // TS Cold
    BQ25896_NTC_FAULT_TS_HOT = 6     // TS Hot
} bq25896_ntc_fault_t;

/**
 * @brief Read watchdog fault status
 * 
 * @param handle Device handle
 * @param fault Pointer to store watchdog fault status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_watchdog_fault(bq25896_handle_t handle, bq25896_watchdog_fault_t *fault);

/**
 * @brief Read boost mode fault status
 * 
 * @param handle Device handle
 * @param fault Pointer to store boost mode fault status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_boost_fault(bq25896_handle_t handle, bq25896_boost_fault_t *fault);

/**
 * @brief Read charge fault status
 * 
 * @param handle Device handle
 * @param fault Pointer to store charge fault status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charge_fault(bq25896_handle_t handle, bq25896_chrg_fault_t *fault);

/**
 * @brief Read battery fault status
 * 
 * @param handle Device handle
 * @param fault Pointer to store battery fault status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_battery_fault(bq25896_handle_t handle, bq25896_bat_fault_t *fault);

/**
 * @brief Read NTC fault status
 * 
 * @param handle Device handle
 * @param fault Pointer to store NTC fault status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ntc_fault(bq25896_handle_t handle, bq25896_ntc_fault_t *fault);


/* ####################################################
*                  REGISTER 0Dh
#################################################### */
/**
 * @brief Set VINDPM threshold setting method   TBD -- MODIFY WHEN INPUT SOURCE IS PLUGGED IN
 * 
 * @param handle Device handle
 * @param mode VINDPM threshold setting method from bq25896_force_vindpm_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
typedef enum {
    BQ25896_VINDPM_RELATIVE = 0,  // Run Relative VINDPM Threshold (default)
    BQ25896_VINDPM_ABSOLUTE = 1   // Run Absolute VINDPM Threshold
} bq25896_force_vindpm_t;

/**
 * @brief Set VINDPM threshold setting method   TBD -- MODIFY WHEN INPUT SOURCE IS PLUGGED IN
 * 
 * @param handle Device handle
 * @param mode VINDPM threshold setting method from bq25896_force_vindpm_t
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vindpm_mode(bq25896_handle_t handle, bq25896_force_vindpm_t mode);

/**
 * @brief Set absolute VINDPM threshold --- TBD -- MODIFY WHEN INPUT SOURCE IS PLUGGED IN
 * This function should only be called when FORCE_VINDPM = 1 (Absolute mode)
 * 
 * @param handle Device handle
 * @param threshold_mv Absolute VINDPM threshold in mV (3900-15300mV)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_absolute_vindpm(bq25896_handle_t handle, uint16_t threshold_mv);


/* ####################################################
*                  REGISTER 0Eh
#################################################### */
/**
 * @brief Thermal Regulation Status values (REG0E [7])
 */
typedef enum {
    BQ25896_THERM_STAT_NORMAL = 0,     // Normal
    BQ25896_THERM_STAT_ACTIVE = 1      // In Thermal Regulation
} bq25896_therm_stat_t;

/**
 * @brief Read thermal regulation status
 * 
 * @param handle Device handle
 * @param therm_stat Pointer to store thermal regulation status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_thermal_regulation_status(bq25896_handle_t handle, bq25896_therm_stat_t *therm_stat);

/**
 * @brief Read battery voltage
 * Reads the ADC conversion of battery voltage from the device
 * 
 * @param handle Device handle
 * @param voltage_mv Pointer to store battery voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_battery_voltage(bq25896_handle_t handle, uint16_t *voltage_mv);


/* ####################################################
*                  REGISTER 0Fh
#################################################### */
/**
 * @brief Read system voltage
 * Reads the ADC conversion of system voltage from the device
 * 
 * @param handle Device handle
 * @param voltage_mv Pointer to store system voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_system_voltage(bq25896_handle_t handle, uint16_t *voltage_mv);


/* ####################################################
*                  REGISTER 10h
#################################################### */
/**
 * @brief Read TS voltage percentage
 * Reads the ADC conversion of TS voltage as percentage of REGN
 * 
 * @param handle Device handle
 * @param ts_percentage Pointer to store TS voltage percentage (21.0-80.0%)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ts_voltage_percentage(bq25896_handle_t handle, float *ts_percentage);


/* ####################################################
*                  REGISTER 11h
#################################################### */
/**
 * @brief VBUS Good Status values (REG11 [7])
 */
typedef enum {
    BQ25896_VBUS_NOT_ATTACHED = 0,  // Not VBUS attached
    BQ25896_VBUS_ATTACHED = 1       // VBUS Attached
} bq25896_vbus_gd_t;

/**
 * @brief Read VBUS good status
 * 
 * @param handle Device handle
 * @param vbus_gd Pointer to store VBUS good status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vbus_good_status(bq25896_handle_t handle, bq25896_vbus_gd_t *vbus_gd);

/**
 * @brief Read VBUS voltage
 * Reads the ADC conversion of VBUS voltage from the device
 * 
 * @param handle Device handle
 * @param voltage_mv Pointer to store VBUS voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vbus_voltage(bq25896_handle_t handle, uint16_t *voltage_mv);


/* ####################################################
*                  REGISTER 12h
#################################################### */
/**
 * @brief Read charge current
 * Reads the ADC conversion of charge current (IBAT) from the device
 * Note: Returns 0 when VBAT < VBATSHORT
 * 
 * @param handle Device handle
 * @param current_ma Pointer to store charge current in mA
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charge_current(bq25896_handle_t handle, uint16_t *current_ma);


/* ####################################################
*                  REGISTER 13h
#################################################### */
/**
 * @brief VINDPM Status values (REG13 [7])
 */
typedef enum {
    BQ25896_VDPM_NOT_ACTIVE = 0,  // Not in VINDPM
    BQ25896_VDPM_ACTIVE = 1       // In VINDPM
} bq25896_vdpm_stat_t;

/**
 * @brief IINDPM Status values (REG13 [6])
 */
typedef enum {
    BQ25896_IDPM_NOT_ACTIVE = 0,  // Not in IINDPM
    BQ25896_IDPM_ACTIVE = 1       // In IINDPM
} bq25896_idpm_stat_t;

/**
 * @brief Read VINDPM status
 * 
 * @param handle Device handle
 * @param vdpm_stat Pointer to store VINDPM status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vdpm_status(bq25896_handle_t handle, bq25896_vdpm_stat_t *vdpm_stat);

/**
 * @brief Read IINDPM status
 * 
 * @param handle Device handle
 * @param idpm_stat Pointer to store IINDPM status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_idpm_status(bq25896_handle_t handle, bq25896_idpm_stat_t *idpm_stat);

/**
 * @brief Read Input Current Limit in effect while ICO is enabled
 * 
 * @param handle Device handle
 * @param current_ma Pointer to store input current limit in mA
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ico_current_limit(bq25896_handle_t handle, uint16_t *current_ma);


/* ####################################################
*                  REGISTER 14h
#################################################### */
/**
 * @brief ICO Status values (REG14 [6])
 */
typedef enum {
    BQ25896_ICO_IN_PROGRESS = 0,     // Optimization is in progress
    BQ25896_ICO_COMPLETE = 1         // Maximum Input Current Detected
} bq25896_ico_status_t;

/**
 * @brief Device Revision values (REG14 [1:0])
 */
typedef enum {
    BQ25896_DEV_REV_0 = 0x0,
    BQ25896_DEV_REV_1 = 0x1,
    BQ25896_DEV_REV_2 = 0x2,         // Expected value (10)
    BQ25896_DEV_REV_3 = 0x3
} bq25896_dev_rev_t;

/**
 * @brief Reset all registers to default values
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_reset_registers(bq25896_handle_t handle);

/**
 * @brief Read ICO status
 * 
 * @param handle Device handle
 * @param ico_status Pointer to store ICO status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ico_status(bq25896_handle_t handle, bq25896_ico_status_t *ico_status);

/**
 * @brief Read device part number
 * 
 * @param handle Device handle
 * @param part_number Pointer to store part number
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_part_number(bq25896_handle_t handle, uint8_t *part_number);

/**
 * @brief Read temperature profile setting
 * 
 * @param handle Device handle
 * @param ts_profile Pointer to store temperature profile value (1 for JEITA)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ts_profile(bq25896_handle_t handle, uint8_t *ts_profile);

/**
 * @brief Read device revision
 * 
 * @param handle Device handle
 * @param dev_rev Pointer to store device revision
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_device_revision(bq25896_handle_t handle, bq25896_dev_rev_t *dev_rev);


#ifdef __cplusplus
}
#endif
/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "kode_bq25896.h"
#include "kode_bq25896_priv.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "bq25896 Driver"

static esp_err_t bq25896_read_reg(bq25896_handle_t handle, uint8_t reg, uint8_t *data);
static esp_err_t bq25896_write_reg(bq25896_handle_t handle, uint8_t reg, uint8_t data);
static esp_err_t bq25896_update_bits(bq25896_handle_t handle, uint8_t reg, uint8_t mask, uint8_t value);

esp_err_t bq25896_init(i2c_master_bus_handle_t i2c_bus, bq25896_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(i2c_bus != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid I2C bus handle");
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid output handle");

    // Allocate memory for the device handle
    bq25896_handle_t dev = calloc(1, sizeof(struct bq25896_dev_t));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for BQ25896 device");

    // Initialize with default configuration
    dev->config = BQ25896_DEFAULT_CONFIG();
    dev->i2c_bus = i2c_bus;

    // Add the device to the I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev->config.dev_addr,
        .scl_speed_hz = 400000,  // Standard 400KHz I2C speed
    };
    
    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus");
        free(dev);
        return ret;
    }
    
    dev->dev_handle = dev_handle;  

    // Try to read device part number to confirm the device is accessible
    uint8_t part_number = 0;
    ret = bq25896_get_part_number(dev, &part_number);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device part number");
        i2c_master_bus_rm_device(dev->dev_handle);
        free(dev);
        return ESP_ERR_NOT_FOUND;
    }

    // Check for valid device part number (should be 0 for BQ25896)
    if (part_number != 0) {
        ESP_LOGE(TAG, "Device part number mismatch: expected 0, got %d", part_number);
        i2c_master_bus_rm_device(dev->dev_handle);
        free(dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Read device revision
    bq25896_dev_rev_t dev_rev;
    ret = bq25896_get_device_revision(dev, &dev_rev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device revision");
        i2c_master_bus_rm_device(dev->dev_handle);
        free(dev);
        return ret;
    }

    // Success, return the handle
    *handle = dev;
    ESP_LOGD(TAG, "BQ25896 initialized successfully, part rev: %d", dev_rev);
    ESP_LOGI(TAG, "BQ25896 initialized successfully, version: %d.%d.%d", KODE_BQ25896_VER_MAJOR, KODE_BQ25896_VER_MINOR, KODE_BQ25896_VER_PATCH);
    
    return ESP_OK;
}

esp_err_t bq25896_delete(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Remove the device from the I2C bus
    if (handle->dev_handle != NULL) {
        i2c_master_bus_rm_device(handle->dev_handle);
    }
    
    free(handle);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 00h
#################################################### */
esp_err_t bq25896_set_hiz_mode(bq25896_handle_t handle, bq25896_hiz_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG00, 
                                       BQ25896_REG00_ENHIZ_MASK, 
                                       state == BQ25896_HIZ_ENABLE ? BQ25896_REG00_ENHIZ_MASK : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s HIZ mode", state == BQ25896_HIZ_ENABLE ? "enable" : "disable");
        return ret;
    }
    
    handle->config.enable_hiz = (state == BQ25896_HIZ_ENABLE);
    
    ESP_LOGI(TAG, "HIZ mode %s", state == BQ25896_HIZ_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_ilim_pin(bq25896_handle_t handle, bq25896_ilim_pin_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG00, 
                                       BQ25896_REG00_EN_ILIM_MASK, 
                                       state == BQ25896_ILIM_PIN_ENABLE ? BQ25896_REG00_EN_ILIM_MASK : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s ILIM pin", state == BQ25896_ILIM_PIN_ENABLE ? "enable" : "disable");
        return ret;
    }
    
    handle->config.enable_ilim_pin = (state == BQ25896_ILIM_PIN_ENABLE);
    
    ESP_LOGI(TAG, "ILIM pin %s (current limit is %s by %s)", 
             state == BQ25896_ILIM_PIN_ENABLE ? "enabled" : "disabled",
             state == BQ25896_ILIM_PIN_ENABLE ? "controlled" : "not controlled",
             state == BQ25896_ILIM_PIN_ENABLE ? "external pin" : "I2C register only");
    return ESP_OK;
}

esp_err_t bq25896_set_input_current_limit(bq25896_handle_t handle, bq25896_ilim_t ilim)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG00, 
                                      BQ25896_REG00_IINLIM_MASK, 
                                      ilim);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set input current limit");
        return ret;
    }
    
    // Calculate the actual current based on the enum value
    uint16_t current_ma;
    if (ilim == BQ25896_ILIM_3250MA) {
        current_ma = 3250;
    } else if (ilim <= BQ25896_ILIM_500MA) {
        current_ma = 100 + (ilim * 50);
    } else {
        current_ma = 100 + ((ilim >> 1) * 100);
    }
    
    ESP_LOGI(TAG, "Input current limit set to %d mA", current_ma);
    
    handle->config.input_current_limit = ilim;
    
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 01h
#################################################### */
esp_err_t bq25896_set_bhot_threshold(bq25896_handle_t handle, bq25896_bhot_t threshold)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG01, 
                                       BQ25896_REG01_BHOT_MASK, 
                                       (threshold << BQ25896_REG01_BHOT_SHIFT) & BQ25896_REG01_BHOT_MASK);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BHOT threshold");
        return ret;
    }
    
    handle->config.bhot_threshold = threshold;
    
    const char* threshold_str;
    switch (threshold) {
        case BQ25896_BHOT_THRESHOLD1:
            threshold_str = "34.75% (Threshold1, default)";
            break;
        case BQ25896_BHOT_THRESHOLD0:
            threshold_str = "37.75% (Threshold0)";
                break;
        case BQ25896_BHOT_THRESHOLD2:
            threshold_str = "31.25% (Threshold2)";
                break;
        case BQ25896_BHOT_DISABLED:
            threshold_str = "Disabled";
                break;
            default:
            threshold_str = "Unknown";
                break;
    }
    
    ESP_LOGI(TAG, "Boost hot temperature monitor threshold set to %s", threshold_str);
    return ESP_OK;
}

esp_err_t bq25896_set_bcold_threshold(bq25896_handle_t handle, bq25896_bcold_t threshold)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG01, 
                                       BQ25896_REG01_BCOLD_MASK, 
                                       threshold == BQ25896_BCOLD_THRESHOLD1 ? BQ25896_REG01_BCOLD_MASK : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BCOLD threshold");
        return ret;
    }

    // Update internal config
    handle->config.bcold_threshold = threshold;
    
    ESP_LOGI(TAG, "Boost cold temperature monitor threshold set to %s%%", 
             threshold == BQ25896_BCOLD_THRESHOLD0 ? "77 (Threshold0, default)" : "80 (Threshold1)");
    return ESP_OK;
}

esp_err_t bq25896_set_vindpm_offset(bq25896_handle_t handle, bq25896_vindpm_os_t offset)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG01, 
                                      BQ25896_REG01_VINDPM_OS_MASK, 
                                      offset);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set VINDPM offset");
        return ret;
    }
    
    // Calculate the actual offset based on the enum value
    uint16_t offset_mv = offset * 100;
    
    ESP_LOGI(TAG, "VINDPM offset set to %d mV", offset_mv);
    
    handle->config.vindpm_offset = offset_mv;
    
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 02h
#################################################### */
esp_err_t bq25896_set_adc_conversion(bq25896_handle_t handle, bq25896_adc_conv_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // First read REG02 to check CONV_RATE bit
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG02, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG02");
        return ret;
    }
    
    // Check if CONV_RATE is set to continuous (1)
    if (reg_val & BQ25896_REG02_CONV_RATE_MASK) {
        ESP_LOGE(TAG, "Cannot control ADC conversion when CONV_RATE is set to continuous");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // Update the register
    ret = bq25896_update_bits(handle, BQ25896_REG02, 
                             BQ25896_REG02_CONV_START_MASK, 
                             state << BQ25896_REG02_CONV_START_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s ADC conversion", 
                 state == BQ25896_ADC_CONV_START ? "start" : "stop");
        return ret;
    }
    
    ESP_LOGI(TAG, "ADC conversion %s", 
             state == BQ25896_ADC_CONV_START ? "started" : "stopped");
    return ESP_OK;
}

esp_err_t bq25896_set_adc_conversion_rate(bq25896_handle_t handle, bq25896_adc_conv_rate_t rate)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG02, 
                                       BQ25896_REG02_CONV_RATE_MASK, 
                                       rate << BQ25896_REG02_CONV_RATE_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ADC conversion rate");
        return ret;
    }
    
    handle->config.conv_rate = rate;
    
    ESP_LOGI(TAG, "ADC conversion rate set to %s", 
             rate == BQ25896_ADC_CONV_RATE_ONESHOT ? "one shot" : "continuous");
    return ESP_OK;
}

esp_err_t bq25896_set_boost_frequency(bq25896_handle_t handle, bq25896_boost_freq_t freq)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG02, 
                                       BQ25896_REG02_BOOST_FREQ_MASK, 
                                       freq << BQ25896_REG02_BOOST_FREQ_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set boost frequency");
        return ret;
    }
    
    handle->config.boost_freq = freq;
    
    ESP_LOGI(TAG, "Boost frequency set to %s", 
             freq == BQ25896_BOOST_FREQ_1500KHZ ? "1.5MHz" : "500KHz");
    return ESP_OK;
}

esp_err_t bq25896_set_ico(bq25896_handle_t handle, bq25896_ico_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG02, 
                                       BQ25896_REG02_ICO_EN_MASK, 
                                       state << BQ25896_REG02_ICO_EN_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s Input Current Optimizer", 
                 state == BQ25896_ICO_ENABLE ? "enable" : "disable");
        return ret;
    }
    
    handle->config.ico_en = (state == BQ25896_ICO_ENABLE);
    
    ESP_LOGI(TAG, "Input Current Optimizer (ICO) %s", 
             state == BQ25896_ICO_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}
esp_err_t bq25896_set_force_dpdm(bq25896_handle_t handle, bq25896_force_dpdm_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG02, 
                                       BQ25896_REG02_FORCE_DPDM_MASK, 
                                       state << BQ25896_REG02_FORCE_DPDM_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s input detection", 
                 state == BQ25896_FORCE_DPDM_ENABLE ? "force" : "release");
        return ret;
    }
    
    ESP_LOGI(TAG, "Input source detection %s", 
             state == BQ25896_FORCE_DPDM_ENABLE ? "forced" : "not forced");
    return ESP_OK;
}

esp_err_t bq25896_set_auto_dpdm(bq25896_handle_t handle, bq25896_auto_dpdm_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG02, 
                                       BQ25896_REG02_AUTO_DPDM_EN_MASK, 
                                       state << BQ25896_REG02_AUTO_DPDM_EN_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s automatic input detection", 
                 state == BQ25896_AUTO_DPDM_ENABLE ? "enable" : "disable");
        return ret;
    }
    
    handle->config.auto_dpdm_en = (state == BQ25896_AUTO_DPDM_ENABLE);
    
    ESP_LOGI(TAG, "Automatic input detection %s", 
             state == BQ25896_AUTO_DPDM_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

/* ####################################################
*                  REGISTER 03h
#################################################### */
esp_err_t bq25896_set_bat_load(bq25896_handle_t handle, bq25896_bat_load_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG03, 
                                       BQ25896_REG03_BAT_LOADEN_MASK, 
                                       state << BQ25896_REG03_BAT_LOADEN_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s battery load", state == BQ25896_BAT_LOAD_ENABLE ? "enable" : "disable");
        return ret;
    }
    
    handle->config.bat_load = (state == BQ25896_BAT_LOAD_ENABLE);
    
    ESP_LOGI(TAG, "Battery load %s", state == BQ25896_BAT_LOAD_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_reset_watchdog(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register - bit auto-clears after reset
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG03, 
                                       BQ25896_REG03_WD_RST_MASK, 
                                       BQ25896_WD_RST_RESET << BQ25896_REG03_WD_RST_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset watchdog timer");
        return ret;
    }
    
    ESP_LOGI(TAG, "Watchdog timer reset");
    return ESP_OK;
}

esp_err_t bq25896_set_otg(bq25896_handle_t handle, bq25896_otg_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG03, 
                           BQ25896_REG03_OTG_CONFIG_MASK, 
                                       state << BQ25896_REG03_OTG_CONFIG_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s OTG mode", state == BQ25896_OTG_ENABLE ? "enable" : "disable");
        return ret;
    }
    
    handle->config.otg_config = (state == BQ25896_OTG_ENABLE);
    
    ESP_LOGI(TAG, "OTG mode %s", state == BQ25896_OTG_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_charging(bq25896_handle_t handle, bq25896_chg_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG03, 
                                       BQ25896_REG03_CHG_CONFIG_MASK, 
                                       state << BQ25896_REG03_CHG_CONFIG_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s charging", state == BQ25896_CHG_ENABLE ? "enable" : "disable");
        return ret;
    }
    
    handle->config.chg_config = (state == BQ25896_CHG_ENABLE);
    
    ESP_LOGI(TAG, "Charging %s", state == BQ25896_CHG_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_sys_min(bq25896_handle_t handle, bq25896_sys_min_t sys_min)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG03, 
                                       BQ25896_REG03_SYS_MIN_MASK, 
                                       sys_min << BQ25896_REG03_SYS_MIN_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set minimum system voltage");
        return ret;
    }
    
    handle->config.sys_min = sys_min;
    
    ESP_LOGI(TAG, "Minimum system voltage set to %d.%dV", 
             3 + (sys_min / 10), (sys_min % 10));
    return ESP_OK;
}

esp_err_t bq25896_set_min_vbat(bq25896_handle_t handle, bq25896_min_vbat_sel_t vbat_sel)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG03, 
                                       BQ25896_REG03_MIN_VBAT_SEL_MASK, 
                                       vbat_sel << BQ25896_REG03_MIN_VBAT_SEL_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set minimum battery voltage");
        return ret;
    }
    
    handle->config.min_vbat_sel = (vbat_sel == BQ25896_MIN_VBAT_2500MV);
    
    ESP_LOGI(TAG, "Minimum battery voltage for boost mode exit set to %d.%dV", 
             vbat_sel == BQ25896_MIN_VBAT_2900MV ? 2 : 2, 
             vbat_sel == BQ25896_MIN_VBAT_2900MV ? 900 : 500);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 04h
#################################################### */
esp_err_t bq25896_set_pumpx(bq25896_handle_t handle, bq25896_pumpx_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG04, 
                                       BQ25896_REG04_EN_PUMPX_MASK, 
                                       state << BQ25896_REG04_EN_PUMPX_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s current pulse control", 
                 state == BQ25896_PUMPX_ENABLE ? "enable" : "disable");
        return ret;
    }
    
    ESP_LOGI(TAG, "Current pulse control %s", 
             state == BQ25896_PUMPX_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_charge_current(bq25896_handle_t handle, bq25896_ichg_t ichg)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Clamp value to maximum allowed
    if (ichg > BQ25896_ICHG_3008MA) {
        ichg = BQ25896_ICHG_3008MA;
        ESP_LOGW(TAG, "Charge current clamped to maximum 3008mA");
    }
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG04, 
                                       BQ25896_REG04_ICHG_MASK, 
                                       ichg << BQ25896_REG04_ICHG_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set charge current");
        return ret;
    }
    
    // Calculate actual current in mA
    uint16_t current_ma = ichg * 64;
    
    handle->config.ichg = current_ma;
    
    ESP_LOGI(TAG, "Fast charge current limit set to %d mA", current_ma);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 05h
#################################################### */
esp_err_t bq25896_set_precharge_current(bq25896_handle_t handle, bq25896_prechg_current_t current)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG05, 
                                       BQ25896_REG05_IPRECHG_MASK,
                                       current << BQ25896_REG05_IPRECHG_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set precharge current");
        return ret;
    }
    
    // Calculate actual current
    uint16_t actual_current = (current + 1) * 64;
    
    handle->config.iprechg = actual_current;
    
    ESP_LOGI(TAG, "Precharge current set to %dmA", actual_current);
    return ESP_OK;
}

esp_err_t bq25896_set_termination_current(bq25896_handle_t handle, bq25896_iterm_current_t current)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG05, 
                                       BQ25896_REG05_ITERM_MASK,
                                       current << BQ25896_REG05_ITERM_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set termination current");
        return ret;
    }
    
    // Calculate actual current
    uint16_t actual_current = (current + 1) * 64;
    
    handle->config.iterm = actual_current;
    
    ESP_LOGI(TAG, "Termination current set to %dmA", actual_current);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 06h
#################################################### */
esp_err_t bq25896_set_charge_voltage(bq25896_handle_t handle, bq25896_vreg_t vreg)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG06, 
                                       BQ25896_REG06_VREG_MASK,
                                       vreg << BQ25896_REG06_VREG_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set charge voltage");
        return ret;
    }
    
    // Calculate actual voltage
    uint16_t actual_voltage = 3840 + (vreg * 16);
    
    handle->config.vreg = actual_voltage;
    
    ESP_LOGI(TAG, "Charge voltage set to %dmV", actual_voltage);
    return ESP_OK;
}

esp_err_t bq25896_set_batlowv(bq25896_handle_t handle, bq25896_batlowv_t threshold)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG06, 
                                       BQ25896_REG06_BATLOWV_MASK,
                                       threshold << BQ25896_REG06_BATLOWV_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BATLOWV threshold");
        return ret;
    }
    
    handle->config.batlowv = threshold;
    
    ESP_LOGI(TAG, "BATLOWV threshold set to %dmV", 
             threshold == BQ25896_BATLOWV_2800MV ? 2800 : 3000);
    return ESP_OK;
}

esp_err_t bq25896_set_vrechg(bq25896_handle_t handle, bq25896_vrechg_t vrechg)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG06, 
                                       BQ25896_REG06_VRECHG_MASK,
                                       vrechg << BQ25896_REG06_VRECHG_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set VRECHG threshold");
        return ret;
    }
    
    handle->config.vrechg = vrechg;
    
    ESP_LOGI(TAG, "VRECHG threshold set to %dmV below VREG", 
             vrechg == BQ25896_VRECHG_100MV ? 100 : 200);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 07h
#################################################### */
esp_err_t bq25896_set_termination_state(bq25896_handle_t handle, bq25896_term_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG07, 
                                       BQ25896_REG07_EN_TERM_MASK,
                                       state << BQ25896_REG07_EN_TERM_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set termination state");
        return ret;
    }
    
    handle->config.en_term = (state == BQ25896_TERM_ENABLE);
    
    ESP_LOGI(TAG, "Charging termination %s", 
             state == BQ25896_TERM_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_stat_pin_state(bq25896_handle_t handle, bq25896_stat_pin_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG07, 
                                       BQ25896_REG07_STAT_DIS_MASK,
                                       state << BQ25896_REG07_STAT_DIS_SHIFT);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STAT pin state");
        return ret;
    }
    
    handle->config.stat_dis = (state == BQ25896_STAT_ENABLE);
    
    ESP_LOGI(TAG, "STAT pin %s", 
             state == BQ25896_STAT_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_watchdog_timer(bq25896_handle_t handle, bq25896_watchdog_t watchdog)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG07, 
                                       BQ25896_REG07_WATCHDOG_MASK,
                                       watchdog << BQ25896_REG07_WATCHDOG_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set watchdog timer");
        return ret;
    }
    
    handle->config.watchdog = watchdog;
    
    const char *watchdog_str;
    switch (watchdog) {
        case BQ25896_WATCHDOG_DISABLE: watchdog_str = "disabled"; break;
        case BQ25896_WATCHDOG_40S: watchdog_str = "40 seconds"; break;
        case BQ25896_WATCHDOG_80S: watchdog_str = "80 seconds"; break;
        case BQ25896_WATCHDOG_160S: watchdog_str = "160 seconds"; break;
        default: watchdog_str = "unknown"; break;
    }
    
    ESP_LOGI(TAG, "Watchdog timer set to %s", watchdog_str);
    return ESP_OK;
}

esp_err_t bq25896_set_safety_timer_state(bq25896_handle_t handle, bq25896_safety_timer_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG07, 
                                       BQ25896_REG07_EN_TIMER_MASK,
                                       state << BQ25896_REG07_EN_TIMER_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set safety timer state");
        return ret;
    }
    
    handle->config.en_timer = (state == BQ25896_SAFETY_TIMER_ENABLE);
    
    ESP_LOGI(TAG, "Safety timer %s", 
             state == BQ25896_SAFETY_TIMER_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_charge_timer(bq25896_handle_t handle, bq25896_chg_timer_t timer)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG07, 
                                       BQ25896_REG07_CHG_TIMER_MASK,
                                       timer << BQ25896_REG07_CHG_TIMER_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set charge timer");
        return ret;
    }
    
    handle->config.chg_timer = timer;
    
    const char *timer_str;
    switch (timer) {
        case BQ25896_CHG_TIMER_5H: timer_str = "5 hours"; break;
        case BQ25896_CHG_TIMER_8H: timer_str = "8 hours"; break;
        case BQ25896_CHG_TIMER_12H: timer_str = "12 hours"; break;
        case BQ25896_CHG_TIMER_20H: timer_str = "20 hours"; break;
        default: timer_str = "unknown"; break;
    }
    
    ESP_LOGI(TAG, "Fast charge timer set to %s", timer_str);
    return ESP_OK;
}
    
esp_err_t bq25896_set_jeita_iset(bq25896_handle_t handle, bq25896_jeita_iset_t iset)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG07, 
                                       BQ25896_REG07_JEITA_ISET_MASK,
                                       iset << BQ25896_REG07_JEITA_ISET_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set JEITA low temperature current");
        return ret;
    }
    
    handle->config.jeita_iset = iset;
    
    ESP_LOGI(TAG, "JEITA low temperature current set to %d%% of ICHG", 
             iset == BQ25896_JEITA_ISET_50PCT ? 50 : 20);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 08h
#################################################### */
esp_err_t bq25896_set_bat_comp(bq25896_handle_t handle, bq25896_bat_comp_t bat_comp)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG08, 
                                       BQ25896_REG08_BAT_COMP_MASK,
                                       bat_comp << BQ25896_REG08_BAT_COMP_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set IR compensation resistor");
        return ret;
    }
    
    handle->config.bat_comp = bat_comp;
    
    // Calculate actual value for logging
    uint8_t mohm_value = 0;
    switch (bat_comp) {
        case BQ25896_BAT_COMP_0MO:    mohm_value = 0;   break;
        case BQ25896_BAT_COMP_20MO:   mohm_value = 20;  break;
        case BQ25896_BAT_COMP_40MO:   mohm_value = 40;  break;
        case BQ25896_BAT_COMP_60MO:   mohm_value = 60;  break;
        case BQ25896_BAT_COMP_80MO:   mohm_value = 80;  break;
        case BQ25896_BAT_COMP_100MO:  mohm_value = 100; break;
        case BQ25896_BAT_COMP_120MO:  mohm_value = 120; break;
        case BQ25896_BAT_COMP_140MO:  mohm_value = 140; break;
    }
    
    ESP_LOGI(TAG, "IR compensation resistor set to %dmΩ", mohm_value);
    return ESP_OK;
}

esp_err_t bq25896_set_vclamp(bq25896_handle_t handle, bq25896_vclamp_t vclamp)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG08, 
                                       BQ25896_REG08_VCLAMP_MASK,
                                       vclamp << BQ25896_REG08_VCLAMP_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set IR compensation voltage clamp");
        return ret;
    }
    
    handle->config.vclamp = vclamp;
    
    // Calculate actual value for logging
    uint8_t mv_value = 0;
    switch (vclamp) {
        case BQ25896_VCLAMP_0MV:    mv_value = 0;   break;
        case BQ25896_VCLAMP_32MV:   mv_value = 32;  break;
        case BQ25896_VCLAMP_64MV:   mv_value = 64;  break;
        case BQ25896_VCLAMP_96MV:   mv_value = 96;  break;
        case BQ25896_VCLAMP_128MV:  mv_value = 128; break;
        case BQ25896_VCLAMP_160MV:  mv_value = 160; break;
        case BQ25896_VCLAMP_192MV:  mv_value = 192; break;
        case BQ25896_VCLAMP_224MV:  mv_value = 224; break;
    }
    
    ESP_LOGI(TAG, "IR compensation voltage clamp set to %dmV above VREG", mv_value);
    return ESP_OK;
}

esp_err_t bq25896_set_treg(bq25896_handle_t handle, bq25896_treg_t treg)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG08, 
                                       BQ25896_REG08_TREG_MASK,
                                       treg << BQ25896_REG08_TREG_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set thermal regulation threshold");
        return ret;
    }
    
    handle->config.treg = treg;
    
    // Calculate actual value for logging
    uint8_t degree_value = 0;
    switch (treg) {
        case BQ25896_TREG_60C:   degree_value = 60;  break;
        case BQ25896_TREG_80C:   degree_value = 80;  break;
        case BQ25896_TREG_100C:  degree_value = 100; break;
        case BQ25896_TREG_120C:  degree_value = 120; break;
    }
    
    ESP_LOGI(TAG, "Thermal regulation threshold set to %d°C", degree_value);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 09h
#################################################### */
esp_err_t bq25896_force_ico(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Set the FORCE_ICO bit to trigger ICO
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG09, 
                                       BQ25896_REG09_FORCE_ICO_MASK,
                                       1 << BQ25896_REG09_FORCE_ICO_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to force ICO");
        return ret;
    }
    
    ESP_LOGI(TAG, "Forced ICO started");
    return ESP_OK;
}

esp_err_t bq25896_set_timer_extension(bq25896_handle_t handle, bq25896_tmr2x_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG09, 
                                       BQ25896_REG09_TMR2X_EN_MASK,
                                       state << BQ25896_REG09_TMR2X_EN_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set timer extension state");
        return ret;
    }
    
    handle->config.tmr2x_en = (state == BQ25896_TMR2X_ENABLE);
    
    ESP_LOGI(TAG, "Safety timer extension %s", 
             state == BQ25896_TMR2X_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_batfet_state(bq25896_handle_t handle, bq25896_batfet_state_t state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG09, 
                                       BQ25896_REG09_BATFET_DIS_MASK,
                                       state << BQ25896_REG09_BATFET_DIS_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BATFET state");
        return ret;
    }
    
    handle->config.batfet_dis = (state == BQ25896_BATFET_DISABLE);
    
    ESP_LOGI(TAG, "BATFET %s", 
             state == BQ25896_BATFET_ENABLE ? "enabled" : "disabled (ship mode)");
    return ESP_OK;
}

esp_err_t bq25896_set_jeita_vset(bq25896_handle_t handle, bq25896_jeita_vset_t vset)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG09, 
                                       BQ25896_REG09_JEITA_VSET_MASK,
                                       vset << BQ25896_REG09_JEITA_VSET_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set JEITA high temperature voltage");
        return ret;
    }
    
    handle->config.jeita_vset = vset;
    
    ESP_LOGI(TAG, "JEITA high temperature voltage set to %s", 
             vset == BQ25896_JEITA_VSET_REDUCED ? "VREG-200mV" : "VREG");
    return ESP_OK;
}
    
esp_err_t bq25896_set_batfet_delay(bq25896_handle_t handle, bq25896_batfet_dly_t delay)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG09, 
                                       BQ25896_REG09_BATFET_DLY_MASK,
                                       delay << BQ25896_REG09_BATFET_DLY_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BATFET turn off delay");
        return ret;
    }
    
    handle->config.batfet_dly = delay;
    
    ESP_LOGI(TAG, "BATFET turn off delay %s", 
             delay == BQ25896_BATFET_DLY_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_batfet_reset(bq25896_handle_t handle, bq25896_batfet_rst_t reset)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG09, 
                                       BQ25896_REG09_BATFET_RST_EN_MASK,
                                       reset << BQ25896_REG09_BATFET_RST_EN_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BATFET full system reset");
        return ret;
    }
    
    handle->config.batfet_rst_en = reset;
    
    ESP_LOGI(TAG, "BATFET full system reset %s", 
             reset == BQ25896_BATFET_RST_ENABLE ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_pumpx_up(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Check if EN_PUMPX is enabled
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG04, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG04");
        return ret;
    }
    
    if (!(reg_val & BQ25896_REG04_EN_PUMPX_MASK)) {
        ESP_LOGE(TAG, "EN_PUMPX must be enabled before PUMPX_UP");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set the PUMPX_UP bit
    ret = bq25896_update_bits(handle, BQ25896_REG09, 
                            BQ25896_REG09_PUMPX_UP_MASK,
                            1 << BQ25896_REG09_PUMPX_UP_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PUMPX_UP");
        return ret;
    }
    
    ESP_LOGI(TAG, "Current pulse control voltage up triggered");
    return ESP_OK;
}

esp_err_t bq25896_pumpx_down(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Check if EN_PUMPX is enabled
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG04, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG04");
        return ret;
    }
    
    if (!(reg_val & BQ25896_REG04_EN_PUMPX_MASK)) {
        ESP_LOGE(TAG, "EN_PUMPX must be enabled before PUMPX_DOWN");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set the PUMPX_DN bit
    ret = bq25896_update_bits(handle, BQ25896_REG09, 
                            BQ25896_REG09_PUMPX_DN_MASK,
                            1 << BQ25896_REG09_PUMPX_DN_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PUMPX_DOWN");
        return ret;
    }
    
    ESP_LOGI(TAG, "Current pulse control voltage down triggered");
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 0Ah
#################################################### */
esp_err_t bq25896_set_boost_voltage(bq25896_handle_t handle, bq25896_boostv_t boostv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG0A, 
                                       BQ25896_REG0A_BOOSTV_MASK,
                                       boostv << BQ25896_REG0A_BOOSTV_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set boost voltage");
        return ret;
    }
    
    // Calculate actual voltage for logging and config
    uint16_t actual_voltage = 4550 + (boostv * 64);
    
    handle->config.boostv = actual_voltage;
    
    ESP_LOGI(TAG, "Boost mode voltage set to %dmV", actual_voltage);
    return ESP_OK;
}

esp_err_t bq25896_set_pfm_boost_mode(bq25896_handle_t handle, bq25896_pfm_boost_t pfm_state)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG0A, 
                                       BQ25896_REG0A_PFM_OTG_DIS_MASK,
                                       pfm_state << BQ25896_REG0A_PFM_OTG_DIS_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PFM boost mode");
        return ret;
    }
    
    handle->config.pfm_otg_dis = (pfm_state == BQ25896_PFM_BOOST_DISABLE);
    
    ESP_LOGI(TAG, "PFM in boost mode %s", 
             pfm_state == BQ25896_PFM_BOOST_ALLOW ? "allowed" : "disabled");
    return ESP_OK;
}

esp_err_t bq25896_set_boost_current_limit(bq25896_handle_t handle, bq25896_boost_lim_t boost_lim)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Check for reserved value
    if (boost_lim > BQ25896_BOOST_LIM_2150MA) {
        ESP_LOGE(TAG, "Invalid boost current limit value (reserved)");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG0A, 
                                       BQ25896_REG0A_BOOST_LIM_MASK,
                                       boost_lim << BQ25896_REG0A_BOOST_LIM_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set boost current limit");
        return ret;
    }
    
    // Calculate actual current for logging and config
    uint16_t actual_current = 0;
    switch (boost_lim) {
        case BQ25896_BOOST_LIM_500MA:  actual_current = 500;  break;
        case BQ25896_BOOST_LIM_750MA:  actual_current = 750;  break;
        case BQ25896_BOOST_LIM_1200MA: actual_current = 1200; break;
        case BQ25896_BOOST_LIM_1400MA: actual_current = 1400; break;
        case BQ25896_BOOST_LIM_1650MA: actual_current = 1650; break;
        case BQ25896_BOOST_LIM_1875MA: actual_current = 1875; break;
        case BQ25896_BOOST_LIM_2150MA: actual_current = 2150; break;
        default: actual_current = 0; break;
    }
    
    handle->config.boost_lim = actual_current;
    
    ESP_LOGI(TAG, "Boost mode current limit set to %dmA", actual_current);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 0Bh
#################################################### */
esp_err_t bq25896_get_vbus_status(bq25896_handle_t handle, bq25896_vbus_stat_t *vbus_stat)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(vbus_stat != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid vbus_stat pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0B, &reg_val);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0B");
            return ret;
        }
    
    *vbus_stat = (reg_val & BQ25896_REG0B_VBUS_STAT_MASK) >> BQ25896_REG0B_VBUS_STAT_SHIFT;
    
    // Log the status
    const char *status_str;
    switch (*vbus_stat) {
        case BQ25896_VBUS_STAT_NO_INPUT:  status_str = "No Input"; break;
        case BQ25896_VBUS_STAT_USB_HOST:  status_str = "USB Host SDP"; break;
        case BQ25896_VBUS_STAT_ADAPTER:   status_str = "Adapter (3.25A)"; break;
        case BQ25896_VBUS_STAT_OTG:       status_str = "OTG"; break;
        default:                          status_str = "Unknown"; break;
    }
    
    ESP_LOGD(TAG, "VBUS Status: %s", status_str);
    return ESP_OK;
}

esp_err_t bq25896_get_charging_status(bq25896_handle_t handle, bq25896_chrg_stat_t *chrg_stat)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(chrg_stat != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid chrg_stat pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0B, &reg_val);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0B");
            return ret;
    }
    
    *chrg_stat = (reg_val & BQ25896_REG0B_CHRG_STAT_MASK) >> BQ25896_REG0B_CHRG_STAT_SHIFT;
    
    // Log the status
    const char *status_str;
    switch (*chrg_stat) {
        case BQ25896_CHRG_STAT_NOT_CHARGING:  status_str = "Not Charging"; break;
        case BQ25896_CHRG_STAT_PRE_CHARGE:    status_str = "Pre-charge"; break;
        case BQ25896_CHRG_STAT_FAST_CHARGING: status_str = "Fast Charging"; break;
        case BQ25896_CHRG_STAT_TERM_DONE:     status_str = "Charge Termination Done"; break;
        default:                              status_str = "Unknown"; break;
    }
    
    ESP_LOGD(TAG, "Charging Status: %s", status_str);
    return ESP_OK;
}

esp_err_t bq25896_get_pg_status(bq25896_handle_t handle, bq25896_pg_stat_t *pg_stat)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(pg_stat != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid pg_stat pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0B, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0B");
        return ret;
    }
    
    *pg_stat = (reg_val & BQ25896_REG0B_PG_STAT_MASK) >> BQ25896_REG0B_PG_STAT_SHIFT;
    
    ESP_LOGD(TAG, "Power Good Status: %s", 
             *pg_stat == BQ25896_PG_STAT_GOOD ? "Good" : "Not Good");
    return ESP_OK;
}

esp_err_t bq25896_get_vsys_status(bq25896_handle_t handle, bq25896_vsys_stat_t *vsys_stat)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(vsys_stat != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid vsys_stat pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0B, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0B");
        return ret;
    }
    
    *vsys_stat = (reg_val & BQ25896_REG0B_VSYS_STAT_MASK) >> BQ25896_REG0B_VSYS_STAT_SHIFT;
    
    ESP_LOGD(TAG, "VSYS Status: %s", 
             *vsys_stat == BQ25896_VSYS_STAT_IN_REG ? 
             "In VSYSMIN regulation (BAT < VSYSMIN)" : 
             "Not in VSYSMIN regulation (BAT > VSYSMIN)");
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 0Ch
#################################################### */
esp_err_t bq25896_get_watchdog_fault(bq25896_handle_t handle, bq25896_watchdog_fault_t *fault)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(fault != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid fault pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0C, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0C");
        return ret;
    }
    
    *fault = (reg_val & BQ25896_REG0C_WATCHDOG_FAULT_MASK) >> BQ25896_REG0C_WATCHDOG_FAULT_SHIFT;
    
    ESP_LOGD(TAG, "Watchdog Fault Status: %s", 
             *fault == BQ25896_WD_FAULT_NORMAL ? "Normal" : "Watchdog timer expiration");
    return ESP_OK;
}

esp_err_t bq25896_get_boost_fault(bq25896_handle_t handle, bq25896_boost_fault_t *fault)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(fault != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid fault pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0C, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0C");
        return ret;
    }
    
    *fault = (reg_val & BQ25896_REG0C_BOOST_FAULT_MASK) >> BQ25896_REG0C_BOOST_FAULT_SHIFT;
    
    ESP_LOGD(TAG, "Boost Mode Fault Status: %s", 
             *fault == BQ25896_BOOST_FAULT_NORMAL ? "Normal" : 
             "VBUS overloaded, OVP, or battery too low");
    return ESP_OK;
}

esp_err_t bq25896_get_charge_fault(bq25896_handle_t handle, bq25896_chrg_fault_t *fault)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(fault != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid fault pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0C, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0C");
        return ret;
    }
    
    *fault = (reg_val & BQ25896_REG0C_CHRG_FAULT_MASK) >> BQ25896_REG0C_CHRG_FAULT_SHIFT;
    
    // Log the status
    const char *status_str;
    switch (*fault) {
        case BQ25896_CHRG_FAULT_NORMAL:       status_str = "Normal"; break;
        case BQ25896_CHRG_FAULT_INPUT_FAULT:  status_str = "Input fault"; break;
        case BQ25896_CHRG_FAULT_THERMAL:      status_str = "Thermal shutdown"; break;
        case BQ25896_CHRG_FAULT_TIMER_EXPIRED: status_str = "Safety Timer Expiration"; break;
        default:                              status_str = "Unknown"; break;
    }
    
    ESP_LOGD(TAG, "Charge Fault Status: %s", status_str);
    return ESP_OK;
}

esp_err_t bq25896_get_battery_fault(bq25896_handle_t handle, bq25896_bat_fault_t *fault)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(fault != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid fault pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0C, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0C");
        return ret;
    }
    
    *fault = (reg_val & BQ25896_REG0C_BAT_FAULT_MASK) >> BQ25896_REG0C_BAT_FAULT_SHIFT;
    
    ESP_LOGD(TAG, "Battery Fault Status: %s", 
             *fault == BQ25896_BAT_FAULT_NORMAL ? "Normal" : "Battery Overvoltage");
    return ESP_OK;
}

esp_err_t bq25896_get_ntc_fault(bq25896_handle_t handle, bq25896_ntc_fault_t *fault)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(fault != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid fault pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0C, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0C");
        return ret;
    }
    
    *fault = (reg_val & BQ25896_REG0C_NTC_FAULT_MASK) >> BQ25896_REG0C_NTC_FAULT_SHIFT;
    
    // Log the status
    const char *status_str;
    switch (*fault) {
        case BQ25896_NTC_FAULT_NORMAL:  status_str = "Normal"; break;
        case BQ25896_NTC_FAULT_TS_WARM: status_str = "TS Warm"; break;
        case BQ25896_NTC_FAULT_TS_COOL: status_str = "TS Cool"; break;
        case BQ25896_NTC_FAULT_TS_COLD: status_str = "TS Cold"; break;
        case BQ25896_NTC_FAULT_TS_HOT:  status_str = "TS Hot"; break;
        default:                        status_str = "Unknown"; break;
    }
    
    ESP_LOGD(TAG, "NTC Fault Status: %s", status_str);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 0Dh
#################################################### */
esp_err_t bq25896_set_vindpm_mode(bq25896_handle_t handle, bq25896_force_vindpm_t mode)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Update the register
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG0D, 
                                       BQ25896_REG0D_FORCE_VINDPM_MASK,
                                       mode << BQ25896_REG0D_FORCE_VINDPM_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set VINDPM mode");
        return ret;
    }
    
    handle->config.force_vindpm = mode;
    
    ESP_LOGI(TAG, "VINDPM threshold setting method set to %s", 
             mode == BQ25896_VINDPM_RELATIVE ? "Relative" : "Absolute");
    return ESP_OK;
}

esp_err_t bq25896_set_absolute_vindpm(bq25896_handle_t handle, uint16_t threshold_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Check if in absolute VINDPM mode
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0D, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0D");
        return ret;
    }
    
    bool is_absolute_mode = (reg_val & BQ25896_REG0D_FORCE_VINDPM_MASK) != 0;
    if (!is_absolute_mode) {
        ESP_LOGW(TAG, "Setting absolute VINDPM when in relative mode - switching to absolute mode");
        
        // Switch to absolute mode
        ret = bq25896_set_vindpm_mode(handle, BQ25896_VINDPM_ABSOLUTE);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    // Calculate register value by subtracting offset (2600mV) then dividing by step size (100mV)
    uint8_t reg_val_vindpm = (threshold_mv - 2600) / 100;
    
    // Clamp value to valid range
    if (threshold_mv < 3900) {
        reg_val_vindpm = 13;  // 3.9V minimum (0001101)
        ESP_LOGW(TAG, "VINDPM threshold clamped to minimum (3900mV)");
    } else if (reg_val_vindpm > 0x7F) {
        reg_val_vindpm = 0x7F;  // 15.3V maximum
        ESP_LOGW(TAG, "VINDPM threshold clamped to maximum (15300mV)");
    }
    
    // Update the register
    ret = bq25896_update_bits(handle, BQ25896_REG0D, 
                             BQ25896_REG0D_VINDPM_MASK,
                             reg_val_vindpm << BQ25896_REG0D_VINDPM_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set absolute VINDPM threshold");
        return ret;
    }
    
    // Calculate actual threshold after clamping
    uint16_t actual_threshold = 2600 + (reg_val_vindpm * 100);
    
    handle->config.vindpm = actual_threshold;
    
    ESP_LOGI(TAG, "Absolute VINDPM threshold set to %dmV", actual_threshold);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 0Eh
#################################################### */
esp_err_t bq25896_get_thermal_regulation_status(bq25896_handle_t handle, bq25896_therm_stat_t *therm_stat)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(therm_stat != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid therm_stat pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0E, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0E");
        return ret;
    }
    
    *therm_stat = (reg_val & BQ25896_REG0E_THERM_STAT_MASK) >> BQ25896_REG0E_THERM_STAT_SHIFT;
    
    ESP_LOGD(TAG, "Thermal Regulation Status: %s", 
             *therm_stat == BQ25896_THERM_STAT_NORMAL ? "Normal" : "In Thermal Regulation");
    return ESP_OK;
}

esp_err_t bq25896_get_battery_voltage(bq25896_handle_t handle, uint16_t *voltage_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(voltage_mv != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid voltage_mv pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0E, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0E");
        return ret;
    }
    
    // Extract battery voltage value
    uint8_t batv = (reg_val & BQ25896_REG0E_BATV_MASK) >> BQ25896_REG0E_BATV_SHIFT;
    
    // Calculate actual voltage: offset (2304mV) + batv * step (20mV)
    *voltage_mv = 2304 + (batv * 20);
    
    ESP_LOGD(TAG, "Battery Voltage: %dmV", *voltage_mv);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 0Fh
#################################################### */
esp_err_t bq25896_get_system_voltage(bq25896_handle_t handle, uint16_t *voltage_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(voltage_mv != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid voltage_mv pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG0F, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG0F");
        return ret;
    }
    
    // Extract system voltage value
    uint8_t sysv = (reg_val & BQ25896_REG0F_SYSV_MASK) >> BQ25896_REG0F_SYSV_SHIFT;
    
    // Calculate actual voltage: offset (2304mV) + sysv * step (20mV)
    *voltage_mv = 2304 + (sysv * 20);
    
    ESP_LOGD(TAG, "System Voltage: %dmV", *voltage_mv);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 10h
#################################################### */
esp_err_t bq25896_get_ts_voltage_percentage(bq25896_handle_t handle, float *ts_percentage)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(ts_percentage != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid ts_percentage pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG10, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG10");
        return ret;
    }
    
    // Extract TS voltage percentage value
    uint8_t tspct = (reg_val & BQ25896_REG10_TSPCT_MASK) >> BQ25896_REG10_TSPCT_SHIFT;
    
    // Calculate the actual percentage using the offset and step values
    // Per the datasheet, the values progress as 21%, 21.465%, 21.93%, 22.86%, 24.72%, 28.44%, 35.88%, 50.76%, etc.
    float percentage = 0.0f;
    
    // The formula is not a simple linear scaling, but follows a pattern of doubling steps
    if (tspct == 0) {
        percentage = 21.0f; // Base offset
    } else {
        // Calculate using the stepping pattern shown in the table
        // Step size doubles with each bit position
        float step = 0.465f; // The smallest step
        float value = 21.0f; // Start with offset
        
        for (int i = 0; i < 7; i++) {
            if (tspct & (1 << i)) {
                value += step;
            }
            step *= 2.0f; // Double the step for the next bit position
        }
        
        percentage = value;
    }
    
    // Clamp to valid range
    if (percentage > 80.0f) {
        percentage = 80.0f;
    }
    
    *ts_percentage = percentage;
    
    ESP_LOGD(TAG, "TS Voltage: %.3f%% of REGN", percentage);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 11h
#################################################### */
esp_err_t bq25896_get_vbus_good_status(bq25896_handle_t handle, bq25896_vbus_gd_t *vbus_gd)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(vbus_gd != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid vbus_gd pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG11, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG11");
        return ret;
    }
    
    *vbus_gd = (reg_val & BQ25896_REG11_VBUS_GD_MASK) >> BQ25896_REG11_VBUS_GD_SHIFT;
    
    ESP_LOGD(TAG, "VBUS Good Status: %s", 
             *vbus_gd == BQ25896_VBUS_ATTACHED ? "Attached" : "Not Attached");
    return ESP_OK;
}
    
esp_err_t bq25896_get_vbus_voltage(bq25896_handle_t handle, uint16_t *voltage_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(voltage_mv != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid voltage_mv pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG11, &reg_val);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG11");
            return ret;
    }
    
    // Extract VBUS voltage value
    uint8_t vbusv = (reg_val & BQ25896_REG11_VBUSV_MASK) >> BQ25896_REG11_VBUSV_SHIFT;
    
    // Calculate actual voltage: offset (2600mV) + vbusv * step (100mV)
    *voltage_mv = 2600 + (vbusv * 100);
    
    ESP_LOGD(TAG, "VBUS Voltage: %dmV", *voltage_mv);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 12h
#################################################### */
esp_err_t bq25896_get_charge_current(bq25896_handle_t handle, uint16_t *current_ma)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(current_ma != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid current_ma pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG12, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG12");
        return ret;
    }
    
    // Extract charge current value
    uint8_t ichgr = (reg_val & BQ25896_REG12_ICHGR_MASK) >> BQ25896_REG12_ICHGR_SHIFT;
    
    // Calculate actual current: ichgr * step (50mA)
    *current_ma = ichgr * 50;
    
    ESP_LOGD(TAG, "Charge Current: %dmA", *current_ma);
    
    // If current is 0, add a note about possible VBAT < VBATSHORT condition
    if (*current_ma == 0) {
        ESP_LOGD(TAG, "Note: Zero current may indicate VBAT < VBATSHORT");
    }
    
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 13h
#################################################### */
esp_err_t bq25896_get_vdpm_status(bq25896_handle_t handle, bq25896_vdpm_stat_t *vdpm_stat)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(vdpm_stat != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid vdpm_stat pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG13, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG13");
        return ret;
    }
    
    *vdpm_stat = (reg_val & BQ25896_REG13_VDPM_STAT_MASK) >> BQ25896_REG13_VDPM_STAT_SHIFT;
    
    ESP_LOGD(TAG, "VINDPM Status: %s", 
             *vdpm_stat == BQ25896_VDPM_ACTIVE ? "Active" : "Not Active");
    return ESP_OK;
}

esp_err_t bq25896_get_idpm_status(bq25896_handle_t handle, bq25896_idpm_stat_t *idpm_stat)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(idpm_stat != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid idpm_stat pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG13, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG13");
        return ret;
    }
    
    *idpm_stat = (reg_val & BQ25896_REG13_IDPM_STAT_MASK) >> BQ25896_REG13_IDPM_STAT_SHIFT;
    
    ESP_LOGD(TAG, "IINDPM Status: %s", 
             *idpm_stat == BQ25896_IDPM_ACTIVE ? "Active" : "Not Active");
    return ESP_OK;
}

esp_err_t bq25896_get_ico_current_limit(bq25896_handle_t handle, uint16_t *current_ma)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(current_ma != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid current_ma pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG13, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG13");
        return ret;
    }
    
    // Extract input current limit value
    uint8_t idpm_lim = reg_val & BQ25896_REG13_IDPM_LIM_MASK;
    
    // Calculate actual current: offset (100mA) + idpm_lim * step (50mA)
    if (idpm_lim == 0) {
        *current_ma = 100; // Default/minimum value
    } else {
        *current_ma = 100 + ((idpm_lim - 1) * 50);
    }
    
    ESP_LOGD(TAG, "ICO Current Limit: %dmA", *current_ma);
    return ESP_OK;
}


/* ####################################################
*                  REGISTER 14h
#################################################### */
esp_err_t bq25896_reset_registers(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Set the REG_RST bit to trigger register reset
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG14, 
                                       BQ25896_REG14_REG_RST_MASK,
                                       1 << BQ25896_REG14_REG_RST_SHIFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset registers");
        return ret;
    }
    
    ESP_LOGI(TAG, "Registers reset to default values");
    
    // Add a small delay to ensure reset completes
    vTaskDelay(pdMS_TO_TICKS(10));
    
    return ESP_OK;
}

esp_err_t bq25896_get_ico_status(bq25896_handle_t handle, bq25896_ico_status_t *ico_status)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(ico_status != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid ico_status pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG14, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG14");
        return ret;
    }
    
    *ico_status = (reg_val & BQ25896_REG14_ICO_OPTIMIZED_MASK) >> BQ25896_REG14_ICO_OPTIMIZED_SHIFT;
    
    ESP_LOGD(TAG, "ICO Status: %s", 
             *ico_status == BQ25896_ICO_COMPLETE ? 
             "Complete (Maximum Current Detected)" : "In Progress");
    return ESP_OK;
}

esp_err_t bq25896_get_part_number(bq25896_handle_t handle, uint8_t *part_number)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(part_number != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid part_number pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG14, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG14");
        return ret;
    }
    
    *part_number = (reg_val & BQ25896_REG14_PN_MASK) >> BQ25896_REG14_PN_SHIFT;
    
    if (*part_number == 0) {
        ESP_LOGD(TAG, "Device Part Number: bq25896");
    } else {
        ESP_LOGW(TAG, "Unexpected part number: %d, expected 0 for bq25896", *part_number);
    }
    
    return ESP_OK;
}

esp_err_t bq25896_get_ts_profile(bq25896_handle_t handle, uint8_t *ts_profile)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(ts_profile != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid ts_profile pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG14, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG14");
        return ret;
    }
    
    *ts_profile = (reg_val & BQ25896_REG14_TS_PROFILE_MASK) >> BQ25896_REG14_TS_PROFILE_SHIFT;
    
    ESP_LOGD(TAG, "Temperature Profile: %s", 
             *ts_profile == 1 ? "JEITA (default)" : "Unknown");
    return ESP_OK;
}

esp_err_t bq25896_get_device_revision(bq25896_handle_t handle, bq25896_dev_rev_t *dev_rev)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(dev_rev != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid dev_rev pointer");
    
    uint8_t reg_val;
    esp_err_t ret = bq25896_read_reg(handle, BQ25896_REG14, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REG14");
        return ret;
    }
    
    *dev_rev = (reg_val & BQ25896_REG14_DEV_REV_MASK) >> BQ25896_REG14_DEV_REV_SHIFT;
    
    ESP_LOGD(TAG, "Device Revision: %d", *dev_rev);
    
    // Check if device revision matches expected value (10 = 2)
    if (*dev_rev != BQ25896_DEV_REV_2) {
        ESP_LOGW(TAG, "Unexpected device revision: %d, expected 2", *dev_rev);
    }
    
    return ESP_OK;
}


/* ####################################################
*                  STATIC FUNCTIONS
#################################################### */
static esp_err_t bq25896_read_reg(bq25896_handle_t handle, uint8_t reg, uint8_t *data)
{
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer = 0;
    esp_err_t ret;
    
    // Use the device handle that's already stored in the structure
    ret = i2c_master_transmit_receive(handle->dev_handle, &reg, 1, &buffer, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X", reg);
        return ret;
    }
    
    *data = buffer;
    return ESP_OK;
}

static esp_err_t bq25896_write_reg(bq25896_handle_t handle, uint8_t reg, uint8_t data)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    
    // Prepare the write buffer (register address followed by data)
    uint8_t write_buf[2] = {reg, data};
    
    // Use the persistent device handle from the structure
    ret = i2c_master_transmit(handle->dev_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X with value 0x%02X", reg, data);
        return ret;
    }
    
    return ESP_OK;
}

static esp_err_t bq25896_update_bits(bq25896_handle_t handle, uint8_t reg, uint8_t mask, uint8_t value)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tmp;
    esp_err_t ret;

    ret = bq25896_read_reg(handle, reg, &tmp);
    if (ret != ESP_OK) {
        return ret;
    }

    tmp &= ~mask;
    tmp |= (value & mask);

    return bq25896_write_reg(handle, reg, tmp);
}











// /**
//  * @brief Get charging power
//  * Calculates the charging power based on current and voltage readings
//  * 
//  * @param handle Device handle
//  * @param power_mw Pointer to store charging power in mW
//  * @return esp_err_t ESP_OK on success, error otherwise
//  */
// esp_err_t bq25896_get_charging_power(bq25896_handle_t handle, uint32_t *power_mw)
// {
//     ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
//     ESP_RETURN_ON_FALSE(power_mw != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid power_mw pointer");
    
//     uint16_t current_ma, battery_mv;
    
//     // Read charge current
//     esp_err_t ret = bq25896_get_charge_current(handle, &current_ma);
//     if (ret != ESP_OK) {
//         return ret;
//     }
    
//     // Read battery voltage
//     ret = bq25896_get_battery_voltage(handle, &battery_mv);
//     if (ret != ESP_OK) {
//         return ret;
//     }
    
//     // Calculate power (P = V * I)
//     // Convert mV * mA to mW (need to divide by 1000)
//     *power_mw = ((uint32_t)battery_mv * (uint32_t)current_ma) / 1000;
    
//     ESP_LOGD(TAG, "Charging Power: %dmW (%dmV × %dmA)", *power_mw, battery_mv, current_ma);
//     return ESP_OK;
// }








// /**
//  * @brief Check if device is charging
//  * 
//  * @param handle Device handle
//  * @param is_charging Pointer to store charging state (true if charging, false otherwise)
//  * @return esp_err_t ESP_OK on success, error otherwise
//  */
// esp_err_t bq25896_is_charging(bq25896_handle_t handle, bool *is_charging)
// {
//     ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
//     ESP_RETURN_ON_FALSE(is_charging != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid is_charging pointer");
    
//     bq25896_chrg_stat_t chrg_stat;
//     esp_err_t ret = bq25896_get_charging_status(handle, &chrg_stat);
//     if (ret != ESP_OK) {
//         return ret;
//     }
    
//     // Device is charging if in pre-charge or fast charging state
//     *is_charging = (chrg_stat == BQ25896_CHRG_STAT_PRE_CHARGE || 
//                    chrg_stat == BQ25896_CHRG_STAT_FAST_CHARGING);
    
//     return ESP_OK;
// }

// /**
//  * @brief Check if charging is complete
//  * 
//  * @param handle Device handle
//  * @param is_complete Pointer to store completion state (true if complete, false otherwise)
//  * @return esp_err_t ESP_OK on success, error otherwise
//  */
// esp_err_t bq25896_is_charge_done(bq25896_handle_t handle, bool *is_complete)
// {
//     ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
//     ESP_RETURN_ON_FALSE(is_complete != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid is_complete pointer");
    
//     bq25896_chrg_stat_t chrg_stat;
//     esp_err_t ret = bq25896_get_charging_status(handle, &chrg_stat);
//     if (ret != ESP_OK) {
//         return ret;
//     }
    
//     // Charging is complete if in termination done state
//     *is_complete = (chrg_stat == BQ25896_CHRG_STAT_TERM_DONE);
    
//     return ESP_OK;
// }
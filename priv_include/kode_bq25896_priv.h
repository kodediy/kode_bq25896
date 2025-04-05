/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "kode_bq25896.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BQ25896_REG00       0x00
#define BQ25896_REG01       0x01
#define BQ25896_REG02       0x02
#define BQ25896_REG03       0x03
#define BQ25896_REG04       0x04
#define BQ25896_REG05       0x05
#define BQ25896_REG06       0x06
#define BQ25896_REG07       0x07
#define BQ25896_REG08       0x08
#define BQ25896_REG09       0x09
#define BQ25896_REG0A       0x0A
#define BQ25896_REG0B       0x0B
#define BQ25896_REG0C       0x0C
#define BQ25896_REG0D       0x0D
#define BQ25896_REG0E       0x0E
#define BQ25896_REG0F       0x0F
#define BQ25896_REG10       0x10
#define BQ25896_REG11       0x11
#define BQ25896_REG12       0x12
#define BQ25896_REG13       0x13
#define BQ25896_REG14       0x14




/* ####################################################
*                  REGISTER 00h
#################################################### */
/**
 * @brief Register 00h (Input Source Control) bit definitions
 * 
 * Bit 7: EN_HIZ - Enable HIZ Mode R/W
 *     0: Disable (Default)
 *     1: Enable
 * 
 * Bit 6: EN_ILIM - Enable ILIM Pin R/W
 *     0: Disable
 *     1: Enable (Default: Enable ILIM pin(1))
 *
 * Bits 5-0: IINLIM - Input Current Limit R/W
 *      Note: Input current is the lower value of either I2C control or ILIM pin
 *      Range: 100mA (000000) – 3.25A (111111)
 *      Step size: 50mA
 *      Default: 500mA (001000)
 */
#define BQ25896_REG00_ENHIZ_MASK         0x80  // Bit 7: Enable HIZ Mode
#define BQ25896_REG00_ENHIZ_SHIFT        7
#define BQ25896_REG00_EN_ILIM_MASK       0x40  // Bit 6: Enable ILIM Pin
#define BQ25896_REG00_EN_ILIM_SHIFT      6
#define BQ25896_REG00_IINLIM_MASK        0x3F  // Bits 5-0: Input Current Limit
#define BQ25896_REG00_IINLIM_SHIFT       0


/* ####################################################
*                  REGISTER 01h
#################################################### */
/**
 * @brief Register 01h (Power-On Configuration) bit definitions
 * 
 * Bit 7-6: BHOT - Boost Mode Hot Temperature Monitor Threshold R/W
 *     00: Vbhot1 Threshold (34.75%) (Default)
 *     01: Vbhot0 Threshold (Typ. 37.75%)
 *     10: Vbhot2 Threshold (Typ. 31.25%)
 *     11: Disable boost mode thermal protection
 *
 * Bit 5: BCOLD - Boost Mode Cold Temperature Monitor Threshold R/W
 *     0: Vbcol0 Threshold (Typ. 77%) (Default)
 *     1: Vbcol1 Threshold (Typ. 80%)
 *
 * Bit 4-0: VINDPM_OS - Input Voltage Limit Offset
 *     Range: 0mV (00000) to 3100mV (11111)
 *     Step size: 100mV
 *     Default: 600mV (00110)
 *     When VBUS at no-load is ≤ 6V, VINDPM_OS is used directly to calculate VINDPM
 *     When VBUS at no-load is > 6V, VINDPM_OS is multiplied by 2 to calculate VINDPM
 *     Minimum VINDPM threshold is clamped at 3.9V
 *     Maximum VINDPM threshold is clamped at 15.3V
 */
#define BQ25896_REG01_BHOT_MASK          0xC0  // Bits 7-6: Boost Hot Temp Monitor
#define BQ25896_REG01_BHOT_SHIFT         6
#define BQ25896_REG01_BCOLD_MASK         0x20  // Bit 5: Boost Mode Cold Temperature Monitor 
#define BQ25896_REG01_BCOLD_SHIFT        5
#define BQ25896_REG01_VINDPM_OS_MASK     0x1F  // Bits 4-0: Input Voltage Limit Offset
#define BQ25896_REG01_VINDPM_OS_SHIFT    0


/* ####################################################
*                  REGISTER 02h
#################################################### */
/**
 * @brief Register 02h (Charge Current Control) bit definitions
 * 
 * Bit 7: CONV_START - ADC Conversion Start Control R/W
 *     0: ADC conversion not active (Default)
 *     1: Start ADC conversion
 *     This bit is read-only when CONV_RATE = 1. The bit stays high during ADC conversion and during input source detection
 *
 * Bit 6: CONV_RATE - ADC Conversion Rate Selection R/W
 *     0: One shot ADC conversion (Default)
 *     1: Start 1s Continous Conversion
 *
 * Bit 5: BOOST_FREQ - Boost Mode Frequency Selection R/W
 *     0: 1.5MHz (Default)
 *     1: 500kHz
 *     Write to this bit is ignored when OTG_CONFIG is enabled.
 *
 * Bit 4: ICO_EN - Input Current Optimizer (ICO) Enable R/W
 *     0: Disable ICO Algorithm
 *     1: Enable ICO Algorithm (Default)
 *
 * Bits 3-2 are Reserved, Default: 00 (R/W)
 *
 * Bit 1: FORCE_DPDM - Force Input Detection R/W
 *     0: Not in PSEL detection (Default)
 *     1: Force PSEL detection
 *
 * Bit 0: AUTO_DPDM_EN - Automatic Input Detection Enable R/W
 *     0: Disable PSEL detection when VBUS is plugged-in
 *     1: Enable PSEL detection when VBUS is plugged-in (Default)
 */
#define BQ25896_REG02_CONV_START_MASK    0x80  // Bit 7: ADC Conversion Start Control
#define BQ25896_REG02_CONV_START_SHIFT   7
#define BQ25896_REG02_CONV_RATE_MASK     0x40  // Bit 6: ADC Conversion Rate Selection
#define BQ25896_REG02_CONV_RATE_SHIFT    6
#define BQ25896_REG02_BOOST_FREQ_MASK    0x20  // Bit 5: Boost Mode Frequency Selection
#define BQ25896_REG02_BOOST_FREQ_SHIFT   5
#define BQ25896_REG02_ICO_EN_MASK        0x10  // Bit 4: Input Current Optimizer Enable
#define BQ25896_REG02_ICO_EN_SHIFT       4
// Bits 3-2 are Reserved
#define BQ25896_REG02_FORCE_DPDM_MASK    0x02  // Bit 1: Force Input Detection
#define BQ25896_REG02_FORCE_DPDM_SHIFT   1
#define BQ25896_REG02_AUTO_DPDM_EN_MASK  0x01  // Bit 0: Automatic Input Detection Enable
#define BQ25896_REG02_AUTO_DPDM_EN_SHIFT 0


/* ####################################################
*                  REGISTER 03h
#################################################### */
/**
 * @brief Register 03h (Charge Control) bit definitions
 * 
 * Bit 7: BAT_LOADEN - Battery Load (IBATLOAD) Enable R/W
 *     0: Disable (Default)
 *     1: Enable
 *
 * Bit 6: WD_RST - I2C Watchdog Timer Reset R/W
 *     0: Normal (Default)
 *     1: Reset (Back to 0 after timer reset)
 *
 * Bit 5: OTG_CONFIG - Boost (OTG) Mode Configuration R/W
 *     0: OTG Disable (Default)
 *     1: OTG Enable
 *
 * Bit 4: CHG_CONFIG - Charge Enable Configuration R/W
 *     0: Charge Disable
 *     1: Charge Enable (Default)
 * 
 * Bits 3-1: SYS_MIN - Minimum System Voltage Limit R/W
 *     Range: 3.0V (000) to 3.7V (111)
 *     Step size: 0.1V
 *     Default: 3.5V (101)
 *
 * Bit 0: MIN_VBAT_SEL - Minimum Battery Voltage (falling) to exit boost mode R/W
 *     0: 2.9V (Default)
 *     1: 2.5V
 */
#define BQ25896_REG03_BAT_LOADEN_MASK    0x80  // Bit 7: Battery Load (IBATLOAD) Enable
#define BQ25896_REG03_BAT_LOADEN_SHIFT   7
#define BQ25896_REG03_WD_RST_MASK        0x40  // Bit 6: I2C Watchdog Timer Reset
#define BQ25896_REG03_WD_RST_SHIFT       6
#define BQ25896_REG03_OTG_CONFIG_MASK    0x20  // Bit 5: Boost (OTG) Mode Configuration
#define BQ25896_REG03_OTG_CONFIG_SHIFT   5
#define BQ25896_REG03_CHG_CONFIG_MASK    0x10  // Bit 4: Charge Enable Configuration
#define BQ25896_REG03_CHG_CONFIG_SHIFT   4
#define BQ25896_REG03_SYS_MIN_MASK       0x0E  // Bits 3-1: Minimum System Voltage Limit
#define BQ25896_REG03_SYS_MIN_SHIFT      1
#define BQ25896_REG03_MIN_VBAT_SEL_MASK  0x01  // Bit 0: Minimum Battery Voltage Selection
#define BQ25896_REG03_MIN_VBAT_SEL_SHIFT 0


/* ####################################################
*                  REGISTER 04h
#################################################### */
/**
 * @brief Register 04h (Fast Charge Current Control) bit definitions
 * 
 * Bit 7: EN_PUMPX - Current pulse control Enable R/W
 *     0: Disable Current pulse control(Default)
 *     1: Enable Current pulse control (PUMPX_UP and PUMPX_DN, see REG09)
 * 
 * Bits 6-0: ICHG - Fast Charge Current Limit R/W
 *     Range: 0mA (0000000) to 3008mA (0101111)
 *     Step size: 64mA
 *     Default: 2048mA (0100000)
 *     ICHG=000000 (0mA) disables charge
 *     ICHG>0101111 (3008mA) is clamped to register value 0101111 (3008mA)
 */
#define BQ25896_REG04_EN_PUMPX_MASK      0x80  // Bit 7: Current pulse control Enable
#define BQ25896_REG04_EN_PUMPX_SHIFT     7
#define BQ25896_REG04_ICHG_MASK          0x7F  // Bits 6-0: Fast Charge Current Limit
#define BQ25896_REG04_ICHG_SHIFT         0


/* ####################################################
*                  REGISTER 05h
#################################################### */
/**
 * @brief Register 05h (Pre-Charge/Termination Current Control) bit definitions
 * 
 * Bit 7-4: IPRECHG - Precharge Current Limit R/W
 *     Range: 64mA (0000) to 1024mA (1111)
 *     Step size: 64mA
 *     Default: 128mA (0001)
 * 
 * Bit 3-0: ITERM - Termination Current Limit R/W
 *     Range: 64mA (0000) to 1024mA (1111)
 *     Step size: 64mA
 *     Default: 256mA (0011)
 */
#define BQ25896_REG05_IPRECHG_MASK       0xF0  // Bits 7-4: Precharge Current Limit
#define BQ25896_REG05_IPRECHG_SHIFT      4
#define BQ25896_REG05_ITERM_MASK         0x0F  // Bits 3-0: Termination Current Limit
#define BQ25896_REG05_ITERM_SHIFT        0


/* ####################################################
*                  REGISTER 06h
#################################################### */
/**
 * @brief Register 06h (Charge Voltage Control) bit definitions
 * 
 * Bits 7-2: VREG - Charge Voltage Limit R/W
 *     Offset: 3.840V
 *     Range: 3.840V (000000) - 4.608V (110000)
 *     Step size: 16mV
 *     Default: 4.208V (010111)
 *     Note: VREG > 110000 (4.608V) is clamped to register value 110000 (4.608V)
 * 
 * Bit 1: BATLOWV - Battery Precharge to Fast Charge Threshold R/W
 *     0: 2.8V
 *     1: 3.0V (default)
 * 
 * Bit 0: VRECHG - Battery Recharge Threshold Offset R/W
 *     (below Charge Voltage Limit)
 *     0: 100mV (VRECHG) below VREG (REG06[7:2]) (default)
 *     1: 200mV (VRECHG) below VREG (REG06[7:2])
 */
#define BQ25896_REG06_VREG_MASK          0xFC  // Bits 7-2: Charge Voltage Limit
#define BQ25896_REG06_VREG_SHIFT         2
#define BQ25896_REG06_BATLOWV_MASK       0x02  // Bit 1: Battery Precharge to Fast Charge Threshold
#define BQ25896_REG06_BATLOWV_SHIFT      1
#define BQ25896_REG06_VRECHG_MASK        0x01  // Bit 0: Battery Recharge Threshold
#define BQ25896_REG06_VRECHG_SHIFT       0


/* ####################################################
*                  REGISTER 07h
#################################################### */
/**
 * @brief Register 07h (Charge Termination/Timer Control) bit definitions
 * 
 * Bit 7: EN_TERM - Charging Termination Enable R/W
 *     0: Disable
 *     1: Enable (Default)
 * 
 * Bit 6: STAT_DIS - STAT Pin Disable R/W
 *     0: Enable STAT pin function (Default)
 *     1: Disable
 * 
 * Bits 5-4: WATCHDOG - I2C Watchdog Timer Setting R/W
 *     00: Disable watchdog timer
 *     01: 40s (Default)
 *     10: 80s
 *     11: 160s
 * 
 * Bit 3: EN_TIMER - Charging Safety Timer Enable R/W
 *     0: Disable
 *     1: Enable (Default)
 * 
 * Bits 2-1: Fast Charge Timer Setting R/W
 *     00: 5 hrs
 *     01: 8 hrs
 *     10: 12 hrs (Default)
 *     11: 20 hrs
 * 
 * Bit 0: JEITA_ISET - JEITA Low Temperature Current Setting R/W
 *     0: 50% of ICHG (REG04[6:0])
 *     1: 20% of ICHG (REG04[6:0]) (Default)
 */
#define BQ25896_REG07_EN_TERM_MASK       0x80  // Bit 7: Charging Termination Enable
#define BQ25896_REG07_EN_TERM_SHIFT      7
#define BQ25896_REG07_STAT_DIS_MASK      0x40  // Bit 6: STAT Pin Disable
#define BQ25896_REG07_STAT_DIS_SHIFT     6
#define BQ25896_REG07_WATCHDOG_MASK      0x30  // Bits 5-4: I2C Watchdog Timer Setting
#define BQ25896_REG07_WATCHDOG_SHIFT     4
#define BQ25896_REG07_EN_TIMER_MASK      0x08  // Bit 3: Charging Safety Timer Enable
#define BQ25896_REG07_EN_TIMER_SHIFT     3
#define BQ25896_REG07_CHG_TIMER_MASK     0x06  // Bits 2-1: Fast Charge Timer Setting
#define BQ25896_REG07_CHG_TIMER_SHIFT    1
#define BQ25896_REG07_JEITA_ISET_MASK    0x01  // Bit 0: JEITA Low Temperature Current Setting
#define BQ25896_REG07_JEITA_ISET_SHIFT   0


/* ####################################################
*                  REGISTER 08h
#################################################### */
/**
 * @brief Register 08h (IR Compensation/Thermal Regulation Control) bit definitions
 * 
 * Bits 7-5: BAT_COMP - IR Compensation Resistor Setting R/W
 *     Range: 0 - 140mΩ
 *     Default: 0Ω (000) (i.e. Disable IRComp)
 *     Values:
 *         000: 0mΩ (default, disabled)
 *         001: 20mΩ
 *         010: 40mΩ
 *         011: 60mΩ
 *         100: 80mΩ
 *         101: 100mΩ
 *         110: 120mΩ
 *         111: 140mΩ
 * 
 * Bits 4-2: VCLAMP - IR Compensation Voltage Clamp above VREG (REG06[7:2]) R/W
 *     Offset: 0mV
 *     Range: 0-224mV
 *     Default: 0mV (000)
 *     Values:
 *         000: 0mV (default)
 *         001: 32mV
 *         010: 64mV
 *         011: 96mV
 *         100: 128mV
 *         101: 160mV
 *         110: 192mV
 *         111: 224mV
 * 
 * Bits 1-0: TREG - Thermal Regulation Threshold R/W
 *     00: 60°C
 *     01: 80°C
 *     10: 100°C
 *     11: 120°C (default)
 */
#define BQ25896_REG08_BAT_COMP_MASK      0xE0  // Bits 7-5: BAT_COMP - IR Compensation Resistor Setting
#define BQ25896_REG08_BAT_COMP_SHIFT     5
#define BQ25896_REG08_VCLAMP_MASK        0x1C  // Bits 4-2: VCLAMP - IR Compensation Voltage Clamp above VREG (REG06[7:2])
#define BQ25896_REG08_VCLAMP_SHIFT       2
#define BQ25896_REG08_TREG_MASK          0x03  // Bits 1-0: TREG - Thermal Regulation Threshold
#define BQ25896_REG08_TREG_SHIFT         0


/* ####################################################
*                  REGISTER 09h
#################################################### */
/**
 * @brief Register 09h (Misc Operation Control) bit definitions
 * 
 * Bit 7: FORCE_ICO - Force Start Input Current Optimizer (ICO) R/W
 *     0: Do not force ICO (Default)
 *     1: Force ICO
 *     Note: This bit is can only be set only and always returns to 0 after ICO starts
 * 
 * Bit 6: TMR2X_EN - Safety Timer Setting during DPM or Thermal Regulation
 *     0: Safety timer not slowed by 2X during input DPM or thermal regulation
 *     1: Safety timer slowed by 2X during input DPM or thermal regulation (Default)
 * 
 * Bit 5: BATFET_DIS - Force BATFET off to enable ship mode R/W
 *     0: Allow BATFET turn on (Default)
 *     1: Force BATFET off
 * 
 * Bit 4: JEITA_VSET - JEITA High Temperature Voltage Setting R/W
 *     0: Set Charge Voltage to VREG-200mV during JEITA high temperature (Default)
 *     1: Set Charge Voltage to VREG during JEITA high temperature
 * 
 * Bit 3: BATFET_DLY - BATFET turn off delay control R/W
 *     0: BATFET turn off immediately when BATFET_DIS bit is set (Default)
 *     1: BATFET turn off delay by tsm_dly when BATFET_DIS bit is set
 * 
 * Bit 2: BATFET_RST_EN - BATFET full system reset enable R/W
 *     0: Disable BATFET full system reset
 *     1: Enable BATFET full system reset (Default)
 * 
 * Bits 1: PUMPX_UP - Current pulse control voltage up enable R/W
 *     0: Disable (Default)
 *     1: Enable
 *     Note: This bit is can only be set when EN_PUMPX bit is set and returns to 0 after current pulse control sequence is completed
 * 
 * Bit 0: PUMPX_DN - Current pulse control voltage down enable R/W
 *     0: Disable (Default)
 *     1: Enable
 *     Note: This bit is can only be set when EN_PUMPX bit is set and returns to 0 after current pulse control sequence is completed
 */
#define BQ25896_REG09_FORCE_ICO_MASK     0x80  // Bit 7: Force Start Input Current Optimizer (ICO)
#define BQ25896_REG09_FORCE_ICO_SHIFT    7
#define BQ25896_REG09_TMR2X_EN_MASK      0x40  // Bit 6: Safety Timer Setting during DPM or Thermal Regulation
#define BQ25896_REG09_TMR2X_EN_SHIFT     6
#define BQ25896_REG09_BATFET_DIS_MASK    0x20  // Bit 5: Force BATFET off to enable ship mode
#define BQ25896_REG09_BATFET_DIS_SHIFT   5
#define BQ25896_REG09_JEITA_VSET_MASK    0x10  // Bit 4: JEITA High Temperature Voltage Setting
#define BQ25896_REG09_JEITA_VSET_SHIFT   4
#define BQ25896_REG09_BATFET_DLY_MASK    0x08  // Bit 3: BATFET turn off delay control
#define BQ25896_REG09_BATFET_DLY_SHIFT   3
#define BQ25896_REG09_BATFET_RST_EN_MASK 0x04  // Bit 2: BATFET full system reset enable
#define BQ25896_REG09_BATFET_RST_EN_SHIFT 2
#define BQ25896_REG09_PUMPX_UP_MASK      0x02  // Bit 1: Current pulse control voltage up enable
#define BQ25896_REG09_PUMPX_UP_SHIFT     1
#define BQ25896_REG09_PUMPX_DN_MASK      0x01  // Bit 0: Current pulse control voltage down enable
#define BQ25896_REG09_PUMPX_DN_SHIFT     0


/* ####################################################
*                  REGISTER 0Ah
#################################################### */
/**
 * @brief Register 0Ah (Boost Mode Control) bit definitions
 * 
 * Bits 7-4: BOOSTV - Boost Mode Voltage Regulation R/W
 *     Offset: 4.55V
 *     Range: 4.55V - 5.51V
 *     Default: 4.988V (0111)
 * 
 * Bit 3: PFM_OTG_DIS - PFM mode allowed in boost mode R/W
 *     0: Allow PFM in boost mode (default)
 *     1: Disable PFM in boost mode
 * 
 * Bits 2-0: BOOST_LIM - Boost Mode Current Limit R/W
 *     Default: 1.4A (011)
 *     Values:
 *         000: 0.5A
 *         001: 0.75A
 *         010: 1.2A
 *         011: 1.4A (default)
 *         100: 1.65A
 *         101: 1.875A
 *         110: 2.15A
 *         111: Reserved
 */
#define BQ25896_REG0A_BOOSTV_MASK        0xF0  // Bits 7-4: BOOSTV - Boost Mode Voltage Regulation
#define BQ25896_REG0A_BOOSTV_SHIFT       4
#define BQ25896_REG0A_PFM_OTG_DIS_MASK   0x08  // Bit 3: PFM_OTG_DIS - PFM mode allowed in boost mode
#define BQ25896_REG0A_PFM_OTG_DIS_SHIFT  3
#define BQ25896_REG0A_BOOST_LIM_MASK     0x07  // Bits 2-0: BOOST_LIM - Boost Mode Current Limit
#define BQ25896_REG0A_BOOST_LIM_SHIFT    0


/* ####################################################
*                  REGISTER 0Bh
#################################################### */
/**
 * @brief Register 0Bh (Status Register) bit definitions
 * 
 * Bits 7-5: VBUS_STAT - VBUS Status register R
 *     000: No Input
 *     001: USB Host SDP
 *     010: Adapter (3.25A)
 *     111: OTG
 *     Note: Software current limit is reported in IINLIM register
 * 
 * Bits 4-3: CHRG_STAT - Charging Status R
 *     00: Not Charging
 *     01: Pre-charge (< VBATLOWV)
 *     10: Fast Charging
 *     11: Charge Termination Done
 * 
 * Bit 2: PG_STAT - Power Good Status R
 *     0: Not Power Good
 *     1: Power Good
 * 
 * Bit 1: Reserved, always reads 1
 * 
 * Bit 0: VSYS_STAT - VSYS Regulation Status R
 *     0: Not in VSYSMIN regulation (BAT > VSYSMIN)
 *     1: In VSYSMIN regulation (BAT < VSYSMIN)
 */
#define BQ25896_REG0B_VBUS_STAT_MASK     0xE0  // Bits 7-5: VBUS Status register
#define BQ25896_REG0B_VBUS_STAT_SHIFT    5
#define BQ25896_REG0B_CHRG_STAT_MASK     0x18  // Bits 4-3: Charging Status
#define BQ25896_REG0B_CHRG_STAT_SHIFT    3
#define BQ25896_REG0B_PG_STAT_MASK       0x04  // Bit 2: Power Good Status
#define BQ25896_REG0B_PG_STAT_SHIFT      2
// Bit 1 is Reserved, always reads 1
#define BQ25896_REG0B_VSYS_STAT_MASK     0x01  // Bit 0: VSYS Regulation Status
#define BQ25896_REG0B_VSYS_STAT_SHIFT    0


/* ####################################################
*                  REGISTER 0Ch
#################################################### */
/**
 * @brief Register 0Ch (Fault Register) bit definitions
 * 
 * Bit 7: WATCHDOG_FAULT - Watchdog Fault Status R
 *     0: Normal
 *     1: Watchdog timer expiration
 * 
 * Bit 6: BOOST_FAULT - Boost Mode Fault Status R
 *     0: Normal
 *     1: VBUS overloaded in OTG, or VBUS OVP, or battery is too low in boost mode
 * 
 * Bits 5-4: CHRG_FAULT - Charge Fault Status R
 *     00: Normal
 *     01: Input fault (VBUS > VACOV or VBAT < VBUS < VVBUSMIN(typical 3.8V))
 *     10: Thermal shutdown
 *     11: Charge Safety Timer Expiration
 * 
 * Bit 3: BAT_FAULT - Battery Fault Status R
 *     0: Normal
 *     1: BATOVP (VBAT > VBATOVP)
 * 
 * Bits 2-0: NTC_FAULT - NTC Fault Status R
 *     Buck Mode:
 *         000: Normal
 *         010: TS Warm
 *         011: TS Cool
 *         101: TS Cold
 *         110: TS Hot
 *     Boost Mode:
 *         000: Normal
 *         101: TS Cold
 *         110: TS Hot
 */
#define BQ25896_REG0C_WATCHDOG_FAULT_MASK 0x80  // Bit 7: Watchdog Fault Status
#define BQ25896_REG0C_WATCHDOG_FAULT_SHIFT 7
#define BQ25896_REG0C_BOOST_FAULT_MASK    0x40  // Bit 6: Boost Mode Fault Status
#define BQ25896_REG0C_BOOST_FAULT_SHIFT   6
#define BQ25896_REG0C_CHRG_FAULT_MASK     0x30  // Bits 5-4: Charge Fault Status
#define BQ25896_REG0C_CHRG_FAULT_SHIFT    4
#define BQ25896_REG0C_BAT_FAULT_MASK      0x08  // Bit 3: Battery Fault Status
#define BQ25896_REG0C_BAT_FAULT_SHIFT     3
#define BQ25896_REG0C_NTC_FAULT_MASK      0x07  // Bits 2-0: NTC Fault Status
#define BQ25896_REG0C_NTC_FAULT_SHIFT     0


/* ####################################################
*                  REGISTER 0Dh
#################################################### */
/**
 * @brief Register 0Dh (VINDPM Register) bit definitions
 * 
 * Bit 7: FORCE_VINDPM - VINDPM Threshold Setting Method R/W
 *     0: Run Relative VINDPM Threshold (default)
 *     1: Run Absolute VINDPM Threshold
 *     Note: Register is reset to default value when input source is plugged-in
 * 
 * Bits 6-0: VINDPM - Absolute VINDPM Threshold R/W
 *     Offset: 2.6V
 *     Range: 3.9V (0001101) - 15.3V (1111111)
 *     Default: 4.4V (0010010)
 *     Notes:
 *         - Value < 0001101 is clamped to 3.9V (0001101)
 *         - Register is read only when FORCE_VINDPM=0 and can be written by internal 
 *           control based on relative VINDPM threshold setting
 *         - Register can be read/write when FORCE_VINDPM = 1
 *         - Register is reset to default value when input source is plugged-in
 */
#define BQ25896_REG0D_FORCE_VINDPM_MASK  0x80  // Bit 7: VINDPM Threshold Setting Method
#define BQ25896_REG0D_FORCE_VINDPM_SHIFT 7
#define BQ25896_REG0D_VINDPM_MASK        0x7F  // Bits 6-0: Absolute VINDPM Threshold
#define BQ25896_REG0D_VINDPM_SHIFT       0


/* ####################################################
*                  REGISTER 0Eh
#################################################### */
/**
 * @brief Register 0Eh (Battery Voltage Register) bit definitions
 * 
 * Bit 7: THERM_STAT - Thermal Regulation Status R
 *     0: Normal
 *     1: In Thermal Regulation
 * 
 * Bits 6-0: BATV - ADC conversion of Battery Voltage (VBAT) R
 *     Offset: 2.304V
 *     Range: 2.304V (0000000) - 4.848V (1111111)
 *     Default: 2.304V (0000000)
 *     Step size: 20mV
 */
#define BQ25896_REG0E_THERM_STAT_MASK   0x80  // Bit 7: Thermal Regulation Status
#define BQ25896_REG0E_THERM_STAT_SHIFT  7
#define BQ25896_REG0E_BATV_MASK         0x7F  // Bits 6-0: Battery Voltage
#define BQ25896_REG0E_BATV_SHIFT        0

/**
 * @brief Convert raw BATV register value to battery voltage in mV
 * Utility function for internal use or debugging
 * 
 * @param batv_reg_val Raw BATV register value (0-127)
 * @return uint16_t Battery voltage in mV
 */
uint16_t bq25896_batv_to_mv(uint8_t batv_reg_val)
{
    // Ensure value is within valid range
    batv_reg_val &= BQ25896_REG0E_BATV_MASK;
    
    // Calculate voltage: offset (2304mV) + batv * step (20mV)
    return 2304 + (batv_reg_val * 20);
}


/* ####################################################
*                  REGISTER 0Fh
#################################################### */
/**
 * @brief Register 0Fh (System Voltage Register) bit definitions
 * 
 * Bit 7: Reserved
 *     Reserved: Always reads 0
 * 
 * Bits 6-0: SYSV - ADC conversion of System Voltage (VSYS) R
 *     Offset: 2.304V
 *     Range: 2.304V (0000000) - 4.848V (1111111)
 *     Default: 2.304V (0000000)
 *     Step size: 20mV
 */
#define BQ25896_REG0F_RESERVED_MASK     0x80  // Bit 7: Reserved (always reads 0)
#define BQ25896_REG0F_RESERVED_SHIFT    7
#define BQ25896_REG0F_SYSV_MASK         0x7F  // Bits 6-0: System Voltage
#define BQ25896_REG0F_SYSV_SHIFT        0

/**
 * @brief Convert raw SYSV register value to system voltage in mV
 * Utility function for internal use or debugging
 * 
 * @param sysv_reg_val Raw SYSV register value (0-127)
 * @return uint16_t System voltage in mV
 */
uint16_t bq25896_sysv_to_mv(uint8_t sysv_reg_val)
{
    // Ensure value is within valid range
    sysv_reg_val &= BQ25896_REG0F_SYSV_MASK;
    
    // Calculate voltage: offset (2304mV) + sysv * step (20mV)
    return 2304 + (sysv_reg_val * 20);
}


/* ####################################################
*                  REGISTER 10h
#################################################### *//**
/**
 * @brief Register 10h (TS Voltage / THERM_STAT Register) bit definitions
 * 
 * Bit 7: Reserved R
 *     Reserved: Always reads 0
 * 
 * Bits 6-0: TSPCT - ADC conversion of TS Voltage (TS) as percentage of REGN R
 *     Offset: 21%
 *     Range: 21% (0000000) - 80% (1111111)
 *     Default: 21% (0000000)
 */
#define BQ25896_REG10_RESERVED_MASK     0x80  // Bit 7: Reserved (always reads 0)
#define BQ25896_REG10_RESERVED_SHIFT    7
#define BQ25896_REG10_TSPCT_MASK        0x7F  // Bits 6-0: TS Voltage as Percentage of REGN
#define BQ25896_REG10_TSPCT_SHIFT       0


/* ####################################################
*                  REGISTER 11h
#################################################### */
/**
 * @brief Register 11h (VBUS Voltage Register) bit definitions
 * 
 * Bit 7: VBUS_GD - VBUS Good Status R
 *     0: Not VBUS attached
 *     1: VBUS Attached
 * 
 * Bits 6-0: VBUSV - ADC conversion of VBUS voltage (VBUS) R
 *     Offset: 2.6V
 *     Range: 2.6V (0000000) - 15.3V (1111111)
 *     Default: 2.6V (0000000)
 *     Step size: 100mV
 */
#define BQ25896_REG11_VBUS_GD_MASK      0x80  // Bit 7: VBUS Good Status
#define BQ25896_REG11_VBUS_GD_SHIFT     7
#define BQ25896_REG11_VBUSV_MASK        0x7F  // Bits 6-0: VBUS Voltage Reading
#define BQ25896_REG11_VBUSV_SHIFT       0

/**
 * @brief Convert raw VBUSV register value to VBUS voltage in mV
 * Utility function for internal use or debugging
 * 
 * @param vbusv_reg_val Raw VBUSV register value (0-127)
 * @return uint16_t VBUS voltage in mV
 */
uint16_t bq25896_vbusv_to_mv(uint8_t vbusv_reg_val)
{
    // Ensure value is within valid range
    vbusv_reg_val &= BQ25896_REG11_VBUSV_MASK;
    
    // Calculate voltage: offset (2600mV) + vbusv * step (100mV)
    return 2600 + (vbusv_reg_val * 100);
}


/* ####################################################
*                  REGISTER 12h
#################################################### */
/**
 * @brief Register 12h (Charge Current Register) bit definitions
 * 
 * Bit 7: Unused R
 *     Always reads 0
 * 
 * Bits 6-0: ICHGR - ADC conversion of Charge Current (IBAT) when VBAT > VBATSHORT R
 *     Offset: 0mA
 *     Range: 0mA (0000000) - 6350mA (1111111)
 *     Default: 0mA (0000000)
 *     Step size: 50mA
 *     Note:
 *         This register returns 0000000 for VBAT < VBATSHORT
 */
#define BQ25896_REG12_UNUSED_MASK       0x80  // Bit 7: Unused (always reads 0)
#define BQ25896_REG12_UNUSED_SHIFT      7
#define BQ25896_REG12_ICHGR_MASK        0x7F  // Bits 6-0: Charge Current Reading
#define BQ25896_REG12_ICHGR_SHIFT       0

/**
 * @brief Convert raw ICHGR register value to charge current in mA
 * Utility function for internal use or debugging
 * 
 * @param ichgr_reg_val Raw ICHGR register value (0-127)
 * @return uint16_t Charge current in mA
 */
uint16_t bq25896_ichgr_to_ma(uint8_t ichgr_reg_val)
{
    // Ensure value is within valid range
    ichgr_reg_val &= BQ25896_REG12_ICHGR_MASK;
    
    // Calculate current: ichgr * step (50mA)
    return ichgr_reg_val * 50;
}


/* ####################################################
*                  REGISTER 13h
#################################################### */ 
/**
 * @brief Register 13h (VDPM / IDPM Status Register) bit definitions
 * 
 * Bit 7: VDPM_STAT - VINDPM Status R
 *     0: Not in VINDPM
 *     1: VINDPM
 * 
 * Bit 6: IDPM_STAT - IINDPM Status R
 *     0: Not in IINDPM
 *     1: IINDPM
 * 
 * Bits 5-0: IDPM_LIM - Input Current Limit in effect while Input Current Optimizer (ICO) is enabled R
 *     Offset: 100mA (default)
 *     Range: 100mA (000000) - 3.25mA (111111)
 *     Step size: 50mA
 */
#define BQ25896_REG13_VDPM_STAT_MASK    0x80  // Bit 7: VINDPM Status
#define BQ25896_REG13_VDPM_STAT_SHIFT   7
#define BQ25896_REG13_IDPM_STAT_MASK    0x40  // Bit 6: IINDPM Status
#define BQ25896_REG13_IDPM_STAT_SHIFT   6
#define BQ25896_REG13_IDPM_LIM_MASK     0x3F  // Bits 5-0: Input Current Limit Setting
#define BQ25896_REG13_IDPM_LIM_SHIFT    0

/**
 * @brief Convert raw IDPM_LIM register value to current in mA
 * Utility function for internal use or debugging
 * 
 * @param idpm_lim_reg_val Raw IDPM_LIM register value (0-63)
 * @return uint16_t Input current limit in mA
 */
uint16_t bq25896_idpm_lim_to_ma(uint8_t idpm_lim_reg_val)
{
    // Ensure value is within valid range
    idpm_lim_reg_val &= BQ25896_REG13_IDPM_LIM_MASK;
    
    // Calculate current: offset (100mA) + (idpm_lim - 1) * step (50mA) for non-zero values
    if (idpm_lim_reg_val == 0) {
        return 100; // Default/minimum value
    } else {
        return 100 + ((idpm_lim_reg_val - 1) * 50);
    }
}

/* ####################################################
*                  REGISTER 14h
#################################################### */ 
/**
 * @brief Register 14h (Device Information Register) bit definitions
 * 
 * Bit 7: REG_RST - Register Reset R/W
 *     0: Keep current register setting (default)
 *     1: Reset to default register value and reset safety timer
 *     Note: Reset to 0 after register reset is completed
 * 
 * Bit 6: ICO_OPTIMIZED - Input Current Optimizer (ICO) Status R
 *     0: Optimization is in progress
 *     1: Maximum Input Current Detected
 * 
 * Bits 5-3: PN - Device Configuration R
 *     000: bq25896
 * 
 * Bit 2: TS_PROFILE - Temperature Profile R
 *     1: JEITA (default)
 * 
 * Bits 1-0: DEV_REV - Device Revision R
 *     10: Device Revision
 */
#define BQ25896_REG14_REG_RST_MASK       0x80  // Bit 7: Register Reset
#define BQ25896_REG14_REG_RST_SHIFT      7
#define BQ25896_REG14_ICO_OPTIMIZED_MASK 0x40  // Bit 6: Input Current Optimizer Status
#define BQ25896_REG14_ICO_OPTIMIZED_SHIFT 6
#define BQ25896_REG14_PN_MASK            0x38  // Bits 5-3: Device Configuration (000: bq25896)
#define BQ25896_REG14_PN_SHIFT           3
#define BQ25896_REG14_TS_PROFILE_MASK    0x04  // Bit 2: Temperature Profile (1: JEITA)
#define BQ25896_REG14_TS_PROFILE_SHIFT   2
#define BQ25896_REG14_DEV_REV_MASK       0x03  // Bits 1-0: Device Revision (10)
#define BQ25896_REG14_DEV_REV_SHIFT      0






/**
 * @brief BQ25896 device structure
 */
struct bq25896_dev_t {
    i2c_master_bus_handle_t i2c_bus;         /*!< I2C bus handle */
    bq25896_config_t config;                  /*!< Current configuration */
};

#ifdef __cplusplus
}
#endif 
# kode_bq25896

ESP-IDF component for the BQ25896 battery charger IC.

## Overview

The BQ25896 is a high-efficiency single-cell Li-Ion/Li-Polymer battery charger and system power path management IC from Texas Instruments. This ESP-IDF component provides a complete API for configuring and controlling the BQ25896 via I2C.

## Register Reference

The BQ25896 contains the following registers:

### REG00 - Input Source Control
- **EN_HIZ** (Bit 7): Enable HIZ Mode `R/W`
  - 0: Disable (Default)
  - 1: Enable
- **EN_ILIM** (Bit 6): Enable ILIM Pin `R/W`
  - 0: Disable
  - 1: Enable (Default)
- **IINLIM** (Bits 5-0): Input Current Limit `R/W`
  - Range: 100mA (000000) – 3.25A (111111)
  - Step size: 50mA
  - Default: 500mA (001000)

### REG01 - Power-On Configuration
- **BHOT** (Bits 7-6): Boost Mode Hot Temperature Monitor Threshold `R/W`
  - 00: Vbhot1 Threshold (34.75%) (Default)
  - 01: Vbhot0 Threshold (37.75%)
  - 10: Vbhot2 Threshold (31.25%)
  - 11: Disable boost mode thermal protection
- **BCOLD** (Bit 5): Boost Mode Cold Temperature Monitor Threshold `R/W`
  - 0: Vbcol0 Threshold (77%) (Default)
  - 1: Vbcol1 Threshold (80%)
- **VINDPM_OS** (Bits 4-0): Input Voltage Limit Offset `R/W`
  - Range: 0mV (00000) to 3100mV (11111)
  - Step size: 100mV
  - Default: 600mV (00110)

### REG02 - Charge Current Control
- **CONV_START** (Bit 7): ADC Conversion Start Control `R/W`
  - 0: ADC conversion not active (Default)
  - 1: Start ADC conversion
  - Note: Read-only when CONV_RATE = 1
- **CONV_RATE** (Bit 6): ADC Conversion Rate Selection `R/W`
  - 0: One shot ADC conversion (Default)
  - 1: Start 1s Continuous Conversion
- **BOOST_FREQ** (Bit 5): Boost Mode Frequency Selection `R/W`
  - 0: 1.5MHz (Default)
  - 1: 500kHz
  - Note: Write ignored when OTG_CONFIG is enabled
- **ICO_EN** (Bit 4): Input Current Optimizer Enable `R/W`
  - 0: Disable ICO Algorithm
  - 1: Enable ICO Algorithm (Default)
- **FORCE_DPDM** (Bit 1): Force Input Detection `R/W`
  - 0: Not in PSEL detection (Default)
  - 1: Force PSEL detection
- **AUTO_DPDM_EN** (Bit 0): Automatic Input Detection Enable `R/W`
  - 0: Disable PSEL detection when VBUS is plugged-in
  - 1: Enable PSEL detection when VBUS is plugged-in (Default)

### REG03 - Charge Control
- **BAT_LOADEN** (Bit 7): Battery Load Enable `R/W`
  - 0: Disable (Default)
  - 1: Enable
- **WD_RST** (Bit 6): I2C Watchdog Timer Reset `R/W`
  - 0: Normal (Default)
  - 1: Reset (Back to 0 after timer reset)
- **OTG_CONFIG** (Bit 5): Boost (OTG) Mode Configuration `R/W`
  - 0: OTG Disable (Default)
  - 1: OTG Enable
- **CHG_CONFIG** (Bit 4): Charge Enable Configuration `R/W`
  - 0: Charge Disable
  - 1: Charge Enable (Default)
- **SYS_MIN** (Bits 3-1): Minimum System Voltage Limit `R/W`
  - Range: 3.0V (000) to 3.7V (111)
  - Step size: 0.1V
  - Default: 3.5V (101)
- **MIN_VBAT_SEL** (Bit 0): Minimum Battery Voltage to exit boost mode `R/W`
  - 0: 2.9V (Default)
  - 1: 2.5V

### REG04 - Fast Charge Current Control
- **EN_PUMPX** (Bit 7): Current pulse control Enable `R/W`
  - 0: Disable (Default)
  - 1: Enable
- **ICHG** (Bits 6-0): Fast Charge Current Limit `R/W`
  - Range: 0mA (0000000) to 3008mA (0101111)
  - Step size: 64mA
  - Default: 2048mA (0100000)

### REG05 - Pre-Charge/Termination Current Control
- **IPRECHG** (Bits 7-4): Precharge Current Limit `R/W`
  - Range: 64mA (0000) to 1024mA (1111)
  - Step size: 64mA
  - Default: 128mA (0001)
- **ITERM** (Bits 3-0): Termination Current Limit `R/W`
  - Range: 64mA (0000) to 1024mA (1111)
  - Step size: 64mA
  - Default: 256mA (0011)

### REG06 - Charge Voltage Control
- **VREG** (Bits 7-2): Charge Voltage Limit `R/W`
  - Offset: 3.840V
  - Range: 3.840V (000000) - 4.608V (110000)
  - Step size: 16mV
  - Default: 4.208V (010111)
- **BATLOWV** (Bit 1): Battery Precharge to Fast Charge Threshold `R/W`
  - 0: 2.8V
  - 1: 3.0V (default)
- **VRECHG** (Bit 0): Battery Recharge Threshold Offset `R/W`
  - 0: 100mV below VREG (default)
  - 1: 200mV below VREG

### REG07 - Charge Termination/Timer Control
- **EN_TERM** (Bit 7): Charging Termination Enable `R/W`
  - 0: Disable
  - 1: Enable (Default)
- **STAT_DIS** (Bit 6): STAT Pin Disable `R/W`
  - 0: Enable STAT pin function (Default)
  - 1: Disable
- **WATCHDOG** (Bits 5-4): I2C Watchdog Timer Setting `R/W`
  - 00: Disable watchdog timer
  - 01: 40s (Default)
  - 10: 80s
  - 11: 160s
- **EN_TIMER** (Bit 3): Charging Safety Timer Enable `R/W`
  - 0: Disable
  - 1: Enable (Default)
- **CHG_TIMER** (Bits 2-1): Fast Charge Timer Setting `R/W`
  - 00: 5 hrs
  - 01: 8 hrs
  - 10: 12 hrs (Default)
  - 11: 20 hrs
- **JEITA_ISET** (Bit 0): JEITA Low Temperature Current Setting `R/W`
  - 0: 50% of ICHG
  - 1: 20% of ICHG (Default)

### REG08 - IR Compensation/Thermal Regulation Control
- **BAT_COMP** (Bits 7-5): IR Compensation Resistor Setting `R/W`
  - Range: 0 - 140mΩ
  - Default: 0Ω (000)
- **VCLAMP** (Bits 4-2): IR Compensation Voltage Clamp `R/W`
  - Range: 0 - 224mV
  - Default: 0mV (000)
- **TREG** (Bits 1-0): Thermal Regulation Threshold `R/W`
  - 00: 60°C
  - 01: 80°C
  - 10: 100°C
  - 11: 120°C (default)

### REG09 - Misc Operation Control
- **FORCE_ICO** (Bit 7): Force Start Input Current Optimizer `R/W`
  - 0: Do not force ICO (Default)
  - 1: Force ICO
  - Note: Returns to 0 after ICO starts
- **TMR2X_EN** (Bit 6): Safety Timer Setting during DPM/Thermal Regulation `R/W`
  - 0: Safety timer not slowed by 2X
  - 1: Safety timer slowed by 2X (Default)
- **BATFET_DIS** (Bit 5): Force BATFET off `R/W`
  - 0: Allow BATFET turn on (Default)
  - 1: Force BATFET off
- **JEITA_VSET** (Bit 4): JEITA High Temperature Voltage Setting `R/W`
  - 0: VREG-200mV (Default)
  - 1: VREG
- **BATFET_DLY** (Bit 3): BATFET turn off delay control `R/W`
  - 0: Turn off immediately (Default)
  - 1: Turn off with delay
- **BATFET_RST_EN** (Bit 2): BATFET full system reset enable `R/W`
  - 0: Disable
  - 1: Enable (Default)
- **PUMPX_UP** (Bit 1): Current pulse control voltage up enable `R/W`
  - 0: Disable (Default)
  - 1: Enable
  - Note: Returns to 0 after pulse control sequence completes
- **PUMPX_DN** (Bit 0): Current pulse control voltage down enable `R/W`
  - 0: Disable (Default)
  - 1: Enable
  - Note: Returns to 0 after pulse control sequence completes

### REG0A - Boost Mode Control
- **BOOSTV** (Bits 7-4): Boost Mode Voltage Regulation `R/W`
  - Offset: 4.55V
  - Range: 4.55V - 5.51V
  - Default: 4.988V (0111)
- **PFM_OTG_DIS** (Bit 3): PFM mode allowed in boost mode `R/W`
  - 0: Allow PFM (default)
  - 1: Disable PFM
- **BOOST_LIM** (Bits 2-0): Boost Mode Current Limit `R/W`
  - 000: 0.5A
  - 001: 0.75A
  - 010: 1.2A
  - 011: 1.4A (default)
  - 100: 1.65A
  - 101: 1.875A
  - 110: 2.15A
  - 111: Reserved

### REG0B - Status Register
- **VBUS_STAT** (Bits 7-5): VBUS Status register `R`
  - 000: No Input
  - 001: USB Host SDP
  - 010: Adapter (3.25A)
  - 111: OTG
- **CHRG_STAT** (Bits 4-3): Charging Status `R`
  - 00: Not Charging
  - 01: Pre-charge (< VBATLOWV)
  - 10: Fast Charging
  - 11: Charge Termination Done
- **PG_STAT** (Bit 2): Power Good Status `R`
  - 0: Not Power Good
  - 1: Power Good
- **VSYS_STAT** (Bit 0): VSYS Regulation Status `R`
  - 0: Not in VSYSMIN regulation (BAT > VSYSMIN)
  - 1: In VSYSMIN regulation (BAT < VSYSMIN)

### REG0C - Fault Register
- **WATCHDOG_FAULT** (Bit 7): Watchdog Fault Status `R`
  - 0: Normal
  - 1: Watchdog timer expiration
- **BOOST_FAULT** (Bit 6): Boost Mode Fault Status `R`
  - 0: Normal
  - 1: VBUS overloaded in OTG, or VBUS OVP, or battery is too low in boost mode
- **CHRG_FAULT** (Bits 5-4): Charge Fault Status `R`
  - 00: Normal
  - 01: Input fault (VBUS > VACOV or VBAT < VBUS < VVBUSMIN)
  - 10: Thermal shutdown
  - 11: Charge Safety Timer Expiration
- **BAT_FAULT** (Bit 3): Battery Fault Status `R`
  - 0: Normal
  - 1: BATOVP (VBAT > VBATOVP)
- **NTC_FAULT** (Bits 2-0): NTC Fault Status `R`
  - Buck Mode:
    - 000: Normal
    - 010: TS Warm
    - 011: TS Cool
    - 101: TS Cold
    - 110: TS Hot
  - Boost Mode:
    - 000: Normal
    - 101: TS Cold
    - 110: TS Hot

### REG0D - VINDPM Register
- **FORCE_VINDPM** (Bit 7): VINDPM Threshold Setting Method `R/W`
  - 0: Relative VINDPM Threshold (default)
  - 1: Absolute VINDPM Threshold
- **VINDPM** (Bits 6-0): Absolute VINDPM Threshold `R/W`
  - Offset: 2.6V
  - Range: 3.9V (0001101) - 15.3V (1111111)
  - Default: 4.4V (0010010)
  - Note: Read-only when FORCE_VINDPM=0

### REG0E - Battery Voltage Register
- **THERM_STAT** (Bit 7): Thermal Regulation Status `R`
  - 0: Normal
  - 1: In Thermal Regulation
- **BATV** (Bits 6-0): Battery Voltage Reading `R`
  - Offset: 2.304V
  - Range: 2.304V - 4.848V
  - Step size: 20mV

### REG0F - System Voltage Register
- **SYSV** (Bits 6-0): System Voltage Reading `R`
  - Offset: 2.304V
  - Range: 2.304V - 4.848V
  - Step size: 20mV

### REG10 - TS Voltage Register
- **TSPCT** (Bits 6-0): TS Voltage as percentage of REGN `R`
  - Offset: 21%
  - Range: 21% - 80%

### REG11 - VBUS Voltage Register
- **VBUS_GD** (Bit 7): VBUS Good Status `R`
  - 0: Not VBUS attached
  - 1: VBUS Attached
- **VBUSV** (Bits 6-0): VBUS Voltage Reading `R`
  - Offset: 2.6V
  - Range: 2.6V - 15.3V
  - Step size: 100mV

### REG12 - Charge Current Register
- **ICHGR** (Bits 6-0): Charge Current Reading `R`
  - Range: 0mA - 6350mA
  - Step size: 50mA
  - Note: Returns 0 for VBAT < VBATSHORT

### REG13 - VDPM/IDPM Status Register
- **VDPM_STAT** (Bit 7): VINDPM Status `R`
  - 0: Not in VINDPM
  - 1: VINDPM
- **IDPM_STAT** (Bit 6): IINDPM Status `R`
  - 0: Not in IINDPM
  - 1: IINDPM
- **IDPM_LIM** (Bits 5-0): Input Current Limit while ICO enabled `R`
  - Offset: 100mA
  - Range: 100mA - 3.25A
  - Step size: 50mA

### REG14 - Device Information Register
- **REG_RST** (Bit 7): Register Reset `R/W`
  - 0: Keep current register setting (default)
  - 1: Reset to default register value and reset safety timer
  - Note: Reset to 0 after register reset is completed
- **ICO_OPTIMIZED** (Bit 6): Input Current Optimizer Status `R`
  - 0: Optimization is in progress
  - 1: Maximum Input Current Detected
- **PN** (Bits 5-3): Device Configuration `R`
  - 000: bq25896
- **TS_PROFILE** (Bit 2): Temperature Profile `R`
  - 1: JEITA (default)
- **DEV_REV** (Bits 1-0): Device Revision `R`
  - 10: Device Revision

## Function Reference

[Add function documentation here]

## Examples

[Add usage examples here]

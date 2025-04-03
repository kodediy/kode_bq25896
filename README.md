# kode_bq25896

ESP-IDF component for the BQ25896 battery charger IC.

## Overview

The BQ25896 is a high-efficiency single-cell Li-Ion/Li-Polymer battery charger and system power path management IC from Texas Instruments. This ESP-IDF component provides a complete API for configuring and controlling the BQ25896 via I2C.

## Key Features

- **Advanced Charging Control**
  - Configurable fast charge current up to 3008mA
  - Precharge and termination current control
  - Dynamic input current optimization
  - Temperature-based charging profiles
  - Safety timer and watchdog protection

- **Input Power Management**
  - High impedance mode support
  - ILIM pin and I2C current limit control
  - Input voltage dynamic power management
  - USB and adapter input detection
  - Input current optimizer (ICO)

- **Safety Features**
  - Thermal regulation and protection
  - Battery voltage protection
  - Input voltage protection
  - Watchdog timer
  - Safety timers with extension support
  - NTC temperature monitoring

- **System Power Path Management**
  - Configurable system voltage regulation
  - Battery to system power path control
  - Minimum system voltage protection
  - Dynamic power source switching
  - VINDPM threshold control

- **Boost (OTG) Mode Operation**
  - Configurable output voltage
  - Current limit protection
  - Frequency selection (500kHz/1.5MHz)
  - Temperature monitoring
  - Battery voltage protection

- **Monitoring and Status**
  - ADC for voltage and current monitoring
  - Thermal regulation status
  - Charging status indication
  - Fault detection and reporting
  - Input source detection
  - Power good indication

- **Advanced Features**
  - Current pulse control (PUMPX)
  - IR compensation
  - JEITA temperature profile support
  - Boost mode frequency selection
  - Automatic and manual input detection
  - Shipping mode support

## Register Functionality

### REG00 - Input Source Control

The REG00 register controls how power is taken from the input source (USB, adapter, etc.).

#### Bit 7 (EN_HIZ) - High Impedance Mode

- **Function**: When enabled, disconnects the input power source by placing the input in a high-impedance state.
- **Use Cases**:
  - Battery-only operation (to force system to run from battery)
  - Input current isolation during testing
  - Power source switching (cleanly disconnect one source before connecting another)
- **Behavior**:
  - When set to 1: Input current is reduced to zero; charging stops
  - When set to 0 (default): Normal input current flow allowed

#### Bit 6 (EN_ILIM) - ILIM Pin Control

- **Function**: Enables or disables the external ILIM pin for hardware-based current limiting.
- **Use Cases**:
  - Hardware current limiting through a resistor connected to the ILIM pin
  - Dynamic current limiting through external circuitry
- **Behavior**:
  - When set to 1 (default): Current limit is determined by ILIM pin resistance
  - When set to 0: Current limit is determined solely by register settings (IINLIM)
  - The actual input current limit is the lower of the two values

#### Bits 5-0 (IINLIM) - Input Current Limit

- **Function**: Sets the maximum current that can be drawn from the input source.
- **Use Cases**:
  - Limiting current drawn from USB ports (100mA/500mA/900mA per USB spec)
  - Preventing overload of weak power adapters
  - Optimizing power draw based on the cable and source capabilities
- **Behavior**:
  - Range: 100mA (000000) to 3.25A (111111) in 50mA steps
  - Default: 500mA (001000) - Suitable for standard USB
  - After USB detection, values may be automatically updated:
    - USB SDP (Standard Downstream Port): Limited to 500mA
    - USB DCP (Dedicated Charging Port): Can go up to 3.25A

### REG01 - Power-On Configuration

The REG01 register configures temperature protection during boost (OTG) mode and the dynamic input voltage management system.

#### Bits 7-6 (BHOT) - Boost Mode Hot Temperature Threshold

- **Function**: Sets the temperature threshold for thermal protection during boost (OTG) mode.
- **Use Cases**:
  - Protecting the system when supplying power to external devices
  - Customizing thermal thresholds based on enclosure thermal characteristics
- **Behavior**:
  - 00 (default): Vbhot1 Threshold (34.75% of REGN) ≈ 60°C
  - 01: Vbhot0 Threshold (37.75% of REGN) ≈ 55°C
  - 10: Vbhot2 Threshold (31.25% of REGN) ≈ 65°C
  - 11: Thermal protection disabled (use with caution)
  - When temperature exceeds threshold, boost operation is suspended

#### Bit 5 (BCOLD) - Boost Mode Cold Temperature Threshold

- **Function**: Sets the cold temperature threshold for boost mode operation.
- **Use Cases**:
  - Preventing boost mode operation in extremely cold conditions
  - Battery protection in low-temperature environments
- **Behavior**:
  - 0 (default): Vbcold0 Threshold (77% of REGN) ≈ 0°C
  - 1: Vbcold1 Threshold (80% of REGN) ≈ -10°C
  - When temperature is below threshold, boost operation is suspended

#### Bits 4-0 (VINDPM_OS) - Input Voltage Limit Offset

- **Function**: Configures the dynamic power management threshold for input voltage.
- **Use Cases**:
  - Preventing brownouts when using weak power sources
  - Optimizing power draw from sources with high impedance
  - Ensuring stable operation with long or thin cables
- **Behavior**:
  - Range: 0mV (00000) to 3100mV (11111) in 100mV steps
  - Default: 600mV (00110)
  - The threshold is calculated as: VBUS - VINDPM_OS (for VBUS ≤ 6V)
  - For higher voltage inputs (VBUS > 6V), the offset is doubled
  - When VBUS drops below the calculated threshold, input current is reduced
  - This creates a dynamic response to input voltage droops

### REG02 - Charge Current Control

The REG02 register controls ADC conversion, boost mode frequency, and input source detection features.

#### Bit 7 (CONV_START) - ADC Conversion Control

- **Function**: Controls the start/stop of ADC conversion
- **Use Cases**:
  - Manual triggering of ADC measurements
  - Reading battery voltage, system voltage, and other analog values
- **Behavior**:
  - When set to 1: Starts ADC conversion
  - When set to 0: ADC conversion not active (default)
  - Read-only when CONV_RATE = 1 (continuous conversion mode)
  - Bit stays high during ADC conversion and input source detection

#### Bit 6 (CONV_RATE) - ADC Conversion Rate

- **Function**: Selects between one-shot or continuous ADC conversion
- **Use Cases**:
  - Continuous monitoring of system parameters
  - Power-efficient single measurements
- **Behavior**:
  - When set to 0: One-shot ADC conversion (default)
  - When set to 1: Continuous conversion with 1-second interval

#### Bit 5 (BOOST_FREQ) - Boost Mode Frequency

- **Function**: Selects the switching frequency for boost (OTG) mode
- **Use Cases**:
  - Optimizing efficiency vs. noise in boost mode
  - Reducing EMI in sensitive applications
- **Behavior**:
  - When set to 0: 1.5MHz operation (default)
  - When set to 1: 500kHz operation
  - Note: Write is ignored when OTG_CONFIG is enabled

#### Bit 4 (ICO_EN) - Input Current Optimizer

- **Function**: Enables/disables the Input Current Optimizer algorithm
- **Use Cases**:
  - Maximizing input current from power-limited sources
  - Optimizing charging efficiency
- **Behavior**:
  - When set to 1: ICO algorithm enabled (default)
  - When set to 0: ICO algorithm disabled
  - Automatically adjusts input current based on source capability

#### Bit 1 (FORCE_DPDM) - Force Input Detection

- **Function**: Forces Power Source Detection
- **Use Cases**:
  - Manual triggering of input source detection
  - Troubleshooting power source identification
- **Behavior**:
  - When set to 1: Forces PSEL detection
  - When set to 0: Not in PSEL detection (default)
  - Self-clears after detection is complete

#### Bit 0 (AUTO_DPDM) - Automatic Input Detection

- **Function**: Controls automatic power source detection when VBUS is connected
- **Use Cases**:
  - Automatic USB/adapter type detection
  - Plug-and-play charging configuration
- **Behavior**:
  - When set to 1: Enables automatic PSEL detection on VBUS connection (default)
  - When set to 0: Disables automatic detection
  - Helps configure appropriate charging parameters based on source type

All these functions are implemented with type-safe enumerations for better code reliability:
- `bq25896_adc_conv_state_t`: Controls ADC conversion start/stop
- `bq25896_adc_conv_rate_t`: Selects one-shot or continuous conversion
- `bq25896_boost_freq_t`: Sets boost mode frequency
- `bq25896_ico_state_t`: Controls Input Current Optimizer
- `bq25896_force_dpdm_state_t`: Controls manual input detection
- `bq25896_auto_dpdm_state_t`: Controls automatic input detection

Each function includes validation of input parameters, error handling, and logging for debugging purposes.

### REG03 - Charge Control

The REG03 register controls battery load, watchdog timer, charging modes, and system voltage thresholds.

#### Bit 7 (BAT_LOADEN) - Battery Load Enable

- **Function**: Controls the battery load (IBATLOAD) feature
- **Use Cases**:
  - Battery capacity testing
  - Discharge testing
  - Battery health assessment
- **Behavior**:
  - When set to 1: Enables battery load
  - When set to 0: Disables battery load (default)
  - Allows controlled discharge testing of the battery

#### Bit 6 (WD_RST) - Watchdog Timer Reset

- **Function**: Controls the I2C watchdog timer reset
- **Use Cases**:
  - Preventing system lockup
  - Maintaining charging safety
  - Ensuring communication integrity
- **Behavior**:
  - When set to 1: Resets the watchdog timer
  - When set to 0: Normal operation (default)
  - Auto-clears after timer reset
  - Must be periodically reset to prevent charging timeout

#### Bit 5 (OTG_CONFIG) - Boost Mode Configuration

- **Function**: Controls the OTG (boost) mode operation
- **Use Cases**:
  - USB OTG power supply
  - External device powering
  - Reverse charging applications
- **Behavior**:
  - When set to 1: Enables OTG mode
  - When set to 0: Disables OTG mode (default)
  - Converts battery voltage to regulated output voltage

#### Bit 4 (CHG_CONFIG) - Charge Enable Configuration

- **Function**: Controls the battery charging function
- **Use Cases**:
  - Battery charging control
  - System power management
  - Maintenance charging
- **Behavior**:
  - When set to 1: Enables charging (default)
  - When set to 0: Disables charging
  - Controls the main charging function of the device

#### Bits 3-1 (SYS_MIN) - Minimum System Voltage

- **Function**: Sets the minimum voltage that the system will regulate to
- **Use Cases**:
  - System brownout prevention
  - Battery over-discharge protection
  - Power path management
- **Behavior**:
  - Range: 3.0V (000) to 3.7V (111)
  - Step size: 100mV
  - Default: 3.5V (101)
  - Values:
    - 000: 3.0V
    - 001: 3.1V
    - 010: 3.2V
    - 011: 3.3V
    - 100: 3.4V
    - 101: 3.5V (default)
    - 110: 3.6V
    - 111: 3.7V

#### Bit 0 (MIN_VBAT_SEL) - Minimum Battery Voltage Selection

- **Function**: Sets the minimum battery voltage threshold for exiting boost mode
- **Use Cases**:
  - Battery protection during OTG operation
  - System stability management
  - Deep discharge prevention
- **Behavior**:
  - When set to 0: 2.9V threshold (default)
  - When set to 1: 2.5V threshold
  - Determines when boost mode operation stops to protect battery

All these functions are implemented with type-safe enumerations for better code reliability:
- `bq25896_bat_load_state_t`: Controls battery load feature
- `bq25896_wd_rst_state_t`: Controls watchdog timer reset
- `bq25896_otg_state_t`: Controls OTG (boost) mode
- `bq25896_chg_state_t`: Controls charging function
- `bq25896_sys_min_t`: Sets minimum system voltage
- `bq25896_min_vbat_sel_t`: Sets minimum battery voltage threshold

Each function includes input validation, error handling, and logging for debugging purposes, maintaining consistency with the existing register control implementations.

### REG04 - Fast Charge Current Control

The REG04 register controls the fast charge current and current pulse control features.

#### Bit 7 (EN_PUMPX) - Current Pulse Control Enable

- **Function**: Controls the current pulse control feature
- **Use Cases**:
  - Dynamic voltage adjustment
  - Efficiency optimization
  - Advanced charging control
- **Behavior**:
  - When set to 1: Enables current pulse control
  - When set to 0: Disables current pulse control (default)
  - Controls PUMPX_UP and PUMPX_DN functionality in REG09

#### Bits 6-0 (ICHG) - Fast Charge Current Limit

- **Function**: Sets the battery charging current during fast charge phase
- **Use Cases**:
  - Battery capacity matching
  - Thermal management
  - Charging speed control
  - Battery longevity optimization
- **Behavior**:
  - Range: 0mA to 3008mA
  - Step size: 64mA
  - Default: 2048mA (0x20)
  - Setting to 0mA (0x00) disables charging
  - Values above 3008mA are clamped to 3008mA
  - Actual current may be lower due to thermal or input current limitations

## Key Terminology

### REGN (Internal Regulated Voltage)

REGN is an internal regulated voltage rail within the BQ25896 that:

- Provides a stable voltage (typically 4.5V) for the chip's internal circuits
- Serves as a reference voltage for the thermal thresholds
- Powers the internal gate drivers and control logic
- Is used as the reference for temperature threshold percentages

When the datasheet mentions thresholds as percentages of REGN (e.g., "34.75% of REGN"), it means the voltage at the TS pin is compared to this fraction of the internal regulated voltage to determine if a temperature threshold has been exceeded.

### Temperature Sensing and REGN

The BQ25896 uses a resistor divider with an NTC (Negative Temperature Coefficient) thermistor to monitor temperature. As temperature increases, the NTC resistance decreases, causing the voltage at the TS pin to decrease as a percentage of REGN.

The chip compares the TS pin voltage to specific percentage thresholds of REGN:
- Hot thresholds (BHOT): Lower percentages (31-38% of REGN) indicate higher temperatures
- Cold thresholds (BCOLD): Higher percentages (77-80% of REGN) indicate lower temperatures

For example, when the TS pin voltage falls below 34.75% of REGN (approximately equivalent to 60°C at the NTC), the hot temperature threshold is triggered. Similarly, when the TS pin voltage rises above 77% of REGN (approximately 0°C), the cold temperature threshold is triggered.

### ADC Conversion System

The BQ25896's ADC (Analog-to-Digital Converter) system provides critical measurements for:
- Battery voltage (VBAT)
- System voltage (VSYS)
- Input voltage (VBUS)
- Charge current (ICHG)
- Temperature sensor voltage (TS)

The ADC system can operate in two modes:
- **One-shot Mode**: Single conversion triggered manually
- **Continuous Mode**: Automatic conversions every second

When a conversion is in progress, the CONV_START bit remains high until completion. This allows software to monitor the conversion status.

### Input Source Detection (DPDM)

The BQ25896 includes sophisticated input source detection through its DPDM system:

- **DPDM**: Data Plus (D+) and Data Minus (D-) lines detection
- Automatically identifies USB port types:
  - Standard Downstream Port (SDP)
  - Charging Downstream Port (CDP)
  - Dedicated Charging Port (DCP)
  - Various proprietary charging schemes

The detection can be triggered in two ways:
1. **Automatic** (AUTO_DPDM): Triggers when VBUS is connected
2. **Manual** (FORCE_DPDM): Software-initiated detection

### Input Current Optimization (ICO)

The Input Current Optimizer is an advanced feature that:

- Dynamically maximizes input current based on source capability
- Prevents source collapse by monitoring input voltage
- Works in conjunction with VINDPM for optimal power draw

ICO operation sequence:
1. Gradually increases input current
2. Monitors input voltage for signs of source stress
3. Finds and maintains the optimal current level
4. Periodically retries to find a higher current capability

### Boost Mode Operation

Boost mode (also called OTG mode) converts battery voltage to a regulated output voltage for powering external devices:

- **Frequency Selection**:
  - 1.5MHz: Higher efficiency, smaller components
  - 500kHz: Lower EMI, better stability in some applications
  
- **Protection Features**:
  - Over-current protection
  - Short-circuit protection
  - Temperature monitoring
  - Input voltage monitoring

The boost frequency selection (BOOST_FREQ) affects:
- Switching losses
- Component size requirements
- EMI/noise characteristics
- Overall efficiency

## How VINDPM Works

The VINDPM (Voltage Input Dynamic Power Management) system is a key feature that makes the BQ25896 suitable for a wide range of power sources:

1. The chip measures the input voltage (VBUS)
2. It subtracts the configured offset (VINDPM_OS)
3. If the input voltage falls below this threshold, the chip:
   - Gradually reduces the input current
   - Maintains a stable input voltage at the threshold
   - Prevents complete collapse of weak power sources

Example scenarios:
- **USB Port with Long Cable**: Setting VINDPM_OS to 600mV prevents voltage droop below 4.4V
- **Weak Solar Panel**: VINDPM automatically reduces current draw as voltage drops, operating at the optimal power point
- **12V Adapter**: With 600mV offset (doubled to 1.2V for VBUS > 6V), prevents input voltage from dropping below 10.8V

## Usage

For basic usage, see the examples directory. Here's a quick start:

```c
#include "kode_bq25896.h"

// Initialize
bq25896_handle_t handle;
bq25896_init(i2c_bus, BQ25896_I2C_ADDR_DEFAULT, &handle);

// Configure with default settings
bq25896_config_t config;
bq25896_get_default_config(&config);
bq25896_configure(handle, &config);

// Start charging
bq25896_start_charging(handle);
```

## License

Apache-2.0

### Power Path Management

The BQ25896's power path management system, controlled primarily through REG03, provides:

- **System Voltage Control**:
  - Maintains stable system voltage above minimum threshold (SYS_MIN)
  - Prevents system brownout during load transients
  - Manages power distribution between input source and battery
  - Supports 3.0V to 3.7V system operation range

- **Operating Modes**:
  1. Normal Charging Mode:
     - System powered from input source
     - Battery charging enabled
     - System voltage regulated above SYS_MIN
  2. Battery-Only Mode:
     - System powered from battery
     - No input source or HIZ mode enabled
     - Battery discharge controlled
  3. Boost (OTG) Mode:
     - Battery powers external devices
     - Regulated output voltage
     - Protected by MIN_VBAT threshold

### Watchdog Protection System

The BQ25896 implements a sophisticated watchdog system that:

- **Prevents System Lock-up**:
  - Requires periodic communication
  - Automatically resets to safe state if communication fails
  - Protects against software failures

- **Timer Operation**:
  - Must be reset periodically (WD_RST)
  - Configurable timeout periods
  - Automatic safety actions on timeout
  - Affects charging and configuration states

- **Safety Features**:
  - Automatic charge termination on timeout
  - Reset to default configuration
  - Fault reporting
  - Communication monitoring

### Battery Load Testing

The battery load (IBATLOAD) feature provides:

- **Controlled Discharge Testing**:
  - Programmable discharge current
  - Accurate capacity measurement
  - Battery health assessment
  - Performance verification

- **Applications**:
  - Capacity verification
  - Internal resistance measurement
  - Age-related degradation assessment
  - Quality control testing

### System Voltage Regulation

The system voltage regulation feature ensures:

- **Dynamic Voltage Control**:
  - Maintains minimum system voltage (SYS_MIN)
  - Prevents battery over-discharge
  - Supports various system requirements
  - Automatic source switching

- **Operation Modes**:
  - Buck mode (charging)
  - Boost mode (OTG)
  - Pass-through mode
  - Battery-only mode

- **Protection Features**:
  - Under-voltage lockout
  - Over-voltage protection
  - Current limiting
  - Thermal regulation

### Boost Mode Operation

The boost mode (OTG) feature includes:

- **Voltage Boost**:
  - Converts battery voltage to higher system voltage
  - Regulated output for external devices
  - Configurable voltage levels
  - Current-limited operation

- **Protection Mechanisms**:
  - Minimum battery voltage monitoring (MIN_VBAT_SEL)
  - Over-current protection
  - Thermal monitoring
  - Input voltage supervision

- **Applications**:
  - USB OTG power source
  - External device charging
  - Backup power systems
  - Portable power bank functionality
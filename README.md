# kode_bq25896

ESP-IDF component for the BQ25896 battery charger IC.

## Overview

The BQ25896 is a high-efficiency single-cell Li-Ion/Li-Polymer battery charger and system power path management IC from Texas Instruments. This ESP-IDF component provides a complete API for configuring and controlling the BQ25896 via I2C.

## Key Features

- Fully configurable battery charging
- Input current limiting and optimization
- OTG (boost) mode support
- Power path management
- JEITA temperature profile support
- Thermal regulation
- System voltage monitoring
- Fault handling

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
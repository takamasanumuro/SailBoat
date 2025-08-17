# Analog Voltage Generator (Sketch Environment)

This sketch provides a configurable analog voltage generator using PWM output with a command-line interface for precise voltage control.

## Features

- **Configurable voltage range**: Default 800mV to 4000mV (0.8V - 4.0V)
- **High-resolution PWM**: 10-bit resolution (1024 steps) using Timer1
- **Command-line interface**: UART/Serial control with intuitive commands
- **Real-time control**: Set voltage by percentage (0-100%) or absolute voltage (mV)
- **Calibration tools**: Built-in calibration sequence and test patterns

## Hardware Setup

### PWM Output Pin
- **Default Pin**: 9 (OC1A - Timer1 Output Compare A)
- **Alternative Pin**: 10 (OC1B - Timer1 Output Compare B)
- **PWM Frequency**: 1kHz (configurable)
- **Resolution**: 10-bit (0-1023 PWM values)

### Low-Pass Filter Circuit
You **MUST** add a low-pass filter after the PWM output to create a stable analog voltage:

```
Pin 9 ----[R]----+---- Analog Output
                 |
                [C]
                 |
                GND
```

**Recommended Values:**
- **R**: 1kΩ - 10kΩ 
- **C**: 1µF - 10µF
- **Cutoff Frequency**: f_c = 1/(2πRC) ≈ 16Hz - 159Hz

For 1kHz PWM with minimal ripple:
- **R = 4.7kΩ, C = 4.7µF** → f_c ≈ 7.2Hz

## Usage

### Compilation and Upload
```bash
# Compile and upload to Arduino Mega
pio run -e sketch -t upload

# Monitor serial output
pio device monitor -e sketch
```

### Serial Commands

Connect via serial terminal at **115200 baud**. Available commands:

| Command | Description | Example |
|---------|-------------|---------|
| `set <0-100>` | Set output percentage | `set 50` (50% = 2.4V) |
| `setp <0-100>` | Set output percentage (alias) | `setp 75` |
| `setv <voltage>` | Set voltage in mV | `setv 2400` (2.4V) |
| `range <min> <max>` | Set voltage range in mV | `range 500 3300` |
| `status` | Show current status | `status` |
| `config` | Show configuration | `config` |
| `cal` | Run calibration sequence | `cal` |
| `test` | Run test pattern (ramp) | `test` |
| `on` | Enable output | `on` |
| `off` | Disable output (0V) | `off` |
| `help` | Show all commands | `help` |

### Example Session
```
> set 0
Set to 0% (800 mV)

> set 50  
Set to 50% (2400 mV)

> setv 3200
Set to 3200 mV (75%)

> status
=== Status ===
Initialized: Yes
Enabled: Yes
Current Percentage: 75%
Current Voltage: 3200 mV
PWM Value: 767
=============

> range 1000 3000
Voltage range updated: 1000 - 3000 mV

> cal
Starting calibration sequence...
Setting 0% (1000 mV) - PWM: 0
Setting 25% (1500 mV) - PWM: 255
Setting 50% (2000 mV) - PWM: 511
Setting 75% (2500 mV) - PWM: 767
Setting 100% (3000 mV) - PWM: 1023
Calibration sequence complete
```

## Configuration

Default configuration (can be modified in `main.cpp`):

```cpp
AnalogVoltageGenerator::VoltageConfig config;
config.min_voltage_mv = 800.0f;     // 0.8V minimum
config.max_voltage_mv = 4000.0f;    // 4.0V maximum  
config.pwm_pin = 9;                 // Timer1 OC1A
config.pwm_frequency_hz = 1000;     // 1kHz PWM
config.pwm_resolution_bits = 10;    // 10-bit (0-1023)
```

## Technical Details

### PWM Calculation
- **PWM Value** = (Percentage / 100) × (2^resolution - 1)
- **Voltage** = min_voltage + (Percentage / 100) × (max_voltage - min_voltage)

### Timer1 Configuration
- **Mode**: Fast PWM (10-bit)
- **Prescaler**: 8 (for ~1kHz at 16MHz)
- **TOP Value**: 1023 (10-bit)
- **Actual Frequency**: 16MHz / (8 × 1024) ≈ 1.95kHz

### Memory Usage
- **RAM**: ~25% (2065 bytes)
- **Flash**: ~5% (12844 bytes)

## Applications

- **Sensor simulation**: Generate reference voltages for testing analog inputs
- **Control signal generation**: Provide setpoint voltages for analog control loops  
- **Calibration**: Generate known voltages for calibrating ADCs or sensors
- **Testing**: Validate analog circuit behavior across voltage ranges

## Safety Notes

⚠️ **Important Safety Considerations:**
- Maximum output voltage is limited by Arduino's 5V supply
- Do not exceed the configured voltage range
- Always use appropriate low-pass filtering for clean analog output
- Verify output voltage with multimeter before connecting to sensitive circuits
- PWM output without filtering may damage analog circuits expecting clean DC

## Troubleshooting

**No output voltage:**
- Check PWM pin connection (default pin 9)
- Verify low-pass filter circuit
- Use `status` command to check if generator is enabled
- Try `on` command to enable output

**Noisy/unstable output:**
- Improve low-pass filter (larger capacitor, smaller resistor)
- Check for proper grounding
- Reduce PWM frequency if needed

**Voltage range issues:**
- Use `range` command to set appropriate min/max values
- Check that requested voltage is within configured range
- Verify calculations with `config` and `status` commands
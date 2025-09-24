# Rudder H-Bridge Controller

Simple command-line interface for testing rudder H-bridge functionality with easy percentage and PWM input controls.

## Features

- **Percentage control**: -100% to +100% (port to starboard)
- **PWM input simulation**: 1000-2000 μs (like RC receiver input)
- **Real-time status display**: Shows direction, PWM values, and visual bar
- **Safety limits**: Respects H-bridge maximum PWM (240)

## Hardware Setup

### H-Bridge Connections
Default pin assignments (configurable in code):
- **Direction A (INA)**: Pin 22
- **Direction B (INB)**: Pin 23  
- **PWM**: Pin 6

### H-Bridge Control Logic
- **PWM > 0**: INA=HIGH, INB=LOW (Starboard/Right)
- **PWM < 0**: INA=LOW, INB=HIGH (Port/Left)
- **PWM = 0**: INA=LOW, INB=LOW (Coast/Stop)

## Usage

### Compilation and Upload
```bash
# Compile and upload to Arduino Mega
pio run -e rudder -t upload

# Monitor serial output
pio device monitor -e rudder
```

### Serial Commands

Connect via serial terminal at **115200 baud**:

| Command | Description | Range | Example |
|---------|-------------|-------|---------|
| `p <percentage>` | Set rudder percentage | -100 to +100 | `p -50` |
| `pwm <value>` | Set PWM input (RC style) | 1000 to 2000 | `pwm 1750` |
| `stop` | Stop rudder (center) | - | `stop` |
| `status` | Show current status | - | `status` |
| `help` | Show all commands | - | `help` |

### Control Mapping

#### Percentage Mode (-100 to +100)
- **-100%**: Full port (left) - PWM = -240
- **-50%**: Half port - PWM = -120  
- **0%**: Center - PWM = 0
- **+50%**: Half starboard - PWM = +120
- **+100%**: Full starboard (right) - PWM = +240

#### PWM Input Mode (1000-2000)
- **1000**: Full port (left) - equivalent to -100%
- **1250**: Half port - equivalent to -50%
- **1500**: Center - equivalent to 0%
- **1750**: Half starboard - equivalent to +50%
- **2000**: Full starboard (right) - equivalent to +100%

## Example Session

```
> p -75
Set rudder to -75%
=== Rudder Status ===
Percentage: -75%
PWM Input: 1125
H-Bridge PWM: -180
Direction: PORT (Left)
PWM Bar: [<<<<<<<---]
==================

> pwm 1800
Set rudder PWM input to 1800
=== Rudder Status ===
Percentage: 60%
PWM Input: 1800
H-Bridge PWM: 144
Direction: STARBOARD (Right)
PWM Bar: [------>>>>]
==================

> stop
Rudder stopped

> status
=== Rudder Status ===
Percentage: 0%
PWM Input: 1500
H-Bridge PWM: 0
Direction: CENTER (Stopped)
PWM Bar: [----------]
==================
```

## Configuration

Pin assignments can be changed in `main.cpp`:

```cpp
RudderController::Config config;
config.direction_pin_a = 22;  // INA pin
config.direction_pin_b = 23;  // INB pin  
config.pwm_pin = 6;           // PWM pin
config.max_pwm = 240;         // Max PWM value
```

## Safety Features

- **PWM limiting**: Automatically limits to ±240 PWM
- **Input validation**: Rejects invalid percentage/PWM values
- **Emergency stop**: `stop` command immediately centers rudder
- **Status monitoring**: Real-time feedback on position and direction

## Technical Details

### Conversion Formulas
```cpp
// Percentage to PWM
pwm = percentage * 240 / 100

// PWM Input to Percentage  
percentage = (pwm_input - 1500) / 5

// PWM Input to PWM
pwm = ((pwm_input - 1500) / 5) * 240 / 100
```

### Memory Usage
- **RAM**: ~24% (2004 bytes)
- **Flash**: ~4% (10456 bytes)

## Troubleshooting

**No rudder movement:**
- Check H-bridge power supply
- Verify pin connections (DIR_A, DIR_B, PWM)
- Ensure H-bridge driver board is properly connected
- Try different PWM values: `p 25` then `p -25`

**Rudder moves wrong direction:**
- Swap direction pins (INA ↔ INB) in configuration
- Or use negative values: `p -50` instead of `p 50`

**Compilation errors:**
- Ensure you're in the correct environment: `pio run -e rudder`
- Check that HBridgeDriver files are in src_rudder folder

## Applications

- **H-bridge testing**: Verify motor driver functionality
- **Rudder calibration**: Find mechanical limits and center position
- **RC integration testing**: Simulate RC receiver PWM signals
- **Manual control**: Direct rudder control for testing/debugging
- **System integration**: Test rudder as part of larger autopilot system

Perfect for testing rudder hardware before integrating into the main sailboat autopilot system!
# SailBoat Dual Environment Setup

This project is configured with two separate PlatformIO environments for different development purposes.

## Environments

### 🚢 Main Environment (`env:main`)
- **Purpose**: Production sailboat autopilot system
- **Source Directory**: `src_main/`
- **Build Type**: Release (optimized)
- **Features**:
  - Full MAVLink communication with Pixhawk
  - PID control for rudder, sail, and throttle
  - Sensor feedback and safety systems
  - Production-ready code

### 🧪 Sketch Environment (`env:sketch`)
- **Purpose**: Testing and experimentation
- **Source Directory**: `src_sketch/`
- **Build Type**: Debug (with symbols)
- **Features**:
  - Lightweight testing framework
  - Easy to modify for quick tests
  - Isolated from production code
  - Perfect for prototyping new features

## Usage

### Building for Main Environment
```bash
# Build main (production) environment
pio run -e main

# Upload to board
pio run -e main -t upload

# Monitor serial output
pio device monitor -e main
```

### Building for Sketch Environment
```bash
# Build sketch (testing) environment  
pio run -e sketch

# Upload to board
pio run -e sketch -t upload

# Monitor serial output
pio device monitor -e sketch
```

### IDE Integration
Most IDEs will show both environments in the project selector. Switch between them as needed.

## Compilation Flags

### Main Environment Flags
- `ENVIRONMENT_MAIN=1` - Identifies main environment
- `SERIAL_DEBUG=1` - Enables serial debugging
- `PID_DEBUG=1` - Enables PID controller debugging
- `-Wall -Wextra` - Enhanced compiler warnings

### Sketch Environment Flags  
- `ENVIRONMENT_SKETCH=1` - Identifies sketch environment
- `SERIAL_DEBUG=1` - Enables serial debugging
- `SKETCH_MODE=1` - Enables sketch-specific features
- `-Wall -Wextra` - Enhanced compiler warnings

## Directory Structure
```
SailBoat/
├── platformio.ini          # Dual environment configuration
├── src_main/               # Main production code
│   ├── main.cpp           # Production autopilot system
│   └── main.hpp           # Production headers
├── src_sketch/             # Sketch testing code
│   └── main.cpp           # Testing/experimentation code
├── lib/                   # Shared libraries
│   └── HBridgeDriver/     # Motor driver library
├── include/               # Shared headers
│   └── mavlink/           # MAVLink protocol headers
└── README_ENVIRONMENTS.md # This file
```

## Best Practices

1. **Keep environments separate**: Don't mix production and test code
2. **Use compilation flags**: Leverage `#ifdef` for environment-specific code
3. **Test in sketch first**: Prototype new features in sketch environment
4. **Production deployment**: Always use main environment for actual boat
5. **Version control**: Both environments are tracked in git

## Adding New Features

1. Start development in `src_sketch/`
2. Test and iterate quickly
3. Once stable, integrate into `src_main/`
4. Use compilation flags to conditionally enable features
5. Maintain backward compatibility in main environment

## Troubleshooting

**Environment not found**: Make sure you're specifying the correct environment name (`main` or `sketch`)

**Build errors**: Check that shared libraries and includes are accessible from both source directories

**Upload issues**: Verify the correct board is connected and environment is selected
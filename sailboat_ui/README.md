# Sailboat Control UI

A Python GUI application for controlling the integrated sailboat autopilot system via serial communication.

## Features

### Connection Management
- Automatic serial port detection
- Connection status indicator
- Configurable baud rate (default: 9600)

### Manual Control Tab
- **Rudder Control**: Slider for -100% to +100% rudder position
- **Motor Control**: Slider for 0% to 100% motor speed
- **Emergency Stop**: Immediate shutdown of all systems
- Real-time value display

### PID Control Tab
- **Angle Control**: Set target angle from -45° to +45°
- **PID Enable/Disable**: Toggle automatic angle control
- **PID Tuning**: Adjust Kp, Ki, Kd parameters
- **Preset Values**: Quick-load common PID configurations
- **Status Requests**: Get current system status

### Debug Tab
- **Serial Output**: Real-time display of all Arduino communication
- **Custom Commands**: Send any command directly to the Arduino
- **Quick Commands**: One-click access to common commands (status, help, etc.)
- **Auto-scroll**: Automatically scroll to newest messages
- **Clear Output**: Clear the debug window

## Installation

1. **Install Python 3.7 or higher**

2. **Install required packages**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the application**:
   ```bash
   python main.py
   ```

## Usage

1. **Connect to Arduino**:
   - Select the appropriate COM port from the dropdown
   - Click "Connect" button
   - Status should show "Connected" in green

2. **Manual Control**:
   - Use sliders to control rudder position and motor speed
   - Click "Set Rudder" or "Set Motor" to send commands
   - Use "Center" and "Stop" for quick positioning

3. **PID Control**:
   - Set desired angle with the slider
   - Click "Set Angle" to update target
   - Enable/disable PID control with checkbox
   - Tune PID parameters or use presets

4. **Debug Monitoring**:
   - All serial communication appears in the debug window
   - Send custom commands for testing
   - Use quick commands for common operations

## Supported Arduino Commands

The UI sends the following commands to the Arduino:

- `rudder <-100-100>` - Set rudder percentage
- `motor <0-100>` - Set motor percentage  
- `angle <degrees>` - Set target angle for PID
- `pid on/off` - Enable/disable PID control
- `tune <kp> <ki> <kd>` - Set PID parameters
- `status` - Get system status
- `stop` - Emergency stop all systems
- `help` - Show available commands

## System Requirements

- Python 3.7+
- tkinter (included with Python)
- pyserial library
- Windows/Linux/macOS compatible

## Architecture

- **SerialManager**: Handles all serial communication in a background thread
- **SailboatUI**: Main GUI interface with tabbed layout
- **SystemStatus**: Data structure for tracking system state
- **Threaded Design**: Non-blocking UI with real-time serial monitoring

## Safety Features

- Emergency stop button prominently displayed
- Connection status clearly indicated
- All commands logged in debug window
- Graceful disconnection on window close
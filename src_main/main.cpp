#include <Arduino.h>
#include "mavlink/mavlink.h"
#include "HBridgeDriver.hpp"
#include "main.hpp"
#include "config.hpp"
#include "actuators.hpp"
#include "pid_controller.hpp"
#include "logger.hpp"
#include "timing_manager.hpp"
#include "Callback.h"

// Global actuator instances
HBridgeDriver winchActuator = HBridgeDriver(Config::Pins::Winch::DIRECTION_A, Config::Pins::Winch::DIRECTION_B, Config::Pins::Winch::PWM);
HBridgeDriver rudderActuator = HBridgeDriver(Config::Pins::Rudder::DIRECTION_A, Config::Pins::Rudder::DIRECTION_B, Config::Pins::Rudder::PWM);  
HBridgeDriver throttleMotor = HBridgeDriver(Config::Pins::Throttle::DIRECTION_A, Config::Pins::Throttle::DIRECTION_B, Config::Pins::Throttle::PWM);

// Pixhawk communication arrays
const uint8_t pixHawkReadingPins[] = {Config::Pins::PixhawkInput::SAIL, Config::Pins::PixhawkInput::RUDDER, Config::Pins::PixhawkInput::THROTTLE};
constexpr uint8_t numberPixhawkPins = Config::Buffers::PIXHAWK_CHANNELS;
int16_t pixHawkReadingsPWM[numberPixhawkPins] = {Config::Calibration::Pixhawk::TRIM_PWM};

// Command variables
Signal<const char*> serialInputSignal;
int rudderAngleCommand = 0;
int throttleSpeedCommand = Config::Calibration::Pixhawk::TRIM_PWM;

// Function to parse serial commands for manual control
void ParseActuatorCommands(const char* message) {
	int length = strlen(message);
	if (length < 2) return;

	if (message[0] == 'r' || message[0] == 'R') {
		int angle = atoi(message + 1);
		constrain(angle, Config::Calibration::Rudder::MIN_ANGLE, Config::Calibration::Rudder::MAX_ANGLE);
		Logger::logActuatorCommand("rudder angle", angle);
		rudderAngleCommand = angle;
	}

	if (message[0] == 't' || message[0] == 'T') {
		int speed = atoi(message + 1);
		Logger::logActuatorCommand("throttle speed", speed);
		throttleSpeedCommand = speed;
	}
}

void setup() {
    Serial.begin(Config::System::SERIAL_BAUD_RATE);
    
    // Initialize logging system first
    Logger::init();
    
    #ifdef ENVIRONMENT_MAIN
        Logger::logSystemStart(Config::System::NAME, Config::System::VERSION);
    #endif
    
    Logger::log(Logger::INFO, "FBoat initializing");
	delay(1000);

    // Initialize timing manager
    TimingManager::init();
    
    // Initialize PID controllers
    PIDController::Controllers::initializeControllers();
    
    // Initialize all hardware using modular functions
    Actuators::initializeHardware();

    // Setup serial command parsing
	FunctionSlot<const char*> parseActuatorCommandsSlot(ParseActuatorCommands);
	serialInputSignal.attach(parseActuatorCommandsSlot);
	
	Logger::log(Logger::INFO, "System initialization complete");
}

void loop() {
	GetSerialInput();
	Actuators::capturePixhawkPulses();
	rudderAngleCommand = Actuators::convertPixhawkReadingToAngle(rudder);
	throttleSpeedCommand = Actuators::getPixhawkReading(throttle);
	PIDController::controlRudder(rudderAngleCommand);
	PIDController::controlThrottle(throttleSpeedCommand);
}

// MAVLink communication with Pixhawk
void MAVLinkToPixhawk(MAVLink_options option, float data) {
	if (!TimingManager::getMAVLinkTimer().isReady()) return;
	
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	switch (option) { 
		case MAVLink_options::rudder_angle:
		{
			mavlink_msg_param_set_pack(Config::Communication::MAVLink::ARDUINO_SYS_ID, 
									  Config::Communication::MAVLink::ARDUINO_COMP_ID, 
									  &msg, 
									  Config::Communication::MAVLink::PIXHAWK_SYS_ID, 
									  Config::Communication::MAVLink::PIXHAWK_COMP_ID, 
									  "RUDDER_ANGLE", data, MAV_VAR_FLOAT);
			Logger::logMAVLinkMessage("RUDDER_ANGLE", data);
		}
		break;

		case MAVLink_options::sail_angle:
		{
			mavlink_msg_param_set_pack(Config::Communication::MAVLink::ARDUINO_SYS_ID, 
									  Config::Communication::MAVLink::ARDUINO_COMP_ID, 
									  &msg, 
									  Config::Communication::MAVLink::PIXHAWK_SYS_ID, 
									  Config::Communication::MAVLink::PIXHAWK_COMP_ID, 
									  "SAIL_ANGLE", data, MAV_VAR_FLOAT);
			Logger::logMAVLinkMessage("SAIL_ANGLE", data);
		}
		break;

		case MAVLink_options::rudder_pwm:
		{
			mavlink_msg_param_set_pack(Config::Communication::MAVLink::ARDUINO_SYS_ID, 
									  Config::Communication::MAVLink::ARDUINO_COMP_ID, 
									  &msg, 
									  Config::Communication::MAVLink::PIXHAWK_SYS_ID, 
									  Config::Communication::MAVLink::PIXHAWK_COMP_ID, 
									  "RUDDER_PWM", data, MAV_VAR_FLOAT);
			Logger::logMAVLinkMessage("RUDDER_PWM", data);
		}
		break;

		case MAVLink_options::sail_pwm:
		{
			mavlink_msg_param_set_pack(Config::Communication::MAVLink::ARDUINO_SYS_ID, 
									  Config::Communication::MAVLink::ARDUINO_COMP_ID, 
									  &msg, 
									  Config::Communication::MAVLink::PIXHAWK_SYS_ID, 
									  Config::Communication::MAVLink::PIXHAWK_COMP_ID, 
									  "SAIL_PWM", data, MAV_VAR_FLOAT);
			Logger::logMAVLinkMessage("SAIL_PWM", data);
		}
		break;
	}  
	
	uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
	Serial3.write(buffer, length);
}

// Serial input processing
void GetSerialInput() {
    static char inputBuffer[Config::Buffers::SERIAL_INPUT_LENGTH];
    static int bufferIndex = 0;
    
	if (!Serial.available()) {
		return;
	}

	char input = Serial.read();
	if (input == '\r') return;

	if (input == '\n') {
		inputBuffer[bufferIndex] = '\0';
		bufferIndex = 0;
		serialInputSignal.fire(inputBuffer);
	} else {
		inputBuffer[bufferIndex++] = input;
		if (bufferIndex >= Config::Buffers::SERIAL_INPUT_LENGTH) {
			bufferIndex = 0;
		}
	}  
}
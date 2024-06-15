#include <Arduino.h>
#include "mavlink/mavlink.h"
#include "HBridgeDriver.hpp"
#include "main.hpp"
#include "Callback.h"

HBridgeDriver throttleMotor = HBridgeDriver(41, 40, 45);   // Digital/Digital/PWM order of pins
HBridgeDriver rudderActuator = HBridgeDriver(43, 42, 44);  // Digital/Digital/PWM order of pins
HBridgeDriver winchActuator = HBridgeDriver(39, 38, 46);   // Digital/Digital/PWM order of pins

constexpr uint8_t rudderPinPotentiometer = A6;
constexpr int16_t rudderAngleOffset = 0;  // To align the rudder to the bow of the boat
constexpr int16_t rudderADCMinThreshold = 308;  // Starboard -->
constexpr int16_t rudderADCMaxThreshold = 661;  // Port <--
constexpr float rudderMinAngle = -47.69; // Starboard
constexpr float rudderMaxAngle = 53.64;  // Port

constexpr uint8_t sailPinPotentiometer = A7;
constexpr int16_t sailAngleOffset = 0;  // To align the rudder to the bow of the boat
constexpr int16_t sailADCMinThreshold = 400; // 2V that corresponds to 90 sail degree
constexpr int16_t sailADCMaxThreshold = 800; // 4V that corresponds to zero sail degree
constexpr int16_t sailMinAngle = 0;
constexpr int16_t sailMaxAngle = 90;

constexpr float rudder_proportional_constant = 5.0f;
constexpr float rudder_integral_constant = 8.0f;
constexpr float sail_proportional_constant = 8.0f;
constexpr float sail_integral_constant = 15.0f;

constexpr uint16_t pixhawkMinimalPWM = 993;
constexpr uint16_t pixhawkMaximumPWM = 1986;
constexpr uint8_t pixHawkReadingPins[] = {sailInputPWMPin, rudderInputPWMPin, throttleInputPWMPin}; // Arduino pins that receive pixhawk output PWM to read with pulseIn() function
constexpr uint8_t numberPixhawkPins = sizeof pixHawkReadingPins / sizeof pixHawkReadingPins[0]; // Get the number of elements in the array
int16_t pixHawkReadingsPWM[numberPixhawkPins] = {1500}; // Array that stores the PWM control signal in microseconds coming from Pixhawk. Default initialized to middle servo trim of 1500us.
enum PrintSelection {
  PrintRudder = 0,
  PrintSail = 1,
  PrintThrottle = 2
};
uint8_t printSelectionIndex = PrintThrottle;

enum throttleSelectionIndex {
  MOTOR_OFF,
  MOTOR_FORWARD,
  MOTOR_OFF_2,
  MOTOR_BACK
};

int throttleSpeedTest = 50;
int throttleSelectionIndex = MOTOR_OFF;

// Flags for conditional compilation
#define PRINT_RUDDER 1
#define PRINT_SAIL 1
#define PRINT_THROTTLE 1
//#define PRINT_RUDDER_READINGS
//#define PRINT_SAIL_READINGS

void TestSelectionIndex() {
    static timer test_selection_timer = millis();
    if (millis() - test_selection_timer < 1000) return;
    test_selection_timer = millis();
    
    switch (printSelectionIndex) {
        case PrintRudder:
            Serial.println("RudderSelection");
            break;
        case PrintSail:
            Serial.println("SailSelection");
            break;
        case PrintThrottle:
            Serial.println("ThrottleSelection");
            break;
        default:
            Serial.println("InvalidSelection");
            break;
    }
}

void printPixhawkArray() {
    Serial.print("Input: ");
    for (auto& pixHawkReading : pixHawkReadingsPWM) {
        Serial.print(pixHawkReading); Serial.print("\t");
    }
    Serial.println();
}

int rudderAngleCommand = 0;
int throttleSpeedCommand = 1500;

void ParseActuatorCommands(const char* message) {
	int length = strlen(message);
	if (length < 2) return;

	if (message[0] == 'r' || message[0] == 'R') {
		int angle = atoi(message + 1);
		constrain(angle, rudderMinAngle, rudderMaxAngle);
		Serial.print("Setting rudder angle to: "); Serial.println(angle);
		rudderAngleCommand = angle;
	}

	if (message[0] == 't' || message[0] == 'T') {
		int speed = atoi(message + 1);
		Serial.print("Setting throttle speed to: "); Serial.println(speed);
		throttleSpeedCommand = speed;
	}
}

Signal<const char*> serialInputSignal;

void setup() {
    Serial.begin(115200);  // For PC communications
    Serial.print("FBoat initializing\n");
	delay(1000);

    throttleMotor.init_channel_A(); // Controls the throttle motor
	rudderActuator.init_channel_A(); // Controls the rudder actuator
	winchActuator.init_channel_A(); // Controls the winch actuator
    pinMode(rudderPinPotentiometer, INPUT); // Feedback for rudder PID
	pinMode(sailPinPotentiometer, INPUT); // Feedback for sail PID
    //for (auto& pixHawkReadingPin : pixHawkReadingPins) pinMode(pixHawkReadingPin, INPUT); // Input pins to read PWM control signals coming from Pixhawk

	FunctionSlot<const char*> parseActuatorCommandsSlot(ParseActuatorCommands);
	serialInputSignal.attach(parseActuatorCommandsSlot);

}

void originalLoop() {
	GetAllPixhawkReadings();
	RudderControl(GetPixhawkReadingToAngle(rudderIndex));
	SailControl(GetPixhawkReadingToAngle(sailIndex));
	ThrottleControl(GetPixhawkReading(throttleIndex));
	GetSerialInput();
}

void loop() {
	//ReadRudder();
	GetSerialInput();
	RudderControl(rudderAngleCommand);
	ThrottleControl(throttleSpeedCommand);
}

float PID_Proportional(float present_error, float proportional_gain) {
    return proportional_gain * present_error;
}

float PID_Integral(float present_error, float integral_gain) {
    static float integral_sum = 0;
    static timer end_time = millis(); 
    static timer begin_time = millis();

    end_time = millis();
    float cycle_time = static_cast<float>(end_time - begin_time) / 1000.f;
    begin_time = millis();

    integral_sum = integral_sum + integral_gain * present_error * cycle_time;
    integral_sum = constrain(integral_sum, -HBridgeDriver::maxPWM, HBridgeDriver::maxPWM); 
    return integral_sum;
}

float PID_Integral_Sail(float present_error, float integral_gain) {
    static int32_t integral_sum = 0;
    static timer end_time = millis(); 
    static timer begin_time = millis();
    end_time = millis();
    float cycle_time = static_cast<float>(end_time - begin_time) / 1000.f;
    begin_time = millis();
    integral_sum = integral_sum + integral_gain * present_error * cycle_time;
    integral_sum = constrain(integral_sum, -HBridgeDriver::maxPWM, HBridgeDriver::maxPWM); 
    return integral_sum;
}

void CheckRudderBoundaries(int rudder_current_angle) {
	if (rudder_current_angle < rudderMinAngle) { // if exceeded port, command to starboard
		while (ReadRudder() < rudderMinAngle) {
			rudderActuator.setPWM(-HBridgeDriver::maxPWM, HBridgeDriver::M1);
		}
	} else if (rudder_current_angle > rudderMaxAngle) { // if exceeded starboard, command to port
		while (ReadRudder() > rudderMaxAngle) {
			rudderActuator.setPWM(HBridgeDriver::maxPWM, HBridgeDriver::M1);
		}
	}
}

int calculatePWM(int angle_desired, int current_angle, int proportional_constant, int integral_constant, int max_pwm) {
    int angle_error = angle_desired - current_angle;
    int output_pwm = PID_Proportional(angle_error, proportional_constant) + PID_Integral(angle_error, integral_constant);
    output_pwm = constrain(output_pwm, -max_pwm, max_pwm); // Constrains output given H-bridge PWM output range
    output_pwm = -output_pwm; // Flips the sign of the output to match the rudder's direction
    if (angle_error > -2 && angle_error < 2) output_pwm = 0; //Prevents excessive microadjustments from the actuator

	#define DEBUG_PID
    #ifdef DEBUG_PID    
	static uint32_t log_timer = millis(); // This section logs information periodically to the serial port
	if (millis() - log_timer > 1500) {
		log_timer = millis();
		Serial.print("\nDesired angle: "); Serial.println(angle_desired);
		Serial.print("Current angle: "); Serial.println(current_angle);
		Serial.print("error: "); Serial.println(angle_error);
		Serial.print("PWM: "); Serial.println(output_pwm); 
	}
    #endif

	return output_pwm;
}

void RudderControl(int rudder_angle_desired) {  
    // port = negative PWM, starboard = positive PWM
    int rudder_current_angle = ReadRudder();
	CheckRudderBoundaries(rudder_current_angle);
    int rudder_output_pwm = calculatePWM(rudder_angle_desired, rudder_current_angle, rudder_proportional_constant, rudder_integral_constant, HBridgeDriver::maxPWM);
	rudderActuator.setPWM(rudder_output_pwm, HBridgeDriver::M1);
}

void SailControl(int sail_angle_desired) {
  	int sail_adc_reading = analogRead(sailPinPotentiometer);
  	if ((sail_adc_reading < 100 || sail_adc_reading > 900)) { // If the potentiometer is close to the end of its range, enters safety mode
  	  	Serial.println("Sail Potentiometer critical!");
  	  	winchActuator.setPWM(0, HBridgeDriver::M1);
  	  	return;
  	}
  	int sail_angle_present = map(sail_adc_reading, sailADCMinThreshold, sailADCMaxThreshold, sailMinAngle, sailMaxAngle);
  	int sail_error = sail_angle_desired - sail_angle_present;
  	int sail_output_pwm = PID_Proportional(sail_error, sail_proportional_constant) + PID_Integral_Sail(sail_error, sail_integral_constant);
  	sail_output_pwm = constrain(sail_output_pwm, -HBridgeDriver::maxPWM, HBridgeDriver::maxPWM);  // Constrains output given H-bridge PWM output range,
  	sail_output_pwm = -sail_output_pwm; // Flips the sign of the output to match the sail's direction
  	if (sail_error > -3 && sail_error < 3) sail_output_pwm = 0; //Prevents excessive microadjustments from the actuator
  	winchActuator.setPWM(sail_output_pwm, HBridgeDriver::M1); 
	
  	#ifdef PRINT_SAIL
  	if (printSelectionIndex == PrintSail) {
  	  	static uint32_t sail_log_timer = millis(); // This section logs information periodically to the serial port
  	  	if (millis() - sail_log_timer < 1500) return;
  	  	sail_log_timer = millis();
  	  	Serial.print("Sail ADC: "); Serial.println(sail_adc_reading);
  	  	Serial.print("\nSail desired: "); Serial.println(sail_angle_desired);
  	  	Serial.print("Sail present: "); Serial.println(sail_angle_present);
  	  	Serial.print("Sail error: "); Serial.println(sail_error);
  	  	Serial.print("Sail PWM: "); Serial.println(sail_output_pwm);
  	}
  	#endif
}

void ThrottleControl(int16_t throttle_signal_pwm) {
  	constexpr int16_t pixhawk_min_pwm = 975;
  	constexpr int16_t pixhawk_trim_pwm = 1500;
  	constexpr int16_t pixhawk_max_pwm = 2000;
  	constexpr int16_t pixhawk_throttle_deadzone = 200;

  	static int16_t previous_valid_throttle_signal = 1500;
	
  	int16_t raw_signal_debug = throttle_signal_pwm;

  	if (throttle_signal_pwm < pixhawk_min_pwm || throttle_signal_pwm > pixhawk_max_pwm) {
  	  	throttle_signal_pwm = previous_valid_throttle_signal;
  	}

  	if (throttle_signal_pwm > pixhawk_trim_pwm + pixhawk_throttle_deadzone) {
  	  	int output_pwm = map(throttle_signal_pwm, pixhawk_trim_pwm + pixhawk_throttle_deadzone, pixhawk_max_pwm, 0, HBridgeDriver::maxPWM);
  	  	throttleMotor.setPWM(output_pwm, HBridgeDriver::M1); 
  	} else if (throttle_signal_pwm < (pixhawk_trim_pwm - pixhawk_throttle_deadzone)) {
  	  	int output_pwm = map(throttle_signal_pwm, pixhawk_min_pwm, pixhawk_trim_pwm - pixhawk_throttle_deadzone, HBridgeDriver::maxPWM, 0);
  	  	throttleMotor.setPWM(-output_pwm, HBridgeDriver::M1);
  	} else {
  	  	throttleMotor.setPWM(0, HBridgeDriver::M1);
  	}

  	previous_valid_throttle_signal = throttle_signal_pwm;
}

int ReadRudder() {
  	int pot_rudder_ADC = analogRead(rudderPinPotentiometer);
  	int pot_rudder_angle = map(pot_rudder_ADC, rudderADCMinThreshold, rudderADCMaxThreshold, rudderMinAngle, rudderMaxAngle);
  	pot_rudder_angle += rudderAngleOffset;
  	
	//#define PRINT_RUDDER_READINGS
	#ifdef PRINT_RUDDER_READINGS
  	static uint32_t rudder_read_timer = millis();
  	if (millis() - rudder_read_timer < 1000) return pot_rudder_angle;
  	rudder_read_timer = millis();
  	Serial.print("Rudder ADC: ");   Serial.println(pot_rudder_ADC);
  	Serial.print("Rudder angle: "); Serial.println(pot_rudder_angle);
  	#endif
  	return pot_rudder_angle;
}

//2V correspondendo a vela em 90 graus/ 4V correspondendo a vela em 0 graus
int ReadSail() {
  	int pot_sail_ADC = analogRead(sailPinPotentiometer); //2V aqui        //4V Aqui            2V-->90 graus  4V-->10 graus
  	int pot_sail_angle = map(pot_sail_ADC, sailADCMinThreshold, sailADCMaxThreshold, sailMaxAngle, sailMinAngle);
  	pot_sail_angle += sailAngleOffset;
  	#ifdef PRINT_SAIL_READINGS
  	static uint32_t sail_read_timer = millis();
  	if (millis() - sail_read_timer < 3000) return pot_sail_angle;
  	sail_read_timer = millis();
  	Serial.print("Rudder ADC: ");   Serial.println(pot_sail_ADC);
  	Serial.print("Rudder angle: "); Serial.println(pot_sail_angle);
  	#endif
  	return pot_sail_angle;
}

void GetAllPixhawkReadings() {
  	for (int i = 0; i < numberPixhawkPins; i++) {
		pixHawkReadingsPWM[i] = pulseIn(pixHawkReadingPins[i], HIGH);
  	}
}

int16_t GetPixhawkReading(pixHawkChannels pixhawk_channel) {
  	for (int i = 0; i < numberPixhawkPins; i++) {
		pixHawkReadingsPWM[i] = pulseIn(pixHawkReadingPins[i], HIGH);
  	}
  	return pixHawkReadingsPWM[pixhawk_channel];
}

int16_t GetPixhawkReadingToAngle(pixHawkChannels pixhawk_channel) {
  	constexpr uint8_t safety_buffer = 10;
  	int16_t angle_desired = 0;
  	switch(pixhawk_channel) { 
  	  	case pixHawkChannels::rudderIndex:
  	  	  	angle_desired = map(pixHawkReadingsPWM[pixhawk_channel], pixhawkMinimalPWM, pixhawkMaximumPWM, rudderMinAngle, rudderMaxAngle);
  	  	  	angle_desired = constrain(angle_desired, rudderMinAngle + safety_buffer, rudderMaxAngle - safety_buffer);
  	  	  	break;
  	  	case pixHawkChannels::sailIndex: // Mapping is inverted for the sail eg 1000us = 0 degrees = 3V; 2000us = 90 degrees = 1V. Sail should be able to go to zero degrees, so no safety buffer is needed..
  	  	  	angle_desired = map(pixHawkReadingsPWM[pixhawk_channel], pixhawkMinimalPWM, pixhawkMaximumPWM, sailMinAngle, sailMaxAngle);
  	  	  	angle_desired = constrain(angle_desired, sailMinAngle, sailMaxAngle - safety_buffer);
  	  	  	break;
  	  	default: // Other options are not PID controlled by angle, so they do not join here.
  	  	  	Serial.print("Error: Trying to get angle from non-angle controlled channel: "); Serial.println(pixhawk_channel);
  	  	  	while(1){} // Infinite loop to prevent the code from continuing
  	  	  	break;
  	}
  	return angle_desired;
}

//envia dados para a pixhawk usando o protocolo MAVLINK
void MAVLinkToPixhawk(MAVLink_options option, float data) {
  	// Set up timer to publish at 1 Hz
  	static uint32_t MAV_publish_timer = 0;
  	static constexpr uint32_t MAV_publish_delay = 1000;
  	if (millis() - MAV_publish_timer < MAV_publish_delay) return;
  	MAV_publish_timer = millis();
  	 // Initialize the required buffers
  	mavlink_message_t msg;
  	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  	switch (option) { 
  	  	case MAVLink_options::rudder_angle:
  	  	{
  	  	  	mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id, "RUDDER_ANGLE", data, MAV_VAR_FLOAT);
  	  	  	Serial3.print("RUDDER_ANGLE: "); Serial.println(data);
  	  	}
  	  	break;

  	  	case MAVLink_options::sail_angle:
  	  	{
  	  	  	mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"SAIL_ANGLE", data, MAV_VAR_FLOAT);
  	  	  	Serial3.print("SAIL_ANGLE: "); Serial.println(data);
  	  	}
  	  	break;

  	  	case MAVLink_options::rudder_pwm:
  	  	{
  	  	  	mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"RUDDER_PWM", data, MAV_VAR_FLOAT);
  	  	  	Serial3.print("RUDDER_PWM: "); Serial.println(data);
  	  	}
  	  	break;

  	  	case MAVLink_options::sail_pwm:
  	  	{
  	  	  	mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"SAIL_PWM", data, MAV_VAR_FLOAT);
  	  	  	Serial3.print("SAIL_PWM: "); Serial.println(data);
  	  	}
  	  	break;
  	}  
	
  	// Copy the message to the send buffer
  	uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
  	// Send the message with the standard UART send function
  	Serial3.write(buffer, length);
}

void GetSerialInput() {
    constexpr int inputBufferLength = 256;
    static char inputBuffer[inputBufferLength];
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
		if (bufferIndex >= inputBufferLength) {
			bufferIndex = 0;
		}
	}  
}


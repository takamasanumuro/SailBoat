#include <Arduino.h>
#include "mavlink/mavlink.h"
#include "UN178Driver.hpp"
#include "RudderCheck.hpp"

                                                                                                                                                                                                                                                                                                                                                 #include "DualVNH5019MotorShield.h" //drive motor
UN178Driver sternBridge = UN178Driver(52, 53, 2, 5, 6, 7); // Stern motor and rudder respectively - Digital/Digital/PWM order of pins // Output pins for motor drivers
UN178Driver winchBridge = UN178Driver(8, 9, 10); // Winch motor - Digital/Digital/PWM order of pins // Output pins for motor drivers
constexpr uint8_t sailPinPotentiometer = A0;
constexpr uint8_t rudderPinPotentiometer = A1;
// Following constants were taken using a multimeter after setting the potentiometer to mid (5k) resistance and aligning the rudder to the center of the boat
constexpr int16_t rudderAngleOffset = 0;  // To align the rudder to the bow of the boat
constexpr int16_t sailAngleOffset = 0;  // To align the rudder to the bow of the boat
constexpr int16_t rudderADCMinThreshold = 320;  // Rudder -37 degrees (Starboard) 494
constexpr int16_t rudderADCMiddleThreshold = 516;
constexpr int16_t rudderADCMaxThreshold = 750;  // Rudder 57 degrees (Port) 554 = /
constexpr int16_t sailADCMinThreshold = 400; // 2V that corresponds to 90 sail degree
constexpr int16_t sailADCMaxThreshold = 800; // 4V that corresponds to zero sail degree

constexpr uint16_t pixhawkMinimalPWM = 993;
constexpr uint16_t pixhawkMaximumPWM = 1986;
constexpr int16_t rudderMinAngle = -45; // Rudder -45 degrees (Starboard) 
constexpr int16_t rudderMaxAngle = 45;  // Rudder 57 degrees (Port)
constexpr int16_t sailMinAngle = 0;
constexpr int16_t sailMaxAngle = 90;

constexpr float rudder_proportional_constant = 5.0f;
constexpr float rudder_integral_constant = 8.0f;
constexpr float sail_proportional_constant = 8.0f;
constexpr float sail_integral_constant = 15.0f;

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

void setup() {
  Serial.begin(115200);  // For PC communications
  Serial.print("Boat Rudder Test\n");
  sternBridge.init(); // Controls the rudder and the stern motor
  winchBridge.init_channel_A(); // Controls the winch, that actuates the sail rope.
  pinMode(rudderPinPotentiometer, INPUT); // Feedback for rudder PID
  pinMode(sailPinPotentiometer, INPUT); // Feedback for sail PID
  for (auto& pixHawkReadingPin : pixHawkReadingPins) pinMode(pixHawkReadingPin, INPUT); // Input pins to read PWM control signals coming from Pixhawk
}

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

void loop() {
  GetAllPixhawkReadings();
  RudderControl(GetPixhawkReadingToAngle(rudderIndex));
  SailControl(GetPixhawkReadingToAngle(sailIndex));
  ThrottleControl(GetPixhawkReading(throttleIndex));
  GetSerialInput();
}

float PID_Proportional(float present_error, float proportional_gain) {
  return proportional_gain*present_error;
}

float PID_Integral(float present_error, float integral_gain) {
  static int32_t integral_sum = 0;
  static timer end_time = millis(); 
  static timer begin_time = millis();
  end_time = millis();
  float cycle_time = static_cast<float>(end_time - begin_time) / 1000.f;
  begin_time = millis();
  integral_sum = integral_sum + integral_gain*present_error*cycle_time;
  integral_sum = constrain(integral_sum, -UN178Driver::maxPWM, UN178Driver::maxPWM); 
  return integral_sum;
}

float PID_Integral_Sail(float present_error, float integral_gain) {
  static int32_t integral_sum = 0;
  static timer end_time = millis(); 
  static timer begin_time = millis();
  end_time = millis();
  float cycle_time = static_cast<float>(end_time - begin_time) / 1000.f;
  begin_time = millis();
  integral_sum = integral_sum + integral_gain*present_error*cycle_time;
  integral_sum = constrain(integral_sum, -UN178Driver::maxPWM, UN178Driver::maxPWM); 
  return integral_sum;
}



void RudderControl(int rudder_angle_desired) {  
  int rudder_adc_reading = analogRead(rudderPinPotentiometer);
  int rudder_angle_present = map(rudder_adc_reading, rudderADCMinThreshold, rudderADCMaxThreshold, rudderMinAngle, rudderMaxAngle);
  rudder_angle_present += rudderAngleOffset;
  // port = negative PWM, starboard = positive PWM
  if (rudder_angle_present < rudderMinAngle) { // if exceeded port, command to starboard
    while (analogRead(rudderPinPotentiometer) < rudderADCMinThreshold) {
      sternBridge.setPWM(UN178Driver::maxPWM, UN178Driver::M2);
      static timer rudder_timer = millis();
      if (millis() - rudder_timer > 500) {
        rudder_timer = millis();
        Serial.print("Rudder ADC: "); Serial.println(rudder_adc_reading);
      }
    }
  } else if (rudder_angle_present > rudderMaxAngle) { // if exceeded starboard, command to port
    while (analogRead(rudderPinPotentiometer) > rudderADCMaxThreshold) {
      sternBridge.setPWM(-UN178Driver::maxPWM, UN178Driver::M2);
      static timer rudder_timer = millis();
      if (millis() - rudder_timer > 500) {
        rudder_timer = millis();
        Serial.print("Rudder ADC: "); Serial.println(rudder_adc_reading);
      }
    }
  }
  
  int rudder_error = rudder_angle_desired - rudder_angle_present;
  int rudder_output_pwm = PID_Proportional(rudder_error, rudder_proportional_constant) + PID_Integral(rudder_error, rudder_integral_constant);
  rudder_output_pwm = constrain(rudder_output_pwm, -UN178Driver::maxPWM, UN178Driver::maxPWM);// Constrains output given H-bridge PWM output range,
  rudder_output_pwm = -rudder_output_pwm; // Flips the sign of the output to match the rudder's direction
  if (rudder_error > -2 && rudder_error < 2) rudder_output_pwm = 0; //Prevents excessive microadjustments from the actuator
  sternBridge.setPWM(rudder_output_pwm, UN178Driver::M2);
  
  #ifdef PRINT_RUDDER
  if (printSelectionIndex == PrintRudder) {
    static uint32_t rudder_log_timer = millis(); // This section logs information periodically to the serial port
    if (millis() - rudder_log_timer < 1500) return;
    rudder_log_timer = millis();
    Serial.print("\nRudder ADC: "); Serial.println(rudder_adc_reading);
    Serial.print("Rudder desired: "); Serial.println(rudder_angle_desired);
    Serial.print("Rudder present: "); Serial.println(rudder_angle_present);
    Serial.print("Rudder error: "); Serial.println(rudder_error);
    Serial.print("Rudder PWM: "); Serial.println(rudder_output_pwm); 
  }
  #endif
}

void SailControl(int sail_angle_desired) {
  
  int sail_adc_reading = analogRead(sailPinPotentiometer);
  if ((sail_adc_reading < 100 || sail_adc_reading > 900)) { // If the potentiometer is close to the end of its range, enters safety mode
    Serial.println("Sail Potentiometer critical!");
    winchBridge.setPWM(0, UN178Driver::M1);
    return;
  }
  int sail_angle_present = map(sail_adc_reading, sailADCMinThreshold, sailADCMaxThreshold, sailMinAngle, sailMaxAngle);
  int sail_error = sail_angle_desired - sail_angle_present;
  int sail_output_pwm = PID_Proportional(sail_error, sail_proportional_constant) + PID_Integral_Sail(sail_error, sail_integral_constant);
  sail_output_pwm = constrain(sail_output_pwm, -UN178Driver::maxPWM, UN178Driver::maxPWM);  // Constrains output given H-bridge PWM output range,
  sail_output_pwm = -sail_output_pwm; // Flips the sign of the output to match the sail's direction
  if (sail_error > -3 && sail_error < 3) sail_output_pwm = 0; //Prevents excessive microadjustments from the actuator
  winchBridge.setPWM(sail_output_pwm, UN178Driver::M1); 
  
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

void ThrottleControl(int16_t throttle_signal) {
  static constexpr int16_t throttle_buffer = 200;
  static constexpr int16_t throttle_trim = 1500;
  static constexpr int16_t min_pixhawk_pwm = 975;
  static constexpr int16_t max_pixhawk_pwm = 2000;
  static uint32_t control_index = 0;
  static uint32_t error_counter = 0;
  static int16_t previous_throttle_signal = 1500;
  
  int16_t raw_signal_debug = throttle_signal;
  int16_t raw_signal_debug_constrain = throttle_signal;
  //throttle_signal = (9*throttle_signal + throttle_signal) / 10;
  if (throttle_signal < min_pixhawk_pwm || throttle_signal > max_pixhawk_pwm) { //! Random signals inside this band won't be processed
    //Serial.print("*******************************\n");
    //Serial.print("Throttle signal out of range: "); Serial.println(throttle_signal);
    //Serial.print("Setting previous PWM: "); Serial.println(previous_throttle_signal);
    //Serial.print("Error counter: "); Serial.println(error_counter);
    //Serial.print("Index: "); Serial.println(control_index);
    throttle_signal = previous_throttle_signal;
    control_index++;
    error_counter++;
    //Serial.print("*******************************\n");
    //Serial.print("\n");
    //sternBridge.setPWM(previous_throttle_signal, UN178Driver::M1); //!Solavanco com motor ativado e passivo
    //sternBridge.setPWM(0, UN178Driver::M1); //! Causa solavanco com motor ativado, mas n passivo
  }
  previous_throttle_signal = throttle_signal;
  throttle_signal = constrain(throttle_signal, 1000, 2000);
  raw_signal_debug_constrain = constrain(raw_signal_debug_constrain, 1000, 2000);
  if (throttle_signal > throttle_trim + throttle_buffer) {
    throttle_signal = map(throttle_signal, throttle_trim + throttle_buffer, 2000, 0, UN178Driver::maxPWM);
    sternBridge.setPWM(throttle_signal, UN178Driver::M1); 
    Serial.print("IF+ Throttle: "); Serial.println(throttle_signal);
    Serial.print("RAW: "); Serial.println(raw_signal_debug);
  }
  else if (throttle_signal < (throttle_trim - throttle_buffer)) {
    throttle_signal = map(throttle_signal, 1000, throttle_trim - throttle_buffer, UN178Driver::maxPWM, 0);
    //throttle_signal = -throttle_signal;
    sternBridge.setPWM(-throttle_signal, UN178Driver::M1);
    Serial.print("IF- Throttle: "); Serial.println(throttle_signal);
    Serial.print("RAW: "); Serial.println(raw_signal_debug);
  }
  else {
    sternBridge.setPWM(0, UN178Driver::M1);
  }

  //Serial.print("Prev throttle: "); Serial.println(previous_throttle_signal);
  
  #if !PRINT_THROTTLE
  if (printSelectionIndex == PrintThrottle) {
    static uint32_t throttle_read_timer = millis();
    if (millis() - throttle_read_timer < 600) return;
    throttle_read_timer = millis();
    Serial.print("Raw throttle signal: "); Serial.println(raw_signal_debug);
    Serial.print("Constrained throttle signal: "); Serial.println(raw_signal_debug_constrain);
    Serial.print("Output signal: "); Serial.println(throttle_signal);
    Serial.print("Index: "); Serial.println(control_index); Serial.print('\n');
    control_index++;
  }
  #endif 
}

int ReadRudder() {
  int pot_rudder_ADC = analogRead(rudderPinPotentiometer);
  int pot_rudder_angle = map(pot_rudder_ADC, rudderADCMinThreshold, rudderADCMaxThreshold, rudderMinAngle, rudderMaxAngle);
  pot_rudder_angle += rudderAngleOffset;
  #ifdef PRINT_RUDDER_READINGS
  static uint32_t rudder_read_timer = millis();
  if (millis() - rudder_read_timer < 3000) return pot_rudder_angle;
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


void ThrottleControlTest() {
  switch (throttleSelectionIndex) {
    case MOTOR_OFF:
      sternBridge.setPWM(throttleSpeedTest, UN178Driver::M1);
      break;
    case MOTOR_FORWARD:
      sternBridge.setPWM(throttleSpeedTest, UN178Driver::M1);
      break;
    case MOTOR_OFF_2:
      sternBridge.setPWM(throttleSpeedTest, UN178Driver::M1);
      break;
    case MOTOR_BACK:
      sternBridge.setPWM(-throttleSpeedTest, UN178Driver::M1);
      break;
  }
}

void PrintTestThrottleVariables(uint32_t time_interval = 1000) {
  static uint32_t print_timer = 0;
  if (millis() - print_timer < time_interval) return;
  print_timer = millis();

  Serial.print("Throttle selection index: "); Serial.println(throttleSelectionIndex);
  Serial.print("Throttle speed test: "); Serial.println(throttleSpeedTest);
}

void GetSerialInput() {
  static char incomingByte = '\0';
  if(Serial.available() > 0) {
    incomingByte = Serial.read();
    switch(incomingByte) {
      case 'p':
        ++printSelectionIndex;
        if (printSelectionIndex > PrintSelection::PrintThrottle) printSelectionIndex = PrintSelection::PrintRudder;
        Serial.print("Print selection: "); Serial.println(printSelectionIndex);
        break;
      case 't':
        throttleSpeedTest += 50;
        if (throttleSpeedTest > 255) {
          throttleSpeedTest = 50;
        }
        break;
      case 'm':
        throttleSelectionIndex++;
        if (throttleSelectionIndex > MOTOR_BACK) {
          throttleSelectionIndex = MOTOR_OFF;
        }
        break;
    }
  }
}


#pragma once
#include "mavlink/mavlink.h"
enum MAVLink_options {
  rudder_pwm = 0,
  rudder_angle = 1,
  sail_pwm = 2,
  sail_angle = 3,
};

enum pixHawkChannels {
  sail = 0, 
  rudder = 1, 
  throttle = 2,
  sailInputPWMPin = A8,
  rudderInputPWMPin = A9,
  throttleInputPWMPin = A10
};

typedef uint32_t timer;
static constexpr uint8_t arduino_sys_id = 2;
static constexpr uint8_t arduino_comp_id = 1;
static constexpr uint8_t pixhawk_sys_id = 1;
static constexpr uint8_t pixhawk_comp_id = 1;


int ReadRudder();
void MoveRudder();
void GetSerialInput();
void CapturePixhawkPulses();
int16_t GetPixhawkReading(pixHawkChannels pixhawk_channel);
int16_t GetPixhawkReadingToAngle(pixHawkChannels pixhawk_channel);
int16_t GetSailAngleFromReading(pixHawkChannels pixhawk_channel);
int ReadSail();
int ReadSailRaw();
int ReadRudder();
float PID_Proportional(float present_error, float proportional_gain);
float PID_Integral(float present_error, float integral_gain);
float PID_Integral_Sail(float present_error, float integral_gain);
void RudderControl(int rudder_angle_desired);
void SailControl(int sail_angle_desired);
void ThrottleControl(int16_t throttle_signal);
void MAVLinkToPixhawk(MAVLink_options option, float data);



#include "UN178Driver.hpp"

UN178Driver::UN178Driver(unsigned char INA1,
                        unsigned char INB1,
                        unsigned char PWM1) { 
  _INA1 = INA1;
  _INB1 = INB1;
  _PWM1 = PWM1;
}

UN178Driver::UN178Driver(unsigned char INA1,
                        unsigned char INB1,
                        unsigned char PWM1,
                        unsigned char INA2,
                        unsigned char INB2,
                        unsigned char PWM2) {
  _INA1 = INA1;
  _INB1 = INB1;
  _PWM1 = PWM1;
  _INA2 = INA2;
  _INB2 = INB2;
  _PWM2 = PWM2;
}

void UN178Driver::init() {
  pinMode(_INA1,OUTPUT);
  pinMode(_INB1,OUTPUT);
  pinMode(_PWM1,OUTPUT);
  pinMode(_INA2,OUTPUT);
  pinMode(_INB2,OUTPUT);
  pinMode(_PWM2,OUTPUT);
}

void UN178Driver::init_channel_A() {
  pinMode(_INA1,OUTPUT);
  pinMode(_INB1,OUTPUT);
  pinMode(_PWM1,OUTPUT);
}

void UN178Driver::setPWM(int pwm, UN178Channel channel) {
  unsigned char reverse = 0;
  if (pwm < 0) {
    pwm = -pwm;  // Make pwm a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (pwm > maxPWM) pwm = maxPWM;  // Max PWM dutycycle
  
  switch (channel) {
    case M1:
      analogWrite(_PWM1, pwm);
      if (pwm == 0) {
        digitalWrite(_INA1,LOW);   // Make the motor coast no
        digitalWrite(_INB1,LOW);   // matter which direction it is spinning.
      } else if (reverse)
      {
        digitalWrite(_INA1,LOW);
        digitalWrite(_INB1,HIGH);
      } else {
        digitalWrite(_INA1,HIGH);
        digitalWrite(_INB1,LOW);
      }
      break;
    case M2:
      analogWrite(_PWM2,pwm); 
      if (pwm == 0) {
        digitalWrite(_INA2,LOW);   // Make the motor coast no
        digitalWrite(_INB2,LOW);   // matter which direction it is spinning.
      }
      else if (reverse) {
        digitalWrite(_INA2,LOW);
        digitalWrite(_INB2,HIGH);
      } else {
        digitalWrite(_INA2,HIGH);
        digitalWrite(_INB2,LOW);
      }
      break;
  }
}



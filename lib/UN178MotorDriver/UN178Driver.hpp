#pragma once
#include <Arduino.h>

class UN178Driver {
public:
    UN178Driver(unsigned char INA1,
                unsigned char INB1,
                unsigned char PWM1);
                
    UN178Driver(unsigned char INA1,
                unsigned char INB1,
                unsigned char PWM1,
                unsigned char INA2,
                unsigned char INB2,
                unsigned char PWM2);
    void init();
    void init_channel_A();
    enum UN178Channel : uint8_t {
        M1 = 0,
        M2 = 1
    }; 
    void setPWM(int pwm, UN178Channel channel);
    constexpr static int16_t maxPWM = 240; // Max PWM allowed for 98% of duty cycle at bits    
  
private:
    unsigned char _INA1;
    unsigned char _INB1;
    unsigned char _PWM1;
    unsigned char _INA2;
    unsigned char _INB2;
    unsigned char _PWM2;              
};


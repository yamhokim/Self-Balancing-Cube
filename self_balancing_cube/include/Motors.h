#ifndef MOTORS_H
#define MOTORS_H

#include "Standard_Imports.h"

namespace Motors {
    // TODO: define motor pins
    extern const unsigned int motor1_pin1;
    extern const unsigned int motor1_pin2;
    extern const unsigned int motor1_pin_enable;
    extern const unsigned int pwmChannel1;
    
    extern const unsigned int motor2_pin1;
    extern const unsigned int motor2_pin2;
    extern const unsigned int motor2_pin_enable;
    extern const unsigned int pwmChannel2;

    extern const unsigned int motor3_pin1;
    extern const unsigned int motor3_pin2;
    extern const unsigned int motor3_pin_enable;
    extern const unsigned int pwmChannel3;

    extern const int PWM_freq;
    extern const int PWM_res;
    extern const int min_duty_cycle;
    extern const int max_duty_cycle;
    extern int duty_cycle;

    void init();
    bool setMotorSpeed(int motor, int speed);  
}

#endif
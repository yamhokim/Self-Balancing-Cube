#ifndef MOTORS_H
#define MOTORS_H

#include "Standard_Imports.h"

namespace Motors {
    // TODO: define motor pins
    const unsigned int motor1_pin1 = 0;
    const unsigned int motor1_pin2 = 0;
    const unsigned int motor1_pin_enable = 0;
    const unsigned int pwmChannel1 = 0;
    
    const unsigned int motor2_pin1 = 0;
    const unsigned int motor2_pin2 = 0;
    const unsigned int motor2_pin_enable = 0;
    const unsigned int pwmChannel2 = 1;

    const unsigned int motor3_pin1 = 0;
    const unsigned int motor3_pin2 = 0;
    const unsigned int motor3_pin_enable = 0;
    const unsigned int pwmChannel3 = 2;

    const int PWM_freq = 5000;
    const int PWM_res = 8;
    const int min_duty_cycle = 0;
    const int max_duty_cycle = pow(2, PWM_res) - 1;
    int duty_cycle = 0;

    void init();
    bool setMotorSpeed(int motor, int speed);  
}

#endif
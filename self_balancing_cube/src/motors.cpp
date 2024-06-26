#include "Motors.h"

namespace Motors {
    const unsigned int motor1_pin1 = 26;
    const unsigned int motor1_pin2 = 25;
    const unsigned int motor1_pin_enable = 15;
    const unsigned int pwmChannel1 = 0;
    
    const unsigned int motor2_pin1 = 18;
    const unsigned int motor2_pin2 = 19;
    const unsigned int motor2_pin_enable = 5;
    const unsigned int pwmChannel2 = 3;

    const unsigned int motor3_pin1 = 27;
    const unsigned int motor3_pin2 = 14;
    const unsigned int motor3_pin_enable = 12;
    const unsigned int pwmChannel3 = 6;

    const int PWM_freq = 3000;
    const int PWM_res = 8;
    const int min_duty_cycle = 0;
    const int max_duty_cycle = 230;//pow(2, PWM_res) - 1;
    int duty_cycle = 0;

    void init() {
        // Initialize motors
        pinMode(motor1_pin1, OUTPUT);
        pinMode(motor1_pin2, OUTPUT);
        pinMode(motor1_pin_enable, OUTPUT);
        ledcSetup(pwmChannel1, PWM_freq, PWM_res);
        ledcAttachPin(motor1_pin_enable, pwmChannel1);

        pinMode(motor2_pin1, OUTPUT);
        pinMode(motor2_pin2, OUTPUT);
        pinMode(motor2_pin_enable, OUTPUT);
        ledcSetup(pwmChannel2, PWM_freq, PWM_res);
        ledcAttachPin(motor2_pin_enable, pwmChannel2);

        pinMode(motor3_pin1, OUTPUT);
        pinMode(motor3_pin2, OUTPUT);
        pinMode(motor3_pin_enable, OUTPUT);
        ledcSetup(pwmChannel3, PWM_freq, PWM_res);
        ledcAttachPin(motor3_pin_enable, pwmChannel3);

        Serial.println("Motors initialized");
    }


    bool setMotorSpeed(int motor, int speed) {
        // Make sure speed is within limits
        double constrained_speed = constrain(abs(speed), min_duty_cycle, max_duty_cycle);
        
        // Set direction
        if (speed < 0) {
            if (motor == 1) {
                digitalWrite(motor1_pin1, HIGH);
                digitalWrite(motor1_pin2, LOW);
            } else if (motor == 2) {
                digitalWrite(motor2_pin1, HIGH);
                digitalWrite(motor2_pin2, LOW);
            } else if (motor == 3) {
                digitalWrite(motor3_pin1, HIGH);
                digitalWrite(motor3_pin2, LOW);
            } else {
                Serial.println("Invalid motor number");
                return false;
            }
        } else {
            if (motor == 1) {
                digitalWrite(motor1_pin1, LOW);
                digitalWrite(motor1_pin2, HIGH);
            } else if (motor == 2) {
                digitalWrite(motor2_pin1, LOW);
                digitalWrite(motor2_pin2, HIGH);
            } else if (motor == 3) {
                digitalWrite(motor3_pin1, LOW);
                digitalWrite(motor3_pin2, HIGH);
            } else {
                Serial.println("Invalid motor number");
                return false;
            }
        }

        // Set speed
        if (motor == 1) {
            ledcWrite(pwmChannel1, int(constrained_speed));
        } else if (motor == 2) {
            ledcWrite(pwmChannel2, int(constrained_speed));
        } else if (motor == 3) {
            ledcWrite(pwmChannel3, int(constrained_speed));
        } else {
            Serial.println("Invalid motor number");
            return false;
        }

        return true;
    }   
}
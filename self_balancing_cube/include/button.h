#ifndef BUTTON_H
#define BUTTON_H

#include "Standard_Imports.h"
#include "MPU6050_cust.h"

namespace Button {
    extern const unsigned int BUTTON_PIN;
    extern bool on;
    
    extern double roll_setpoint;
    extern double pitch_setpoint;
    extern double yaw_setpoint;

    void init();
    void buttonPressed();
    bool onState();
}

#endif
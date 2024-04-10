#include "button.h"

namespace Button {
    const unsigned int BUTTON_PIN = 7;
    bool on = false;

    double roll_setpoint = 0;
    double pitch_setpoint = 0;
    double yaw_setpoint = 0;

    void init() {
        pinMode(BUTTON_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, RISING);
        Serial.println("Button initialized");
    }

    void buttonPressed() {
        noInterrupts();
        // Calibrate setpoints
        Serial.println("Button pressed, calibrating setpoints...");
        std::vector<double> angles = MPU::readData();
        if (angles.empty()) {
            Serial.println("Failed to calibrate setpoints, system not turned on");
            interrupts();
            return;
        }
        roll_setpoint = angles[0];
        pitch_setpoint = angles[1];
        yaw_setpoint = angles[2];

        Serial.println("Setpoints calibrated");

        on = !on;
        interrupts();
        return;
    }

    bool onState() {
        return on;
    }
}
#include "button.h"

namespace Button {
    const unsigned int BUTTON_PIN = 33;
    bool on = false;

    void init() {
        pinMode(BUTTON_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, FALLING);
        Serial.println("Button initialized");
    }

    void buttonPressed() {
        noInterrupts();
        on = !on;
        interrupts();
    }

    bool onState() {
        return on;
    }
}
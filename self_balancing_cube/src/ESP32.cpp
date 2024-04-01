#include "ESP32.h"

namespace mcu {
    void init() {
        // Initialize serial ports on the ESP32
        Serial.begin(9600);
        Serial.println("Serial initialized");

        // Assertion check
        if (!Serial) {
            Serial.println("Failed to initialize serial");
            while (1) {
                delay(10);
            }
            throw std::exception();
        }
    }
}
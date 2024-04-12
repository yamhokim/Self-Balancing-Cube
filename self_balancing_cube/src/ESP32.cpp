#include "ESP32.h"

namespace mcu {
    void init() {
        // Initialize serial ports on the ESP32
        Serial.begin(9600);

        // Assertion check
        if (!Serial) {
            Serial.println("Failed to initialize serial");
            throw std::exception();
        }
        else {
            Serial.println("Serial initialized");
        }
    }
}
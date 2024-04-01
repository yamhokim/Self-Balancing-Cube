#include "MPU6050.h"

namespace MPU {

    Adafruit_MPU6050 mpu;

    void init() {
        // Initialize MPU6050
        if (!mpu.begin()) {
            Serial.println("Failed to find MPU6050 chip");
            while (1) {
                delay(10);
            }

            throw std::exception();
        }
        Serial.println("MPU6050 Found!");

        // Setup motion detection           // TODO: tune these values (also do these need to be assertion checked?)
        mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
        mpu.setMotionDetectionThreshold(1);
        mpu.setMotionDetectionDuration(20);
        mpu.setInterruptPinLatch(true);
        mpu.setInterruptPinPolarity(true);
        mpu.setMotionInterrupt(true);

        Serial.println("MPU6050 initialized");
        delay(100);
    }


    std::vector<double> readData() {
        std::vector<double> data;
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        data.push_back(a.acceleration.x);
        data.push_back(a.acceleration.y);
        data.push_back(a.acceleration.z);

        data.push_back(g.gyro.x);
        data.push_back(g.gyro.y);
        data.push_back(g.gyro.z);

        return data;
    }
}
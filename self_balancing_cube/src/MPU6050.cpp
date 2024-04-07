#include "MPU6050.h"

namespace MPU {

    Adafruit_MPU6050 mpu;
    std::vector<double> offset = {0.0, 0.0, 0.0, 0.00, 0.00, 0.00};

    void init() {
        // Initialize MPU6050
        Wire.begin();

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
        //mpu.setMotionDetectionThreshold(1);
        //mpu.setMotionDetectionDuration(LOOP_TIME);
        //mpu.setInterruptPinLatch(true);
        //mpu.setInterruptPinPolarity(true);
        //mpu.setMotionInterrupt(true);
    

        Serial.println("MPU6050 initialized");
        delay(100);

        //std::vector<double> calibration_readings = readData();
        //offset = calibration_readings;
        //Serial.println("MPU6050 offset calculated");
    }


    std::vector<double> readData() {
        std::vector<double> data;
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        data.push_back(a.acceleration.x + offset[0]);
        data.push_back(a.acceleration.y + offset[1]);
        data.push_back(a.acceleration.z + offset[2]);

        data.push_back(g.gyro.x + offset[3]);
        data.push_back(g.gyro.y + offset[4]);
        data.push_back(g.gyro.z + offset[5]);

        return data;
    }


    std::vector<double> calc_change(std::vector<double> data) {
        std::vector<double> change;
        for (int i = 0; i < data.size(); i++) {
            
            change.push_back((LOOP_TIME/100.0)*(data[i]));
            
        }
        return change;
    }
}
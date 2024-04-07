#include "MPU6050.h"

namespace MPU {

    MPU6050 mpu6050(Wire);

    void init() {
        // Initialize MPU6050
        Wire.begin();
        mpu6050.begin();
        mpu6050.calcGyroOffsets(true);
    }


    std::vector<double> readData() {
        std::vector<double> data;
        mpu6050.update();

        data.push_back(mpu6050.getAccX());
        data.push_back(mpu6050.getAccY());
        data.push_back(mpu6050.getAccZ());
        
        data.push_back(mpu6050.getAngleX());
        data.push_back(mpu6050.getAngleY());
        data.push_back(mpu6050.getAngleZ());
    }


    std::vector<double> calc_change(std::vector<double> data) {
        std::vector<double> change;
        for (int i = 0; i < data.size(); i++) {
            change.push_back((LOOP_TIME/100.0)*(data[i])); 
        }
        return change;
    }
}
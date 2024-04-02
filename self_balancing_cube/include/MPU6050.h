#ifndef MPU6050_H
#define MPU6050_H

#include "Standard_Imports.h"
#include <Adafruit_MPU6050.h>

#define LOOP_TIME 5

namespace MPU {
    extern Adafruit_MPU6050 mpu;

    void init();
    std::vector<double> readData();
    std::vector<double> calc_change(std::vector<double> data);
}

#endif
#ifndef MPU6050_H
#define MPU6050_H

#include "Standard_Imports.h"
#include <Adafruit_MPU6050.h>

namespace MPU {
    Adafruit_MPU6050 mpu;

    void init();
    std::vector<double> readData();
}

#endif
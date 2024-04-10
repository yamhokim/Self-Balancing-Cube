#ifndef MPU6050_H
#define MPU6050_H

#include "Standard_Imports.h"
#include <Adafruit_MPU6050.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

namespace MPU {
    void init();
    std::vector<double> readData();
    void dmpDataReady();
}

#endif
#ifndef MPU6050_H
#define MPU6050_H

#include "Standard_Imports.h"
#include <Adafruit_MPU6050.h>

namespace MPU {
    void init();
    std::vector<double> readData();
    std::vector<double> calc_change(std::vector<double> data);
}

#endif
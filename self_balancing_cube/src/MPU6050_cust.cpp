#include "MPU6050_cust.h"
#include "helper_3dmath.h"

// **********************************************************************************************************
    // MPU6050 READINGS ARE OBTAINED THORUGH THE I2CDEVLIB LIBRARY. THIS LIBRARY INCLUDES THE
    // FILES: I2Cdev.h, I2Cdev.cpp, MPU6050.h, MPU6050.cpp, helper_3dmath.h, MPU6050_6Axis_MotionApps612.h,
    // and MPU6050_6Axis_MotionApps612.cpp.

    // SOME OF THE MPU6050 INITIALIZATION CODE IS HEAVILY BASED ON EXAMPLE CODE FROM I2CDEVLIB.
    // THE EXAMPLE CODE CAN BE FOUND AT:
    // https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6_using_DMP_V6.12/MPU6050_DMP6_using_DMP_V6.12.ino
// **********************************************************************************************************

namespace MPU {

    MPU6050_6Axis_MotionApps612 mpu;
    int INTERRUPT_PIN = 2;
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Quaternion q;           // [w, x, y, z]         quaternion container
    float euler[3];         // [psi, theta, phi]    Euler angle container
    VectorInt16 gyro;       // [x, y, z]            gyro container

    void init() {

        // Initialize MPU6050
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

        // initialize device
        Serial.println(F("Initializing I2C devices..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);

        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(51);
        mpu.setYGyroOffset(8);
        mpu.setZGyroOffset(21);
        mpu.setXAccelOffset(1150);
        mpu.setYAccelOffset(-50);
        mpu.setZAccelOffset(1060);
        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // Calibration Time: generate offsets and calibrate our MPU6050
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            Serial.println();
            mpu.PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);

            // enable Arduino interrupt detection
            Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
            Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
            Serial.println(F(")..."));
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        }

        // Calibrate MPU6050
        Serial.println("Calibrating MPU6050...");
        std::vector<double> angles;
        for (int i = 0; i < 500; i++) {
            angles = MPU::readData();
            delay(10);
        }
        Serial.println("MPU6050 calibrated");

        return;
    }


    std::vector<double> readData() {
        std::vector<double> data;
        if (!dmpReady) {
            Serial.println("DMP not ready");
            return {};
        }

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            data.push_back(euler[2] * 180 / M_PI);
            data.push_back(euler[1] * 180 / M_PI);
            data.push_back(euler[0] * 180 / M_PI);

            mpu.dmpGetGyro(&gyro, fifoBuffer);
            data.push_back(gyro.x);
            data.push_back(gyro.y);
            data.push_back(gyro.z);
        }

        return data;
    }


    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    void dmpDataReady() {
        mpuInterrupt = true;
    }
}
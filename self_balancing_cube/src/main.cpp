#include "Standard_Imports.h"
#include "Motors.h"
#include "MPU6050.h"
#include "ESP32.h"

// Initialize variables
std::vector<double> mpu_data;
std::vector<double> motor_data;

const double loop_time = 0.0;
double last_time = 0.0;

double roll_err = 0.0;
double pitch_err = 0.0;
double yaw_err = 0.0;

double roll_err_sum = 0.0;
double pitch_err_sum = 0.0;
double yaw_err_sum = 0.0;

double roll_err_deriv = 0.0; 
double pitch_err_deriv = 0.0;
double yaw_err_deriv = 0.0;

double roll_err_deriv_filtered = 0.0;
double pitch_err_deriv_filtered = 0.0;
double yaw_err_deriv_filtered = 0.0;

double roll_ctrl = 0.0;
double pitch_ctrl = 0.0;
double yaw_ctrl = 0.0;

// TODO: tune all of these values
double kp= 0.0;
double ki= 0.0;
double kd= 0.0;

double alpha = 1.0;

double roll_setpoint = 0.0;
double pitch_setpoint = 0.0;
double yaw_setpoint = 0.0;

// ################# PID #################

void setup() {

  // Initialize serial ports on the ESP32
  mcu::init();

  // Initialize MPU6050
  MPU::init();

  // Initialize motors (pins and PWM probably)
  Motors::init();

  // TODO: might need a calibration sequence for reference values
  // ex. gyrodata might not be perfectly lined up with 0 when stationary

}

void loop() {
  // TODO: maybe add a start stop condition or something as an interrupt

  // ################# Enforce timer on loop #################
  if (millis() - last_time < loop_time) {
    return;
  }

  // ################# Read MPU6050 data #################
  if (MPU::mpu.getMotionInterruptStatus()) { // TODO: check if this is even needed, we might just always want to read everything
    mpu_data = MPU::readData(); // x, y, z -> roll, yaw, pitch (programmed like this, depends on orientation of MPU6050 on robot)
  }

  if (mpu_data.size() == 0) { // Ensure we have MPU data
    return;
  }

  // ################# Calculate PID #################
  // TODO: i think we could design the control so that it spins on a corner (like a holonomic robot) which is pretty cool
  // Kp
  roll_err = roll_setpoint - mpu_data[0];
  pitch_err = pitch_setpoint - mpu_data[1];
  yaw_err = yaw_setpoint - mpu_data[2];

  // Ki
  roll_err_sum += roll_err;
  pitch_err_sum += pitch_err;
  yaw_err_sum += yaw_err;

  // Kd 
  roll_err_deriv = (roll_err - roll_err_deriv) / loop_time;
  pitch_err_deriv = (pitch_err - pitch_err_deriv) / loop_time;
  yaw_err_deriv = (yaw_err - yaw_err_deriv) / loop_time;

  roll_err_deriv_filtered = alpha * roll_err_deriv + (1-alpha) * roll_err_deriv_filtered;
  pitch_err_deriv_filtered = alpha * pitch_err_deriv + (1-alpha) * pitch_err_deriv_filtered;
  yaw_err_deriv_filtered = alpha * yaw_err_deriv + (1-alpha) * yaw_err_deriv_filtered;

  // Control values
  roll_ctrl = kp * roll_err + ki * roll_err_sum + kd * roll_err_deriv_filtered;
  pitch_ctrl = kp * pitch_err + ki * pitch_err_sum + kd * pitch_err_deriv_filtered;
  yaw_ctrl = kp * yaw_err + ki * yaw_err_sum + kd * yaw_err_deriv_filtered;

  // ################# Set motor speeds #################
  // The current math here essentially assumes we attach the MPU aligned to the face of the cube
  // i.e. it is already in the reference frame of the wheels
  // ALSO: we might need to invert some of these values depending on how the motors are wired
  // The negative sign is because reaction force is equal and opposite, but just a matter of semantics really
  if(!Motors::setMotorSpeed(1, -roll_ctrl)) {
    Serial.println("Failed to set motor speed");
    return;
  };
  if(!Motors::setMotorSpeed(2, -pitch_ctrl)) {
    Serial.println("Failed to set motor speed");
    return;
  };
  if(!Motors::setMotorSpeed(3, -yaw_ctrl)) {
    Serial.println("Failed to set motor speed");
    return;
  
  };

  // ################# Update end time #################
  last_time = millis();

}

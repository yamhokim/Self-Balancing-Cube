#include "Standard_Imports.h"
#include "Motors.h"
#include "MPU6050_cust.h"
#include "ESP32.h"
#include "button.h"

// Initialize variables
std::vector<double> mpu_data;
std::vector<double> motor_data;

double last_time = 0.0;

double roll_curr = 0.0;
double pitch_curr = 0.0;
double yaw_curr = 0.0;
std::vector<double> angles;

double roll_err = 0.0;
double pitch_err = 0.0;
double yaw_err = 0.0;

double roll_err_sum = 0.0;
double pitch_err_sum = 0.0;
double yaw_err_sum = 0.0;
double windup_cap = 40.0;

double roll_err_deriv = 0.0; 
double pitch_err_deriv = 0.0;
double yaw_err_deriv = 0.0;
int deriv_count = 1;

double roll_err_deriv_filtered = 0.0;
double pitch_err_deriv_filtered = 0.0;
double yaw_err_deriv_filtered = 0.0;

double roll_ctrl = 0.0;
double pitch_ctrl = 0.0;
double yaw_ctrl = 0.0;

double kp= 250.0;
double ki= 0.03;
double kd= 2.0;

double alpha = 0.2;

// ################# PID #################

void setup() {

  // Initialize serial ports on the ESP32
  mcu::init();

  // Initialize MPU6050
  MPU::init();

  // Initialize motors
  Motors::init();

  // Initialize button
  Button::init();

  Serial.println("Setup complete");
}

void loop() {
  // ################# Enforce timer on loop #################
  if (millis() - last_time < LOOP_TIME or !Button::onState()) {
    return;
  }

  // ################# Read MPU6050 data #################
  mpu_data = MPU::readData();
  if (mpu_data.empty()) {
    last_time = millis();
    return;
  }

  // Update current angles

  roll_curr = mpu_data[0];
  pitch_curr = mpu_data[1];
  yaw_curr = mpu_data[2];

  // Ensure angles are within 0-360
  roll_curr = fmod(roll_curr, 360.0);
  pitch_curr = fmod(pitch_curr, 360.0);
  yaw_curr = fmod(yaw_curr, 360.0);

  // ################# Calculate PID #################
  // Kp
  noInterrupts();
  roll_err = roll_curr - Button::roll_setpoint;
  pitch_err = pitch_curr - Button::pitch_setpoint;
  yaw_err = yaw_curr - Button::yaw_setpoint;
  interrupts();

  // Ki
  roll_err_sum -= mpu_data[3];
  pitch_err_sum -= mpu_data[4];
  yaw_err_sum -= mpu_data[5];

  roll_err_sum = constrain(roll_err_sum, -windup_cap, windup_cap);
  pitch_err_sum = constrain(pitch_err_sum, -windup_cap, windup_cap);
  yaw_err_sum = constrain(yaw_err_sum, -windup_cap, windup_cap);

  // Kd 
  roll_err_deriv = -mpu_data[3];
  pitch_err_deriv = -mpu_data[4];
  yaw_err_deriv = -mpu_data[5];

  // IIR filter to smooth out derivative (too noisy)
  roll_err_deriv_filtered = alpha * roll_err_deriv + (1-alpha) * roll_err_deriv_filtered;
  pitch_err_deriv_filtered = alpha * pitch_err_deriv + (1-alpha) * pitch_err_deriv_filtered;
  yaw_err_deriv_filtered = alpha * yaw_err_deriv + (1-alpha) * yaw_err_deriv_filtered;

  // Control values
  roll_ctrl = kp * roll_err + ki * roll_err_sum + kd * roll_err_deriv_filtered;
  pitch_ctrl = kp * pitch_err + ki * pitch_err_sum + kd * pitch_err_deriv_filtered;
  yaw_ctrl = kp * yaw_err + ki * yaw_err_sum + kd * yaw_err_deriv_filtered;

  // ################# Set motor speeds #################
  if(!Motors::setMotorSpeed(1, yaw_ctrl)) {
    Serial.println("Failed to set motor speed");
    return;
  };
  if(!Motors::setMotorSpeed(2, pitch_ctrl)) {
    Serial.println("Failed to set motor speed");
    return;
  };
  if(!Motors::setMotorSpeed(3, roll_ctrl)) { //used to be yaw  +roll_ctrl_prev         
    Serial.println("Failed to set motor speed");
    return;
  };

  // ################# Update end time #################
  last_time = millis();
}

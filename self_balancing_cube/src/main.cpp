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
double windup_cap = 25.0;

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

double kp= 140.0;
double ki= 8;
double kd= 0.60;

double alpha = 0.2;

double roll_setpoint = 0.0;
double pitch_setpoint = 0.0;
double yaw_setpoint = 0.0;

bool setpoint_set = false;

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
  if (setpoint_set and !Button::onState()) {
    Serial.println("Control off");
    setpoint_set = false;

    // Stop motors
    Motors::setMotorSpeed(1, 0);
    Motors::setMotorSpeed(2, 0);
    Motors::setMotorSpeed(3, 0);
  }

  if (millis() - last_time < LOOP_TIME or !Button::onState()) {
    return;
  }

  if (Button::onState() and !setpoint_set) {
    angles = MPU::readData();
    if (angles.empty()) {
      return;
    }
    roll_setpoint = angles[0];
    pitch_setpoint = angles[1];
    yaw_setpoint = angles[2];
    setpoint_set = true;
    Serial.println("Control on, setpoint set");
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
  roll_err = roll_curr - roll_setpoint;
  pitch_err = pitch_curr - pitch_setpoint;
  yaw_err = yaw_curr - yaw_setpoint;

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
  //Serial.println(String(kp*roll_err) + " " + String(ki*roll_err_sum) + " " + String(kd*roll_err_deriv_filtered));
  // ################# Update end time #################
  last_time = millis();
}

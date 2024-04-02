#include "Standard_Imports.h"
#include "Motors.h"
#include "MPU6050.h"
#include "ESP32.h"

// Initialize variables
std::vector<double> mpu_data;
std::vector<double> motor_data;

//const double loop_time = 5; // in milliseconds
double last_time = 0.0;

double roll_curr = 0.0;
double pitch_curr = 0.0;
double yaw_curr = 0.0;
std::vector<double> angle_deltas;

double roll_err = 0.0;
double pitch_err = 0.0;
double yaw_err = 0.0;

double roll_err_prev = 0.0;
double pitch_err_prev = 0.0;
double yaw_err_prev = 0.0;

double roll_err_sum = 0.0;
double pitch_err_sum = 0.0;
double yaw_err_sum = 0.0;
double windup_cap = 10.0;

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
double kp= 500.0;
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
  // TODO: fix overflow possibility (i.e. millis() overflows)
  if (millis() - last_time < LOOP_TIME) {
    return;
  }

  // ################# Read MPU6050 data #################
  //if (MPU::mpu.getMotionInterruptStatus()) { // TODO: check if this is even needed, we might just always want to read everything
  mpu_data = MPU::readData(); // x, y, z -> roll, yaw, pitch (programmed like this, depends on orientation of MPU6050 on robot)
  //}else{
  //  return;
  //}

  // if (mpu_data.size() == 0) { // Ensure we have MPU data
  //   return;
  // }

  // Update current angles
  angle_deltas = MPU::calc_change(mpu_data);
  roll_curr += angle_deltas[3];
  pitch_curr += angle_deltas[4];
  yaw_curr += angle_deltas[5];

  roll_curr = fmod(roll_curr, 360.0);
  pitch_curr = fmod(pitch_curr, 360.0);
  yaw_curr = fmod(yaw_curr, 360.0);

  // ################# Calculate PID #################
  // TODO: i think we could design the control so that it spins on a corner (like a holonomic robot) which is pretty cool
  // Kp
  roll_err = roll_setpoint - roll_curr;
  pitch_err = pitch_setpoint - pitch_curr;
  yaw_err = yaw_setpoint - yaw_curr;

  // TODO: need to add integral windup check
  // Ki
  roll_err_sum += roll_err;
  pitch_err_sum += pitch_err;
  yaw_err_sum += yaw_err;

  roll_err_sum = constrain(roll_err_sum, -windup_cap, windup_cap);
  pitch_err_sum = constrain(pitch_err_sum, -windup_cap, windup_cap);
  yaw_err_sum = constrain(yaw_err_sum, -windup_cap, windup_cap);

  // Kd 
  roll_err_deriv = (roll_err - roll_err_prev) / LOOP_TIME;
  pitch_err_deriv = (pitch_err - pitch_err_prev) / LOOP_TIME;
  yaw_err_deriv = (yaw_err - yaw_err_prev) / LOOP_TIME;

  roll_err_deriv_filtered = alpha * roll_err_deriv + (1-alpha) * roll_err_deriv_filtered;
  pitch_err_deriv_filtered = alpha * pitch_err_deriv + (1-alpha) * pitch_err_deriv_filtered;
  yaw_err_deriv_filtered = alpha * yaw_err_deriv + (1-alpha) * yaw_err_deriv_filtered;

  // Control values
  roll_ctrl = kp * roll_err; // + ki * roll_err_sum + kd * roll_err_deriv_filtered;
  pitch_ctrl = kp * pitch_err; // + ki * pitch_err_sum + kd * pitch_err_deriv_filtered;
  yaw_ctrl = kp * yaw_err; // + ki * yaw_err_sum + kd * yaw_err_deriv_filtered;

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

  //Serial.println("Roll: " + String(roll_curr) + " Pitch: " + String(pitch_curr) + " Yaw: " + String(yaw_curr));// + "\n Roll Ctrl: " + String(roll_ctrl) + " Pitch Ctrl: " + String(pitch_ctrl) + " Yaw Ctrl: " + String(yaw_ctrl));
  // Serial.println("Roll Vel: " + String(mpu_data[3]) + " Pitch Vel: " + String(mpu_data[4]) + " Yaw Vel: " + String(mpu_data[5]));
  Serial.println("Roll ctrl" + String(roll_ctrl) + " Pitch ctrl" + String(pitch_ctrl) + " Yaw ctrl" + String(yaw_ctrl));
  // ################# Update end time #################
  last_time = millis();

}

#include "Standard_Imports.h"
#include "Motors.h"
#include "MPU6050_cust.h"
#include "ESP32.h"

// Initialize variables
std::vector<double> mpu_data;
std::vector<double> motor_data;

//const double loop_time = 5; // in milliseconds
double last_time = 0.0;

double roll_curr = 0.0;
double pitch_curr = 0.0;
double yaw_curr = 0.0;
std::vector<double> angles;

double roll_err = 0.0;
double pitch_err = 0.0;
double yaw_err = 0.0;

double roll_prev = 0.0;
double pitch_prev = 0.0;
double yaw_prev = 0.0;

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

// TODO: tune all of these values
double kp= 250.0; //225
double ki= 0.03;
double kd= 2.0;//.0005; //0

// 160, 0.03, 10.50

double alpha = 0.2;

double roll_setpoint = 0.0;
double pitch_setpoint = 0.0;
double yaw_setpoint = 0.0;

// TODO: tune deadband region
double deadband = 0;
double roll_ctrl_prev = 0.0;
double pitch_ctrl_prev = 0.0;
double yaw_ctrl_prev = 0.0;

// ################# PID #################

void setup() {

  // Initialize serial ports on the ESP32
  mcu::init();

  // Initialize MPU6050
  MPU::init();

  Motors::init();

  Serial.println("Starting setpoint calibration");
  for (int i = 0; i < 500; i++) {
    angles = MPU::readData();
    delay(10);
  }

  angles = MPU::readData();
  roll_setpoint = angles[0];
  pitch_setpoint = angles[1];
  yaw_setpoint = angles[2];

  Serial.println("Finished calibrating setpoints");

  // Initialize motors (pins and PWM probably)
  

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
  if (mpu_data.empty()) {
    last_time = millis();
    return;
  }
  
  //}else{
  //  return;
  //}

  // if (mpu_data.size() == 0) { // Ensure we have MPU data
  //   return;
  // }

  // Update current angles
  roll_prev = roll_curr;
  pitch_prev = pitch_curr;
  yaw_prev = yaw_curr;

  roll_curr = mpu_data[0];
  pitch_curr = mpu_data[1];
  yaw_curr = mpu_data[2];

  //roll_curr = fmod(roll_curr, 360.0);
  //pitch_curr = fmod(pitch_curr, 360.0);
  //yaw_curr = fmod(yaw_curr, 360.0);

  // ################# Calculate PID #################
  // TODO: i think we could design the control so that it spins on a corner (like a holonomic robot) which is pretty cool
  // Kp
  roll_err = roll_curr - roll_setpoint;
  pitch_err = pitch_curr - pitch_setpoint;
  yaw_err = yaw_curr - yaw_setpoint;

  // if (abs(roll_err) < deadband) {
  //   roll_err = 0;
  // }
  // if (abs(pitch_err) < deadband) {
  //   pitch_err = 0;
  // }
  // if (abs(yaw_err) < deadband) {
  //   yaw_err = 0;
  // }


  // TODO: need to add integral windup check
  // Ki
  // roll_err_sum += roll_err;
  // pitch_err_sum += pitch_err;
  // yaw_err_sum += yaw_err;

  roll_err_sum += mpu_data[3];
  pitch_err_sum += mpu_data[4];
  yaw_err_sum += mpu_data[5];

  roll_err_sum = constrain(roll_err_sum, -windup_cap, windup_cap);
  pitch_err_sum = constrain(pitch_err_sum, -windup_cap, windup_cap);
  yaw_err_sum = constrain(yaw_err_sum, -windup_cap, windup_cap);

  // Kd 
  // roll_err_deriv = (roll_curr - roll_prev) / LOOP_TIME;
  // pitch_err_deriv = (pitch_curr - pitch_prev) / LOOP_TIME;
  // yaw_err_deriv = (yaw_curr - yaw_prev) / LOOP_TIME;

  roll_err_deriv = -mpu_data[3];
  pitch_err_deriv = mpu_data[4];
  yaw_err_deriv = mpu_data[5];

  roll_err_deriv_filtered = alpha * roll_err_deriv + (1-alpha) * roll_err_deriv_filtered;
  pitch_err_deriv_filtered = alpha * pitch_err_deriv + (1-alpha) * pitch_err_deriv_filtered;
  yaw_err_deriv_filtered = alpha * yaw_err_deriv + (1-alpha) * yaw_err_deriv_filtered;

  // Control values
  roll_ctrl = kp * roll_err + ki * roll_err_sum + kd * roll_err_deriv_filtered;
  Serial.println("Curr speed " + String(roll_ctrl_prev) + " roll ctrl "+ String(roll_ctrl) + " prop ctrl " + String(kp*roll_err) + " int ctrl " + String(ki*roll_err_sum) + " derv ctrl " + String(kd*roll_err_deriv_filtered));
  pitch_ctrl = kp * pitch_err + ki * pitch_err_sum + kd * pitch_err_deriv_filtered;
  yaw_ctrl = kp * yaw_err + ki * yaw_err_sum + kd * yaw_err_deriv_filtered;

  // ################# Set motor speeds #################
  // The current math here essentially assumes we attach the MPU aligned to the face of the cube
  // i.e. it is already in the reference frame of the wheels
  // ALSO: we might need to invert some of these values depending on how the motors are wired
  // The negative sign is because reaction force is equal and opposite, but just a matter of semantics really
  

  // double max_speed_temp =245;
  // double offset_temp = 75;
  // double threshold = 1.5;
  // noInterrupts();
  // if (abs(roll_err) > threshold){
  //   if (roll_err < 0){
  //     Motors::setMotorSpeed(3, max_speed_temp);
  //   }else{
  //     Motors::setMotorSpeed(3, -max_speed_temp);
  //   }
  // }
  // else if (abs(roll_err) > 0.5) {
  //   if (roll_err < 0){
  //     Motors::setMotorSpeed(3, pow((roll_err/threshold),2)*(max_speed_temp-offset_temp)+offset_temp);
  //   }else{
  //     Motors::setMotorSpeed(3, -(pow((roll_err/threshold),2)*(max_speed_temp-offset_temp)+offset_temp));
  //   }
  // }
  // interrupts();
  // double print_temp = constrain((pow((roll_err/threshold),2)*(max_speed_temp-offset_temp)+offset_temp), 0, 255);
  // Serial.println("Roll err: " + String(roll_err) + " Roll ctrl: " + String(print_temp));
  // last_time = millis();
  // return;

  //roll_ctrl = constrain(roll_ctrl, -510, 510);
  if(!Motors::setMotorSpeed(1, -roll_ctrl+roll_ctrl_prev)) {
    Serial.println("Failed to set motor speed");
    return;
  };
  if(!Motors::setMotorSpeed(2, -pitch_ctrl+pitch_ctrl_prev)) {
    Serial.println("Failed to set motor speed");
    return;
  };
  if(!Motors::setMotorSpeed(3, roll_ctrl)) { //used to be yaw  +roll_ctrl_prev         
    Serial.println("Failed to set motor speed");
    return;
  };
  roll_ctrl_prev = constrain(roll_ctrl+roll_ctrl_prev, -255, 255);
  pitch_ctrl_prev = constrain(-pitch_ctrl+pitch_ctrl_prev, -255, 255);
  yaw_ctrl_prev = constrain(-yaw_ctrl+yaw_ctrl_prev, -255, 255);


  //Serial.println("Roll: " + String(roll_curr) + " Pitch: " + String(pitch_curr) + " Yaw: " + String(yaw_curr));// + "\n Roll Ctrl: " + String(roll_ctrl) + " Pitch Ctrl: " + String(pitch_ctrl) + " Yaw Ctrl: " + String(yaw_ctrl));
  // Serial.println("Roll Vel: " + String(mpu_data[3]) + " Pitch Vel: " + String(mpu_data[4]) + " Yaw Vel: " + String(mpu_data[5]));
  //Serial.print("Roll ctrl" + String(roll_ctrl) + " Pitch ctrl" + String(pitch_ctrl) + " Yaw ctrl" + String(yaw_ctrl));
  //Serial.print(" Roll err: " + String(roll_err) + " Pitch err: " + String(pitch_err) + " Yaw err: " + String(yaw_err));
  //Serial.println(" " + String(kp * roll_err) + ", " + String(kd * pitch_err_deriv_filtered));
  // ################# Update end time #################
  last_time = millis();

  // Just seeing if updating deriv less frequently results in better results
  deriv_count++;

}

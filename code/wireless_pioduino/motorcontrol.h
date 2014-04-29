/* motorcontrol.h -- handles the motors, encoders, and velocity outputs
   contains setSpeed() and drive() */
#include "Encoder.h"
#include "PID_v1.h"

//motor pins
const int rightMtrSpeedPin = 5;
const int leftMtrSpeedPin = 6;
const int rightMtrDirPin = 4;
const int leftMtrDirPin = 7;

//Encoder setup
//each encoder must have one interrupt enabled pin see ./Encoder.h
Encoder rightEncoder(2, 8); 
Encoder leftEncoder(3, 9);
int32_t encoderCountRight = 0;
int32_t encoderCountLeft = 0;
int32_t lastEncoderCountRight = 0;
int32_t lastEncoderCountLeft = 0;

//PID setup
const double kp = 0.3;
const double ki = 2.2;
const double kd = 0.002; 
double leftWheelDesired = 0;
double leftWheelMeasured = 0;
double leftWheelControl = 0;
double leftWheelRotational = 0;
double leftWheelLinear = 0;
double rightWheelDesired = 0;
double rightWheelMeasured = 0;
double rightWheelControl = 0;
double rightWheelRotational = 0;
double rightWheelLinear = 0;
PID leftWheelPID(&leftWheelMeasured, &leftWheelControl, &leftWheelDesired, kp, ki, kd, DIRECT);
PID rightWheelPID(&rightWheelMeasured, &rightWheelControl, &rightWheelDesired, kp, ki, kd, DIRECT);

//conversion factors and calculation values
const int wheelDiameter = 42;
const int ticksPerRev = 5960; //2000 for 100:1, 5960 for 298:1
const int velocityConvFactor = PI*wheelDiameter*1000;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
float rightEncoderDelta = 0;
float leftEncoderDelta = 0;
/* 320RPM MOTOR: 320RPM @ 6v, so say 300 RPM at 5v = 5 rev/sec, 2000 ticks/rev of output shaft, = 10000 ticks/sec
   255/10000 = 0.0255 conversion factor for seconds * 1000 25.5 conversion factor ms */
float soarPWMConvFactor = 28.5;

/* writes PWM commands to the motors takes a motor control pin, a motor
    direction pin, and a speed.  Negative speed means drive backwards. */
void setSpeed(int direction, int control, int speed) {
  if(speed < 0) {
    digitalWrite(direction, LOW);
  } else digitalWrite(direction, HIGH);
  analogWrite(control, abs(speed));
}

/* Determines wheel velocities from encoder counts and runs the PID loop */
void drive() {
  leftWheelDesired = leftWheelLinear + leftWheelRotational;
  rightWheelDesired = rightWheelLinear + rightWheelRotational;
  currentTime = millis();
  double dT = (currentTime - previousTime);
  
  //The encoder counts are mirrored because they're on oposite sides.
  //One could also switch pins in the Encoder declaration, but this seems more intuitive.
  encoderCountRight = -rightEncoder.read();
  encoderCountLeft = leftEncoder.read();
  rightEncoderDelta = encoderCountRight - lastEncoderCountRight;
  leftEncoderDelta = encoderCountLeft - lastEncoderCountLeft;
  lastEncoderCountRight = encoderCountRight;
  lastEncoderCountLeft = encoderCountLeft;
  rightWheelMeasured = (rightEncoderDelta/dT)*soarPWMConvFactor;
  leftWheelMeasured = (leftEncoderDelta/dT)*soarPWMConvFactor;

  //refresh *WheelOuputs see ./PID_v1.cpp
  leftWheelPID.Compute();
  rightWheelPID.Compute();
  setSpeed(rightMtrDirPin, rightMtrSpeedPin, rightWheelControl);
  setSpeed(leftMtrDirPin, leftMtrSpeedPin, leftWheelControl);
  previousTime = currentTime;
}

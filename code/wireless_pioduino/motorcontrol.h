/* motorcontrol.h -- handles the motors, encoders, and velocity outputs
   contains setSpeed() and drive() */
#include "Encoder.h"
#include "PID_v1.h"

Encoder R_Enc(2, 8); 
Encoder L_Enc(3, 9);
/* attaches quadrature encoders. On Romeo board pins 2 and 3 must
   be used for interrupt support.  Each encoder must have at least
   one interruptable pin. */
const int motorcontrolR = 5;     //MR Speed Control pin
const int motorcontrolL = 6;     //ML Speed Control pin
const int MR = 4;    //MR Direction Control pin
const int ML = 7;    //ML Direction Control pin
const int WHEEL_D = 42; //wheel diameter in mm
const int TICKSPERREV = 5960; //2000 for 100:1, 5960 for 298:1
const int MMPS = PI*WHEEL_D*1000;
unsigned long previousVTime = 0; //used for measuring velocity
unsigned long currentVTime = 0;
int VEL_L = 0; //velocities in mm/s
int VEL_R = 0;
int32_t cur_count_r = 0, cur_count_l = 0; //keeps track of encoder values
int32_t last_count_r = 0, last_count_l = 0;
float vel_r, vel_l; //velocities in encoder count differences
float ticks_to_PWM = 28.5; //conversion factor to take mm/s commands from soar and change them to PWM values
/* 320RPM MOTOR: 320RPM @ 6v, so say 300 RPM at 5v = 5 rev/sec, 2000 ticks/rev of output shaft, = 10000 ticks/sec
   255/10000 = 0.0255 conversion factor for seconds * 1000 25.5 conversion factor ms */
/* 100RPM MOTOR: 100RPM @ 6v, so say 90 RPM @ 5V, = 1.5 rev/sec, 20*298 = 5960 ticks/rev of output shaft = 8940 ticks/sec
   255/8940 = 0.0285 conversion factor for seconds *1000 28.5 conversion factor ms */
double SetpointLeft, InputLeft, OutputLeft, RotpointLeft, LinpointLeft; //PID setup
double SetpointRight, InputRight, OutputRight, RotpointRight, LinpointRight;
PID leftPID(&InputLeft, &OutputLeft, &SetpointLeft,0.3,2.2,.002, DIRECT); //kp=0.4, ki=2.2, kd=0.002
PID rightPID(&InputRight, &OutputRight, &SetpointRight,0.3,2.2,0.002, DIRECT);

/* writes PWM commands to the motors takes a motor pin, a motor control pin
   and a speed.  Negative speed means drive backwards */
void setSpeed(int motor, int control, int speed) {
  if(speed < 0) {
    digitalWrite(motor, LOW);
  } else digitalWrite(motor, HIGH);
  analogWrite(control, abs(speed));
}

/* determines velocities from encoder counts and runs the PID loop */
void drive() {
  //update all the variables for driving and PID
  SetpointLeft = LinpointLeft + RotpointLeft;
  SetpointRight = LinpointRight + RotpointRight;
  currentVTime = millis();
  double dT = (currentVTime - previousVTime);
  cur_count_r = -R_Enc.read();//the encoder counts are mirrored because they're on oposite sides
  cur_count_l = L_Enc.read();
  vel_r = cur_count_r - last_count_r;
  vel_l = cur_count_l - last_count_l;
  last_count_r = cur_count_r;
  last_count_l = cur_count_l;
  InputRight = (vel_r/dT)*ticks_to_PWM;
  InputLeft = (vel_l/dT)*ticks_to_PWM;
  leftPID.Compute();
  rightPID.Compute();
  setSpeed(MR, motorcontrolR, OutputRight); setSpeed(ML, motorcontrolL, OutputLeft);
  VEL_R = (vel_r/(dT*TICKSPERREV))*MMPS;
  VEL_L = (vel_l/(dT*TICKSPERREV))*MMPS;
  previousVTime = currentVTime;
}

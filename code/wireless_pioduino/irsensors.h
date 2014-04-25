/* irsensors.h -- interacts with all of the IR sensors
   contains getIRValue(), readIR() */
#define L_IRpin 1
#define C_IRpin 2
#define R_IRpin 3
const float stepsize = 5.0/1024; // 5V supply, value which is between 0 and 1023
const float delta = 10*(80-10)/2; // worked out from figure 2 http://www.pololu.com/file/0J85/gp2y0a21yk0f.pdf

/* Reads 5 voltages from the IR pins and averages them to
   find the distance from the object being observed.  Takes
   an int for the pin an IR sensor is on and returns the
   distance measurement as an int */
int getIRValue(int IRpin) {
  float sensor_volts = 0.0;
  float sensor_distance = 0.0;
  float samples = 5.0;
  for(int i=0; i<samples ;i++) {
    sensor_volts = analogRead(IRpin)* stepsize; // value from sensor * stepsize
    sensor_distance += delta*pow(sensor_volts, -1.10); // theoretical distance / (1/Volts)S
  }
  sensor_distance = sensor_distance/samples; //average
  return((int)(sensor_distance)); // print the distance in mm
}

/* basically just a buffer in case we need a 4th IR sensor
   (it will be on a non consecutive pin) */
int readIR(int sensor) {
  sensor++; //sensors are on pins 1,2,3 soar calls 0,1,2
  switch (sensor) {
  case L_IRpin:
    return(getIRValue(L_IRpin));
  case C_IRpin:
    return(getIRValue(C_IRpin));
  case R_IRpin:
    return(getIRValue(R_IRpin));
  default:
    return(0);
  }
}

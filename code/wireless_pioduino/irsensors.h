/* irsensors.h -- interacts with all of the IR sensors
   contains getIRValue(), readIR() */
#define leftIRPin 0
#define centerLeftIRPin 1
#define centerRightIRPin 2
#define rightIRPin 3
const float voltConvFactor = 5.0/1024;
// worked out from figure 2 in gp2y0a21yk0f.pdf
const float delta = 10*(80-10)/2;

/* Reads 5 voltages from the IR pins and averages them to
   find the distance from the object being observed.  Takes
   an int for the pin an IR sensor is on and returns the
   distance measurement as an int */
int getIRValue(int IRpin) {
  float IRVolts = 0.0;
  float distance = 0.0;
  float samples = 5.0;
  for(int i=0; i<samples ;i++) {
    IRVolts = analogRead(IRpin)* voltConvFactor;
    distance += delta*pow(IRVolts, -1.10); // theoretical distance / (1/Volts)S
  }
  distance = distance/samples; //average
  return((int)(distance)); // print the distance in mm
}

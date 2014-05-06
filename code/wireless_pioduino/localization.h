/* localization.h -- handles the position estimations of the robot from encoder data
   contains calculatePosition() */

//2000 for 100:1, 5960 for 298:1 0.85 is an "empirical fudge factor"
//TODO: account for fudge factor. Mostly likely caused by inaccuracies in soarPWMConvFactor.
const float encoderMMConvFactor = PI*wheelDiameter/(5960*0.85);
const int robotDiameter = 107; //in mm
int xpos;
int ypos;
float thpos;
float lastTh = 0;
//Note: different the lastEncoderCount*
int32_t prevEncoderCountRight = 0;
int32_t prevEncoderCountLeft = 0;

/* Determines xpos, ypos, and thpos from encoder readings
   thpos is calculated based on the assumption that the
   robot has rotated very little since the last calculation. */
void calculatePosition() {
  int dR = encoderCountRight - prevEncoderCountRight;
  int dL = encoderCountLeft - prevEncoderCountLeft;
  prevEncoderCountRight = encoderCountRight;
  prevEncoderCountLeft = encoderCountLeft;
  thpos = ((dL-dR)*encoderMMConvFactor/robotDiameter) + lastTh;
  if(thpos > 2*PI) thpos -= 2*PI;
  else if(thpos < 0) thpos += 2*PI;
  int distance = encoderMMConvFactor*(dR + dL)/2;
  xpos += distance*cos(thpos);
  ypos += distance*sin(thpos);
  lastTh = thpos;
  thpos = thpos*180/PI;
  return;
}

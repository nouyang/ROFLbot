/* localization.h -- handles the position estimations of the robot from encoder data
   contains calculatePosition() */
const float TICKSTOMM = PI*WHEEL_D/(5960*0.85); //2000 for 100:1, 5960 for 298:1 0.85 is an "empirical fudge factor"
const int ROBOT_DIAMETER = 89; //in mm
int xpos;
int ypos;
float thpos;
float lastth = 0;
int32_t prev_enc_r = 0;
int32_t prev_enc_l = 0;

/* Determines xpos, ypos, and thpos from encoder readings
   thpos is calculated based on the assumption that the
   robot has rotated very little since the last calculation. */
void calculatePosition() {
  int dR = cur_count_r - prev_enc_r;
  int dL = cur_count_l - prev_enc_l;
  prev_enc_r = cur_count_r;
  prev_enc_l = cur_count_l;
  thpos = ((dL-dR)*TICKSTOMM/ROBOT_DIAMETER) + lastth;
  if(thpos > 2*PI) thpos -= 2*PI;
  else if(thpos < 0) thpos += 2*PI;
  int distance = TICKSTOMM*(dR + dL)/2;
  xpos += distance*cos(thpos);
  ypos += distance*sin(thpos);
  lastth = thpos;
  thpos = thpos*180/PI;
  return;
}

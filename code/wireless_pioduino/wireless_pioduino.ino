/* wireless_pioduino.ino -- is the main component of the firmware it is responsible
   for interpretting and carrying out soar commands, sending SIP packets to soar,
   and doing various setup routines.
   contains setup(), loop(), sendSIP(), sendIOSIP(), sendEncSIP(), resetall() */
#include "constants.h"
#include "packethandling.h"
#include "motorcontrol.h"
#include "irsensors.h"
#include "localization.h"
#include <avr/wdt.h>

byte recpkt[200]; //packet revceived from soar 200 bytes is the biggest packet soar would ever send to the robot
size_t recdatalength;
unsigned long watchdog = 0; //passive value to check for connection
unsigned long sipTime = 0; //passive value to send SIPs
unsigned int sonarOrder[] = {0,1,2};
boolean connected = false;
boolean SIPflag = false;
boolean IOflag = false;
boolean sonarflag = true;
boolean Encflag = false;
boolean enabled = false;

void setup() {
  Serial1.begin(19200);
  //Serial.begin(19200);
  pinMode(13, OUTPUT);
  leftPID.SetMode(AUTOMATIC); rightPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(100); rightPID.SetSampleTime(100);
  leftPID.SetOutputLimits(-255,255); rightPID.SetOutputLimits(-255,255);
  InputLeft = 0; InputRight = 0;
  SetpointLeft = 0; SetpointRight = 0;
}

void loop() {
  if(Serial1.available() >= 3) { //greater than three means we have received a whole header :)
    recvPacket(recpkt);
    if(recpkt[0] != FAIL) {
      watchdog = millis();
      if(!connected) { //if not connected yet deal with sync commands
        byte command = recpkt[0];
        byte sndpkt[1];
        switch (command) {
        case SYNC0:
          sndpkt[0] = SYNC0;
          sendPacket(sndpkt, sizeof(sndpkt));
          break;
        case SYNC1:
          sndpkt[0] = SYNC1;
          sendPacket(sndpkt, sizeof(sndpkt));
          break;
        case SYNC2: { //send sync command as well as name of robot, class, and subclass null terminated
          byte SYNC2pkt[] = {SYNC2, 'N', 'Y', 'A', 'N', '!', 0, 'N', 'A', 'R', 'W', 'H', 'A', 'L', 0, 0x41, 0x2B, 0x2B, 0};
          sendPacket(SYNC2pkt, sizeof(SYNC2pkt));
          connected = true;
          digitalWrite(13, HIGH);
          SIPflag = true;
          break;
        }
        }
      } else { //connected? do the command dance! Many are not implemented because soar doesn't use them
        switch (recpkt[0]) {
        case PULSE:
          break;
        case OPEN:
          //starts the robot in a clean state turns on white LED
          InputLeft = 0; InputRight = 0;
          RotpointLeft = 0; RotpointRight = 0;
          LinpointLeft = 0; LinpointRight = 0;
          leftPID.setITerm(0); rightPID.setITerm(0);
          digitalWrite(13,HIGH);
          break;
        case CLOSE:
          resetall();
          break;
        case POLLING:
          //change the order of sonar firing or neglect some sonar alters sonarOrder
          for(int i = 0; i < sizeof(sonarOrder)/sizeof(int); i++) {
            sonarOrder[i] = recpkt[i+2];
          }
          break;
        case ENABLE:
          //enable or disable the motors (toggles enabled)
          if(recpkt[2] == 1) enabled = true;
          else {
            enabled = false;
            setSpeed(MR, motorcontrolR, 0);
            setSpeed(ML, motorcontrolL, 0);
          }
          break;
        case SETA:
          //set acceleration or deceleration point
          break;
        case SETV:
          //set velocity max speed
          //MAXVEL = recpkt[2] | (recpkt[3]<<8); //need to alter and map to mm/sec works for now
          break;
        case SETO:
          //reset internal coordinates to 0,0,0. Affects lasth, xpos, and ypos.
          lastth = 0;
          xpos = 0;
          ypos = 0;
          break;
        case MOVE:
          //translate a distance in mm
          break;
        case ROTATE:
          //rotate an amount in degrees/sec
          break;
        case SETRV:
          //reset max rotation velocity in degrees/sec
          break;
        case VEL:
          //move forward or reverse in mm/sec. Affects LinpointLeft and LinpointRight
          if(enabled) {
            LinpointLeft = recpkt[2] | (recpkt[3]<<8);
            if(recpkt[1] == 0x1B) { //check for negative integers
              LinpointLeft = -LinpointLeft;
            }
            LinpointRight = LinpointLeft;
            drive();
          }
          break;
        case HEAD:
          //turn to absolute heading counterclockwise in degrees
          break;
        case DHEAD:
          //turn relative to current heading counterclockwise in degrees
          break;
        case SAY:
          //control the pieazo?
          break;
        case JOYREQUEST:
          //request one (1), a continuous stream (>1), or stop (0) joystick SIPs
          break;
        case CONFIG:
          //send a configuration SIP
          break;
        case ENCODER: {
          //request encoder reading (1), or stream (>1), or stop sending readings(0). Toggles Encflag
          int argument = recpkt[2] | recpkt[3]<<8;
          if(argument == 0) {
            Encflag = false;
          } else if(argument == 1) {
            sendEncSIP();
          } else if(argument > 1) {
            sendEncSIP();
            Encflag = true;
          }
          break;
        }
        case RVEL: {
          //rotate in degrees per second.  Alters RotpointLeft and RotpointRight.
          if(enabled) {
            int rvel = recpkt[2] | (recpkt[3]<<8);
            if(recpkt[1] == 0x1B) { //check for negative integers
              RotpointLeft = -rvel;
              RotpointRight = rvel;
            } else {
              RotpointLeft = rvel;
              RotpointRight = -rvel;
            }
            drive();
          }
          break;
        }
        case DCHEAD:
          //heading setpoint relative to last setpoint
          break;
        case SETRA:
          //set rotational acceleration
          break;
        case SONAR:
          //send 0 to disable all sonar or 1 to enable them. Toggles sonarflag
          if(recpkt[2]&0x01 == 1) sonarflag = true;
          else sonarflag = false;
          //pioneer has four arrays I only care about the one hence the &0x01
          break;
        case STOP:
          //stop moving motors stay enabled.  Affects SetpointLeft and SetpointRight
          if(enabled) {
            SetpointLeft = 0;
            SetpointRight = 0;
            drive();
          }
          break;
        case DIGOUT:
          //Msbits is a byte mask that selects output port(s) for changes; lsbits set (1) or reset (0) the selected port.
          break;
        case VEL2:
          //set independant wheel velocities LSB=right; MSB=left
          break;
        case GRIPPER:
          break;
        case ADSEL:
          //set the A/D port number to be used in the SIP
          break;
        case GRIPPERVAL:
          //gripper server values
          break;
        case GRIPREQUEST:
          //Request one, a continuous stream (>1), or stop (0)Gripper SIPs.
          break;
        case IOREQUEST: {
          //request 1, a continuous stream of SIPs (>1), or stop (0). Toggles IOflag
          int argument = recpkt[2] | recpkt[3]<<8;
          if(argument == 0) {
            IOflag = false;
          } else if(argument == 1) {
            sendIOSIP();
          } else if(argument > 1) {
            sendIOSIP();
            IOflag = true;
          }
          break;
        }
        case TTY2:
          //Send string argument to serial device on AUX port on uC
          break;
        case GETAUX:
          //Request to retrieve 1-200 bytes from the aux serial channel; 0 flushes the aux serial input buffer.
          break;
        case BUMPSTALL:
          //Stop and register a stall if front (1), rear (2) or either (3) bump-ring contacted. Off (default) is 0.
          break;
        case TCM2:
          //TCM2 module commands??
          break;
        case JOYDRIVE:
          //allow (1) or deny (0) joystick control
          break;
        case SONARCYCLE:
          //Change the sonar cycle time; in milliseconds
          break;
        case HOSTBAUD:
          //Change the HOST serial port baud rate to 0=9600, 1=19200, 2=38400, 3=57600, or 4=115200.
          break;
        case AUX1BAUD:
          //Change the AUX1 serial port baud rate (see HOSTBAUD).
          break;
        case AUX2BAUD:
          //Change the AUX2 serial port baud rate (see HOSTBAUD).
          break;
        case AUX3BAUD:
          //Change the AUX3 serial port baud rate (see HOSTBAUD).
          break;
        case E_STOP:
          //emergency stop overrides deceleration
          setSpeed(MR, motorcontrolR, 0);
          setSpeed(ML, motorcontrolL, 0);
          break;
        case M_STALL:
          //Argument 1=MOTORS button off causes a stall
          break;
        case GYROREQUEST:
          //Request one, a continuous stream (>1), or stop (0) Gyro SIPs.
          break;
        case LCDWRITE:
          //there is no LCD screen on our robot lol
          break;
        case TTY4:
          break;
        case GETAUX3:
          break;
        case TTY3:
          break;
        case GETAUX2:
          break;
        case CHARGE:
          //0=release; 1=deploy autocharge-docking mechanism.
          break;
        case RESET:
          //Force a power on-like reset of the microcontroller.
          cli();                  // Clear interrupts
          wdt_enable(WDTO_15MS);  // Set the Watchdog to 15ms
          while(1);               // Enter an infinite loop
          break;
        case MAINTENANCE:
          //Engage microcontroller maintenance (ARSHstub) mode.
          break;
        default:
          break;
        }
      }
    }
  }
  if(millis() - sipTime > 100 && connected) { //soar expects SIPs every 100ms or so
    calculatePosition();
    if(SIPflag) {
      int sonarValues[3];
      if(!sonarflag) memset(sonarValues,0,sizeof(sonarValues)/sizeof(int));
      else {
        for(int i = 0; i < sizeof(sonarValues)/sizeof(int); i++) {
          sonarValues[i] = readIR(sonarOrder[i]);//populates the SIP with sonar readings
        }
      }
      sendSIP(sonarOrder,sonarValues,sizeof(sonarValues)/sizeof(int));
    }
    if(enabled) drive(); //update PID
    if(Encflag) sendEncSIP(); //send EncSIP
    if(IOflag) sendIOSIP(); //send IOSIP
    sipTime = millis(); //reset the time since last SIP update
  }
  if((millis() - watchdog) > 2000) { //if it hasn't seen a soar command for a while stop and reset
    resetall();
  }
}

/*constructs and sends the Server Information Packet
  argumens: number of sonar readings being sent.
  much smaller than the original SIP only location
  and sonar data is sent*/
void sendSIP(unsigned int sonarDiscs[], int sonarReadings[], size_t sonarCount) {
  byte SIPpkt[8+3*sonarCount];
  SIPpkt[0] = 0x32 | enabled;
  SIPpkt[1] = (int)xpos&0xFF;
  SIPpkt[2] = (int)xpos>>8;
  SIPpkt[3] = (int)ypos&0xFF;
  SIPpkt[4] = (int)ypos>>8;
  SIPpkt[5] = (int)thpos&0xFF;
  SIPpkt[6] = (int)thpos>>8;
  /* SIPpkt[7] = VEL_L&0xFF; //left wheel velocity  */
  /* SIPpkt[8] = VEL_L>>8; */
  /* SIPpkt[9] = VEL_R&0xFF;  //right wheel velocity */
  /* SIPpkt[10] = VEL_R>>8; */
  /* SIPpkt[11] = 0; //battery */
  /* SIPpkt[7] = stallbump&0xFF; */
  /* SIPpkt[8] = stallbump>>8; */
  /* SIPpkt[14] = 0; //control */
  /* SIPpkt[15] = 0; */
  /* SIPpkt[16] = 0; //flags */
  /* SIPpkt[17] = 0; */
  /* SIPpkt[18] = 0; //compass */
  SIPpkt[7] = sonarCount&0xFF;
  for(int i=0; i < sonarCount; i++) {
    SIPpkt[8+i*3] = sonarDiscs[i];
    SIPpkt[9+i*3] = sonarReadings[i]&0xFF;
    SIPpkt[10+i*3] = sonarReadings[i]>>8;
  }
  /* int index = 20+3*sonarCount; */
  /* SIPpkt[index] = 0; //grip state */
  /* SIPpkt[index+1] = 0; //anport */
  /* SIPpkt[index+2] = 0; //analog */
  /* SIPpkt[index+3] = 0; //digin */
  /* SIPpkt[index+4] = 0; //digout */
  /* SIPpkt[index+5] = 0; //batteryx10 */
  /* SIPpkt[index+6] = 0; */
  /* SIPpkt[index+7] = 0; //chargestate */
  sendPacket(SIPpkt, sizeof(SIPpkt));
}

/* Constructs and sends the IOSIP packet it only contains
   values for 3 analog pins, the rest are blank.  The blank
   ones are left in in case more analog readings are desired
   so that it will be a simple change. It is also smaller than
   the one from original pioneer OS. */
void sendIOSIP() {
  byte IOSIPpkt[17];
  int analog0 = analogRead(A0);
  int analog4 = analogRead(A4);
  int analog5 = analogRead(A5);
  IOSIPpkt[0] = 0xF0; //packet type always 0xF0
  /* IOSIPpkt[1] = 4; //number of digital input bytes */
  /* IOSIPpkt[2] = 0; //IDO-8 bits mapped */
  /* IOSIPpkt[3] = 0; //front bumper bits mapped */
  /* IOSIPpkt[4] = 0; //rear bumper bits mapped */
  /* IOSIPpkt[5] = 0; //IR inputs */
  /* IOSIPpkt[6] = 1; //number of digital output bytes */
  /* IOSIPpkt[7] = 0; //digital output bytes */
  /* IOSIPpkt[8] = 9; //number of A/D balues */
  IOSIPpkt[1] = 0; //Analog port values resolution 0-1023 two bytes!, 0-5 VDC
  IOSIPpkt[2] = 0;
  IOSIPpkt[3] = 0;
  IOSIPpkt[4] = 0;
  IOSIPpkt[5] = 0;
  IOSIPpkt[6] = 0;
  IOSIPpkt[7] = 0;
  IOSIPpkt[8] = 0;
  IOSIPpkt[9] = analog0&0xFF;
  IOSIPpkt[10] = analog0>>8;
  IOSIPpkt[11] = analog4&0xFF;
  IOSIPpkt[12] = analog4>>8;
  IOSIPpkt[13] = analog5&0xFF;
  IOSIPpkt[14] = analog5>>8;
  /* IOSIPpkt[9] = 0; */
  /* IOSIPpkt[10] = 0; */
  /* IOSIPpkt[11] = 0; */
  /* IOSIPpkt[12] = 0; */
  /* IOSIPpkt[13] = 0; */
  /* IOSIPpkt[14] = 0; */
  IOSIPpkt[15] = 0;
  IOSIPpkt[16] = 0;
  /* IOSIPpkt[25] = 0;   //battery analog input two bytes!   */
  /* IOSIPpkt[26] = 0;   */
  sendPacket(IOSIPpkt, sizeof(IOSIPpkt));
}

/* Constructs and sends the Encoder SIP */
void sendEncSIP() {
  byte EncSIP[5];
  cur_count_r = R_Enc.read();
  cur_count_l = -L_Enc.read();
  EncSIP[0] = 0x90;
  EncSIP[1] = cur_count_l&0xFF;
  EncSIP[2] = cur_count_l>>8;
  EncSIP[3] = cur_count_r&0xFF;
  EncSIP[4] = cur_count_r>>8;
}

/* Should return the robot to a state similar to the start
   all speeds, flags, encoders, and positions are set to
   zero */
void resetall() {
  setSpeed(MR, motorcontrolR, 0); setSpeed(ML, motorcontrolL, 0);
  InputLeft = 0; InputRight = 0;
  RotpointLeft = 0; RotpointRight = 0;
  LinpointLeft = 0; LinpointRight = 0;
  leftPID.setITerm(0); rightPID.setITerm(0);
  last_count_l = 0; last_count_r = 0;
  cur_count_l = 0; cur_count_r = 0;
  L_Enc.write(0); R_Enc.write(0);
  enabled = false;
  Encflag = false;
  IOflag = false;
  SIPflag = false;
  connected = false;
  lastth = 0;
  xpos = 0;
  ypos = 0;
  digitalWrite(13, LOW);
  return;
}

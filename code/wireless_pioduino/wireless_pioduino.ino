// wireless_pioduino.ino -- is the main component of the firmware it is responsible
//   for interpretting and carrying out soar commands, sending SIP packets to soar,
//   and doing various setup routines.
//   contains setup(), loop(), sendSIP(), sendIOSIP(), sendEncoderSIP(), resetAll() 

#include "constants.h"
#include "packethandling.h"
#include "motorcontrol.h"
#include "irsensors.h"
#include "localization.h"
#include <avr/wdt.h>

byte receivedPkt[200];
const int pktHeaderSize = 3;
unsigned int sonarOrder[] = {0,1,2,3};
const int ledPin = 13;

//flags and timers
unsigned long watchdog = 0;
unsigned long sipTime = 0;
boolean connected = false;
boolean SIPFlag = false;
boolean IOFlag = false;
boolean sonarFlag = true;
boolean encoderFlag = false;
boolean motorsEnabled = false;

//default setup function for Arduino runs once at boot
//see http://arduino.cc/en/Reference/Setup
void setup() {
  Serial1.begin(19200);
  //Serial.begin(19200);
  pinMode(ledPin, OUTPUT);
  
  //Initialize PID loops see ./PID_v1.cpp
  leftWheelPID.SetMode(AUTOMATIC);
  leftWheelPID.SetSampleTime(100); 
  leftWheelPID.SetOutputLimits(-255,255);
  leftWheelMeasured = 0;
  leftWheelDesired = 0;
  rightWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetSampleTime(100);
  rightWheelPID.SetOutputLimits(-255,255);
  rightWheelMeasured = 0;
  rightWheelDesired = 0;
}

//default loop function for Arduino runs continuously
//see http://arduino.cc/en/Reference/loop
void loop() {
  if(Serial1.available() >= pktHeaderSize) { 
    receivePkt(receivedPkt);
    if(receivedPkt[0] != FAIL) {
      watchdog = millis();

      //SoaR commands are all dealt with using a large switch-case statement.
      if(!connected) {
        byte command = receivedPkt[0];
        byte sndpkt[1];
        switch (command) {
        case SYNC0:
          sndpkt[0] = SYNC0;
          sendPkt(sndpkt, sizeof(sndpkt));
          break;
        case SYNC1:
          sndpkt[0] = SYNC1;
          sendPkt(sndpkt, sizeof(sndpkt));
          break;
        case SYNC2: { //send sync command as well as name of robot, class, and subclass null terminated
          byte SYNC2pkt[] = {SYNC2, 'N', 'Y', 'A', 'N', '!', 0, 'N', 'A', 'R', 'W', 'H', 'A', 'L', 0, 0x41, 0x2B, 0x2B, 0};
          sendPkt(SYNC2pkt, sizeof(SYNC2pkt));
          connected = true;
          digitalWrite(ledPin, HIGH);
          SIPFlag = true;
          break;
        }
        }
      } else { 
        switch (receivedPkt[0]) {
        case PULSE:
          break;
        case OPEN:
          //starts the robot in a clean state turns on white LED
          leftWheelMeasured = 0;
          leftWheelRotational = 0;
          leftWheelLinear = 0;
          leftWheelPID.setITerm(0);
          rightWheelMeasured = 0;
          rightWheelRotational = 0;
          rightWheelLinear = 0;
          rightWheelPID.setITerm(0);
          digitalWrite(ledPin,HIGH);
          break;
        case CLOSE:
          resetAll();
          break;
        case POLLING:
          //change the order of sonar firing or neglect some sonar alters sonarOrder
          for(int i = 0; i < sizeof(sonarOrder)/sizeof(int); i++) {
            sonarOrder[i] = receivedPkt[i+2];
          }
          break;
        case ENABLE:
          //enable or disable the motors (toggles motorsEnabled)
          if(receivedPkt[2] == 1) motorsEnabled = true;
          else {
            motorsEnabled = false;
            setSpeed(rightMtrDirPin, rightMtrSpeedPin, 0);
            setSpeed(leftMtrDirPin, leftMtrSpeedPin, 0);
          }
          break;
        case SETA:
          //set acceleration or deceleration point
          break;
        case SETV:
          //set velocity max speed
          //MAXVEL = receivedPkt[2] | (receivedPkt[3]<<8); //need to alter and map to mm/sec works for now
          break;
        case SETO:
          //reset internal coordinates to 0,0,0. Affects lasth, xpos, and ypos.
          lastTh = 0;
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
          //move forward or reverse in mm/sec. Affects leftWheelLinear and rightWheelLinear
          if(motorsEnabled) {
            leftWheelLinear = receivedPkt[2] | (receivedPkt[3]<<8);
            if(receivedPkt[1] == 0x1B) { //check for negative integers
              leftWheelLinear = -leftWheelLinear;
            }
            rightWheelLinear = leftWheelLinear;
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
          //control a pieazo?
          break;
        case JOYREQUEST:
          //request one (1), a continuous stream (>1), or stop (0) joystick SIPs
          break;
        case CONFIG:
          //send a configuration SIP
          break;
        case ENCODER: {
          //request encoder reading (1), or stream (>1), or stop sending readings(0). Toggles encoderFlag
          int argument = receivedPkt[2] | receivedPkt[3]<<8;
          if(argument == 0) {
            encoderFlag = false;
          } else if(argument == 1) {
            sendEncoderSIP();
          } else if(argument > 1) {
            sendEncoderSIP();
            encoderFlag = true;
          }
          break;
        }
        case RVEL: {
          //rotate in degrees per second.  Alters leftWheelRotational and rightWheelRotational.
          if(motorsEnabled) {
            int rvel = receivedPkt[2] | (receivedPkt[3]<<8);
            if(receivedPkt[1] == 0x1B) { //check for negative integers
              leftWheelRotational = -rvel;
              rightWheelRotational = rvel;
            } else {
              leftWheelRotational = rvel;
              rightWheelRotational = -rvel;
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
          //send 0 to disable all sonar or 1 to enable them. Toggles sonarFlag
          if(receivedPkt[2]&0x01 == 1) sonarFlag = true;
          else sonarFlag = false;
          //pioneer has four arrays I only care about the one hence the &0x01
          break;
        case STOP:
          //stop moving motors stay enabled.  Affects leftWheelDesired and rightWheelDesired
          if(motorsEnabled) {
            leftWheelDesired = 0;
            rightWheelDesired = 0;
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
          //request 1, a continuous stream of SIPs (>1), or stop (0). Toggles IOFlag
          int argument = receivedPkt[2] | receivedPkt[3]<<8;
          if(argument == 0) {
            IOFlag = false;
          } else if(argument == 1) {
            sendIOSIP();
          } else if(argument > 1) {
            sendIOSIP();
            IOFlag = true;
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
          setSpeed(rightMtrDirPin, rightMtrSpeedPin, 0);
          setSpeed(leftMtrDirPin, leftMtrSpeedPin, 0);
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
    if(SIPFlag) {
      int sonarValues[sizeof(sonarOrder)/sizeof(int)];
      if(!sonarFlag) memset(sonarValues,0,sizeof(sonarValues)/sizeof(int));
      else {
        for(int i = 0; i < sizeof(sonarValues)/sizeof(int); i++) {sonarValues[i] = getIRValue(sonarOrder[i]);
        }
      }
      sendSIP(sonarOrder,sonarValues,sizeof(sonarValues)/sizeof(int));
    }
    if(motorsEnabled) drive(); //update PID
    if(encoderFlag) sendEncoderSIP();
    if(IOFlag) sendIOSIP();
    sipTime = millis();
  }
  //If there hasn't been a SoaR command for > 2 seconds we're probably disconnected
  if((millis() - watchdog) > 2000) {
    resetAll();
  }
}

/* Constructs and sends the Server Information Packet
   argumens: number of sonar readings being sent.
   much smaller than the original SIP only location
   and sonar data is sent see pg. 30 code/pioneer-manualv3.pdf*/
void sendSIP(unsigned int sonarDiscs[], int sonarReadings[], size_t sonarCount) {
  byte SIPpkt[8+3*sonarCount];
  SIPpkt[0] = 0x32 | motorsEnabled;
  SIPpkt[1] = (int)xpos&0xFF;
  SIPpkt[2] = (int)xpos>>8;
  SIPpkt[3] = (int)ypos&0xFF;
  SIPpkt[4] = (int)ypos>>8;
  SIPpkt[5] = (int)thpos&0xFF;
  SIPpkt[6] = (int)thpos>>8;
  SIPpkt[7] = sonarCount&0xFF;
  for(int i=0; i < sonarCount; i++) {
    SIPpkt[8+i*3] = sonarDiscs[i];
    SIPpkt[9+i*3] = sonarReadings[i]&0xFF;
    SIPpkt[10+i*3] = sonarReadings[i]>>8;
  }
  sendPkt(SIPpkt, sizeof(SIPpkt));
}

/* Constructs and sends the IOSIP packet it only contains
   values for 3 analog pins, the rest are blank.  The blank
   ones are left in in case more analog readings are desired
   so that it will be a simple change. It is also smaller than
   the one from original pioneer OS. */
void sendIOSIP() {
  byte IOSIPpkt[17];
  int leftPhotodiode = analogRead(A4); 
  int rightPhotodiode = analogRead(A5); 
  int servoPotVoltage = analogRead(A11);
  IOSIPpkt[0] = 0xF0; //packet type always 0xF0
  IOSIPpkt[1] = 0; //Analog port values resolution 0-1023 two bytes!, 0-5 VDC
  IOSIPpkt[2] = 0;
  IOSIPpkt[3] = 0;
  IOSIPpkt[4] = 0;
  IOSIPpkt[5] = 0;
  IOSIPpkt[6] = 0;
  IOSIPpkt[7] = 0;
  IOSIPpkt[8] = 0;
  IOSIPpkt[9] = servoPotVoltage&0xFF;
  IOSIPpkt[10] = servoPotVoltage>>8; 
  IOSIPpkt[11] = leftPhtotodiode&0xFF;
  IOSIPpkt[12] = leftPhotodiode>>8;
  IOSIPpkt[13] = rightPhotodiode&0xFF;
  IOSIPpkt[14] = rightPhotodiode>>8;
  IOSIPpkt[15] = 0;
  IOSIPpkt[16] = 0;
  sendPkt(IOSIPpkt, sizeof(IOSIPpkt));
}

/* Constructs and sends the Encoder SIP
   see pg. 38 code/pioneer-robotv3.pdf */
void sendEncoderSIP() {
  byte encoderSIP[5];
  encoderCountRight = -rightEncoder.read();
  encoderCountLeft = leftEncoder.read();
  encoderSIP[0] = 0x90;
  encoderSIP[1] = encoderCountLeft&0xFF;
  encoderSIP[2] = encoderCountLeft>>8;
  encoderSIP[3] = encoderCountRight&0xFF;
  encoderSIP[4] = encoderCountRight>>8;
}

/* Returns the robot to a state similar to a just booted state
   all speeds, flags, encoders, and positions are set to
   zero */
void resetAll() {
  //left motor
  setSpeed(leftMtrDirPin, leftMtrSpeedPin, 0);
  leftWheelMeasured = 0;
  leftWheelRotational = 0;
  leftWheelLinear = 0;
  leftWheelPID.setITerm(0);
  lastEncoderCountLeft = 0;
  encoderCountLeft = 0;
  leftEncoder.write(0);

  //right motor
  setSpeed(rightMtrDirPin, rightMtrSpeedPin, 0);
  rightWheelMeasured = 0;
  rightWheelRotational = 0;
  rightWheelLinear = 0;
  rightWheelPID.setITerm(0);
  lastEncoderCountRight = 0;
  encoderCountRight = 0;
  rightEncoder.write(0);

  //flags
  motorsEnabled = false;
  encoderFlag = false;
  IOFlag = false;
  SIPFlag = false;
  connected = false;

  //position
  lastTh = 0;
  xpos = 0;
  ypos = 0;

  digitalWrite(ledPin, LOW);
  return;
}

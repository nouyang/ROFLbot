/* constants.h -- contains all of the soar commands and their values */
#include "Arduino.h"

const byte SYNC0 = 0;
const byte SYNC1 = 1;
const byte SYNC2 = 2;

//Argument types
const byte  ARGINT		= 0x3B;
const byte  ARGNINT		= 0x1B;
const byte  ARGSTR		= 0x2B;

// P2OS Commands
const byte PULSE				=   0;
const byte OPEN					=   1;
const byte CLOSE				=   2;
const byte POLLING				=   3;
const byte ENABLE				=   4;
const byte SETA					=   5;
const byte SETV					=   6;
const byte SETO					=   7;
const byte MOVE					=   8;
const byte ROTATE				=   9;
const byte SETRV				=  10;
const byte VEL					=  11;
const byte HEAD					=  12;
const byte DHEAD				=  13;
const byte SAY					=  15;
const byte JOYREQUEST				=  17;
const byte CONFIG				=  18;
const byte ENCODER				=  19;
const byte RVEL					=  21;
const byte DCHEAD				=  22;
const byte SETRA				=  23;
const byte SONAR				=  28;
const byte STOP					=  29;
const byte DIGOUT				=  30;
const byte VEL2					=  32;
const byte GRIPPER				=  33;
const byte ADSEL				=  35;
const byte GRIPPERVAL				=  36;
const byte GRIPREQUEST				=  37;
const byte IOREQUEST				=  40;
const byte TTY2					=  42;
const byte GETAUX				=  43;
const byte BUMPSTALL				=  44;
const byte TCM2					=  45;
const byte JOYDRIVE				=  47;
const byte SONARCYCLE				=  48;
const byte HOSTBAUD				=  50;
const byte AUX1BAUD				=  51;
const byte AUX2BAUD				=  52;
const byte AUX3BAUD				=  53;
const byte E_STOP				=  55;
const byte M_STALL				=  56;
const byte GYROREQUEST				=  58;
const byte LCDWRITE				=  59;
const byte TTY4					=  60;
const byte GETAUX3				=  61;
const byte TTY3					=  66;
const byte GETAUX2				=  67;
const byte CHARGE				=  68;
const byte RESET				= 254;
const byte MAINTENANCE				= 255;
const byte FAIL                                 = 253;

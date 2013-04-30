#include <Servo.h> 
#include <EEPROM.h>
#include <Serial.h>

#define STEPPERstepPIN 14
#define STEPPERdirPIN  15
#define SERIALbaud  9600
#define SERVOGRIPPERPIN 1
#define SERVOUNDERPIN 2
#define SERVOOVERPIN 3

/* Address in EEPROM to store the distance between the gripper pads. */
const int gripStartAddress = 100;
/* Address in EEPROM to store current height. */
const int heightStartAddress = 102;
/* Array with the 3 different servo angles for standby position */
char standbyAngles[3] = {0,90,0};

/* Create a servo object for the three different servos on the arm. */
Servo gripperRot, underArmRot, overArmRot;
Servo servos[3] = {gripperRot, underArmRot, overArmRot};

/* 
  Variable that holds the current distance between the gripper pads in mm. 
  Every step the stepper takes is 7.5 degrees, the gears have a ratio of 12/30 and the threaded rod got a M6 thread. 
  Distance moved = 2 * (steps*(15/720)/(12/30)) * 0,001 = steps/60000 [m]
*/
int gripperDist;
/*
  Variable that holds the current height of the arm. The motor that controlls the height is an ordinary DC-motor but 
  with an encoder with 4 fields attached to the threaded rod. The rod has a pitch of 4 mm/rev. 
  (or maybe we attach the encoder to another part of the elevation gears...)  
*/
int currentHeight;

// Variables for the servo angles.
char gripperAng, underArmAng, overArmAng;
 
void setup() { 
    
  /* Start a serialconnection with specified baudrate and 8 data bits, no parity and 1 stop bit. */
  Serial1.begin(SERIALbaud, SERIAL_8N1);
  
  /* Say hello. */
  Serial1.println("Serial up and running!");
  
  /* Attach the servos to the pins on the Mega */
  gripperRot.attach(SERVOGRIPPERPIN);
  underArmRot.attach(SERVOUNDERPIN);
  overArmRot.attach(SERVOOVERPIN);
  
  /* Fetch paddistance from EEPROM */
  gripperDist = EEPROM_readInt(gripStartAddress);
  
  /* Check if an error has occured and the gripper spacing is unknown, if so calibrate! */
  if (gripperDist == 0xFFFF){             
    //calibrateGripper();
  }
  
  /* Fetch current arm height from EEPROM and check if it is valid */
  currentHeight = EEPROM_readInt(heightStartAddress);
  if (currentHeight == 0xFFFF) {
    //calibrateHeight();
  }  
} 
 
void loop() 
{ 
  if (Serial1.available() >= 4) {
    char buffer[4];
    Serial1.readBytesUntil('\n',buffer,4);
    switch(buffer[0]){
      case 'f':
        // Execute the function with the opcode in buffer[1].
        operate(buffer[1]);
        break;
      case 's':
        // Shut down.
        break;
    }
  }
  delay(15);                           // waits for the servo to get there 
} 
/*
 * Function that takes an operation code and there after performes that operation.
 */
void operate(char opcode) {
  switch(opcode){
    case 0:
      /* Load the carparts from the table. */
      break;
    case 1:
      /* Unload the carparts from the AGV */
      break;
    case 2:
      /* Enter transport mode (the arm positioned to take as lite place as possible) */
      moveServos(servos, standbyAngles);
      /* Set gripping width to 155 mm */
      gripperGrip(155);
      break;
    case 3:
      /* Calibrate gripper */
      break;
    case 4:
      /* Calibrate the heightsensor/encoder */
      break;
  }
}
/* 
 * Function for moving all three servos "at the same time" 
*/
void moveServos(Servo servo[3], char angle[3]) {
  char i;
  for(i = 0; i < 3; i++){
    servo[i].write(angle[i]);
  }
}

void modifyAltitude(int desiredHeight) {
  
  /* Write 0xFFFF to currentHeights EEPROM address so that incase of a power failure when moving up or down we will know that we are in an unknown state. */
  EEPROM_writeInt(heightStartAddress, 0xFFFF);
  
  /* Loop while we need to move */
  while(desiredHeight != currentHeight) {
    
    /* Going up */
    if(desiredHeight > currentHeight){
      // set motor to go UP
      // read encoder
      // increase currentHeight   
    } 
    /* Going down */
    else {
      // set motor to go DOWN
      // read encoder
      // decrease currentHeight
    }
  }
  
  /* Write the currentHeight to EEPROM for reusage after powerloop */
  EEPROM_writeInt(heightStartAddress, currentHeight);
}

/* 
 * Function that open/closes the gripper to the desired spacing given as the input 
 * argument desiredGripDist [mm] 
 */
void gripperGrip(int desiredGripDist) {
  
  /* Write 0xFFFF to EEPROM so that we can check if we actually know how far appart the pads are at setup. */
  EEPROM_writeInt(gripStartAddress, 0xFFFF);       
  
  /* Calculate how many steps the stepper has to take to get to the new position */
  int stepsToTake = (gripperDist-desiredGripDist) * 60;
  
  /* If negative number of steps the spacing should incerased and vice versa */
  if(stepsToTake < 0 ) { 
    // Set rotation to CCW    
    digitalWrite(STEPPERdirPIN, LOW);      
    int i = stepsToTake * (-1);
    
    // Take the amount of steps calculated
    for(i;i > 0; i--) {            
      digitalWrite(STEPPERstepPIN, HIGH);
      
      // Delay 2 ms (total 4 ms) to get a step duty cycle of aprox 4 ms -> 312 RPM  
      delay(2);                            
      digitalWrite(STEPPERstepPIN, LOW);
      delay(2);
    }
  } else if(stepsToTake > 0) {
    
    // Set rotation to CW
    digitalWrite(STEPPERdirPIN, HIGH);     
    int i = stepsToTake;
    
    // Take the amount of steps calculated
    for(i;i > 0; i--) {                   
      digitalWrite(STEPPERstepPIN, HIGH);
      
      // Delay 2 ms (total 4 ms) to get a step duty cycle of aprox 4 ms -> 312 RPM  
      delay(2);                            
      digitalWrite(STEPPERstepPIN, LOW);
      delay(2);
    }
  } else {
    // No need to move, already at position.
  }
  // Write the current distance between the pads to non-volatile memory. (EEPROM)
  EEPROM_writeInt(gripStartAddress, desiredGripDist);
}
 
/*
  Function for writing a int to EEPROM.
*/
void EEPROM_writeInt(int address, int value){
   byte high = (byte)((value & 0xFF00) >> 8);
   byte low = (byte)(value & 0x00FF);
   EEPROM.write(address++, high);
   EEPROM.write(address, low);
}
/*
  Function for reading a int from EEPROM.
*/
int EEPROM_readInt(int address){
   int high = ((int) EEPROM.read(address++)) << 8;
   int low = (int) EEPROM.read(address);
   return (high | low);
}


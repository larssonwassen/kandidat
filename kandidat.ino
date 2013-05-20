#include <Servo.h> 
#include <EEPROM.h>
#include <Serial.h>

#define DEBUG 1
#define STEPPERstepPIN 42
#define STEPPERdirPIN  43
#define SERIALbaud  9600
#define SERVOGRIPPERPIN 10
#define SERVOUNDERPIN 11
#define SERVOOVERPIN 12
#define ENCODERPIN 40

#define debug_print(message) \
            do { if (DEBUG) Serial.println(message); } while (0)

/* Address in EEPROM to store the distance between the gripper pads. */
const int gripStartAddress = 100;

/* Address in EEPROM to store current height. */
const int heightStartAddress = 102;

/* Array with the 3 different servo angles for standby position */
unsigned char standbyAngles[3] = {0,90,13};

/* Angles for different loading routines */
unsigned char angles0[3] = {55, 50, 13};
unsigned char angles1[3] = {90, 107, 150};
unsigned char currentAngles[3];

/* Create a servo object for the three different servos on the arm. */
Servo gripperRot, underArmRot, overArmRot;
Servo servos[3] = {gripperRot, underArmRot, overArmRot};

/* 
 * Variable that holds the current distance between the gripper pads in mm. 
 * Every step the stepper takes is 7.5 degrees, the gears have a ratio of 12/30 and the threaded rod got a M6 thread. 
 * Distance moved = 2 * (steps*(15/720)/(12/30)) * 0,001 = steps/60000 [m]
 */
byte gripperDist = 206;
/*
 * Variable that holds the current height of the arm. The motor that controlls the height is an ordinary DC-motor but 
 * with an encoder with 8 fields attached to the threaded rod. The rod has a pitch of 4 mm/rev. 
 * (or maybe we attach the encoder to another part of the elevation gears...)  
 */
byte currentHeight = 119;
char blackOrWhite = 0;
char oldBlackOrWhite = 0;

/* Variables for the servo angles. */
char gripperAng, underArmAng, overArmAng;
 
void setup() {
  EEPROM.write(heightStartAddress, 69);
  /* Setup the pins for the stepper motor as output and the pin for the encoder as input. */
  pinMode(STEPPERstepPIN, OUTPUT);
  pinMode(STEPPERdirPIN, OUTPUT);
  pinMode(ENCODERPIN, INPUT);  
  
  /* Start serialconnection with PC over usb and with the sabertooth 2x25 on serial1. 
   * Use specified baudrate and 8 data bits, no parity and 1 stop bit. 
   */
  Serial.begin(SERIALbaud, SERIAL_8N1);
  Serial1.begin(SERIALbaud, SERIAL_8N1);
  
  debug_print("Serial up nad running!");
  
  /* Attach the servos to the pins on the Mega */
  gripperRot.attach(10);
  underArmRot.attach(11);
  overArmRot.attach(12);
  servos[0].write(standbyAngles[0]);
  servos[1].write(standbyAngles[1]);
  servos[2].write(standbyAngles[2]);
  currentAngles[0] = standbyAngles[0];  
  currentAngles[1] = standbyAngles[1];  
  currentAngles[2] = standbyAngles[2];
  
  /* Fetch the old distance between the gripper pads from EEPROM */
  gripperDist = EEPROM.read(gripStartAddress);
  
  /* Check if an error occured during last run and the gripper spacing is unknown, if so calibrate! */
  if (gripperDist == 0xFF){  
    debug_print("Calibrate gripper");
    gripperDist = 206;
  }
  
  /* Fetch current arm height from EEPROM and check if it is valid or if an error occured during last run. */
  currentHeight = EEPROM.read(heightStartAddress);
  if (currentHeight == 0xFF) {
    debug_print("Calibrate height");
    currentHeight = 119;
  }  
} 
 
void loop() {
    
   /* Wait for a command to arrive by serial com. */ 
  if (Serial.available() >= 5) { 
    
    /* Read the message recived and se if it was anyting of value */
    char messageBuffer[5];
    Serial.readBytesUntil('\n',messageBuffer,5);
    switch(messageBuffer[0]){
      case 'f':
      debug_print("Function code f recived.");
        /* Execute an complete series of commands specified by the opcode in messageBuffer[1]. */
        operate(messageBuffer[1]);
        break;
      case 's':
        /* Move a servo, specified by messageBuffer[1] (gripperRot -> 0, underArmRot -> 1, overArmRot -> 2) to the angle given in messageBuffer[2] */
        debug_print("Function code s recived.");
        debug_print(messageBuffer[2]);
        if(messageBuffer[1] == '1') { 
          debug_print("Writing to servo for griprot.");
          servos[0].write(messageBuffer[2]);
        } else if(messageBuffer[1] == '2') { 
          debug_print("Writing to servo for underarm.");
          servos[1].write(messageBuffer[2]);
        } else if(messageBuffer[1] == '3') { 
          debug_print("Writing to servo for overarm.");
          servos[2].write(messageBuffer[2]);
        }
        break;       
      case 'h':
        /* Run the elevation engine with the speed specified in the messageBuffer[1] byte. (1(Ctrl-B) -> full reverse, 64(@) -> stop, 127(Tilde) -> full forward) */
        debug_print("Function code h recived. Writing ");
        debug_print(messageBuffer[1]);
        debug_print(" to elevation motor.");
        Serial1.write(messageBuffer[1]);
        break;
    }
  } 
} 

/*
 * Function that takes an operation code and there after performes that operation.
 */
void operate(char opcode) {
  switch(opcode){
    case '0':
      debug_print("Opcode 0.");
      /* Load the carparts from the table. */
      //gripperGrip(200);
      //modifyAltitude(300);      
      moveServos(servos, angles0);
      break;
    case '1':
      Serial.println("Opcode 1.");
      /* Unload the carparts from the AGV */
      //gripperGrip(200);
      //modifyAltitude(300);      
      moveServos(servos, angles1);      
      break;
    case '2':
      debug_print("Opcode 2.");
      /* Enter transport mode (the arm positioned to take as lite place as possible) */
      moveServos(servos, standbyAngles);
      break;
    case '3':
      debug_print("Opcode 3.");
      /* Calibrate gripper */      
      gripperGrip(193);
      //modifyAltitude(150);
      break;
    case '4':
      debug_print("Opcode 4.");
      /* Calibrate the heightsensor/encoder */
      gripperGrip(205);
     //modifyAltitude(200);
      break;
  }
}
/* 
 * Function for moving all three servos "at the same time" 
*/
void moveServos(Servo servo[3], unsigned char angle[3]) {
  char i;
  unsigned char newAngle;
  int deltaAngle;
  for(i = 0; i < 3; i++){
    deltaAngle = currentAngles[i] - angle[i];
    while (deltaAngle > 0){
      newAngle = currentAngles[i]-1;
      debug_print(newAngle);
      servo[i].write(newAngle);
      delay(30);
      currentAngles[i] = newAngle;
      deltaAngle = currentAngles[i] - angle[i];
    }
    while (deltaAngle < 0){
      newAngle = currentAngles[i]+1;
      debug_print(newAngle);
      servo[i].write(newAngle);
      delay(30);
      currentAngles[i] = newAngle;
      deltaAngle = currentAngles[i] - angle[i];
    }
  }
}
/*
 * Function for changing the height of the arm. A PID-regulator or atleast PI.regulator should be implemented here.
 * 43,175 encodersteps/mm 
*/ 
void modifyAltitude(int desiredHeight) {
  debug_print(currentHeight);
  
  
  /* Write 0xFF to currentHeights EEPROM address so that incase of a power failure when moving up or down we will know that we are in an unknown state. */
  EEPROM.write(heightStartAddress, 0xFF);
  
  int heightDelta = 0;
  int stepsToTake = 0;
  int stepsTaken = 0;
  char blackOrWhite = 0;
  char oldBlackOrWhite = 0; 
  
  heightDelta = currentHeight-desiredHeight;
  debug_print(heightDelta);
  
  /* Going up */
  if(heightDelta < 0){
    debug_print("Up");
    
    /* The arm changes 1 mm in height every 43 encoderStep, the height delta is negative so change its sign with -43. */
    stepsToTake = heightDelta * (-137);
    
    /* Full speed (DEC 1) Up */
    Serial1.write(0x01);
    debug_print(stepsToTake);
    
    /* Until we have reached the desired position, read the encoder and look for a change. */ 
    while(stepsTaken!=stepsToTake){     
      blackOrWhite = digitalRead(ENCODERPIN);
      if(oldBlackOrWhite != blackOrWhite){
        stepsTaken++;
        oldBlackOrWhite = blackOrWhite; 
      }
    }  
  } /* Going down */
  else if (heightDelta > 0) {
    debug_print("Down");
    
    /* The arm changes 1 mm in height every 43 encoderStep. */
    stepsToTake = heightDelta * 137;
    
    /* Full speed (DEC 127) down */
    Serial1.write(0x7F); 
    debug_print(stepsToTake);
    
    /* Until we have reached the desired position, read the encoder and look for a change. */
    while(stepsTaken != stepsToTake){     
      blackOrWhite = digitalRead(ENCODERPIN);
      if(oldBlackOrWhite != blackOrWhite){
        stepsTaken++;
        oldBlackOrWhite = blackOrWhite; 
      }
    }
  }
  /* Write 64 (DEC) to the Sabertooth to stop the elevation motor */
  Serial1.write(0x40); 
  
  /* Update the currentHeight variable to represent the new height */
  currentHeight = desiredHeight;
  
  /* Write the currentHeight to EEPROM for reusage after powerloop */
  EEPROM.write(heightStartAddress, currentHeight);
}

/* 
 * Function that open/closes the gripper to the desired spacing given as the input 
 * argument desiredGripDist [mm] 
 */
void gripperGrip(int desiredGripDist) {
  debug_print("Enter gripperGrip with desired dist ");
  debug_print(desiredGripDist);
  
  /* Write 0xFF to EEPROM so that we can check if we actually know how far appart the pads are at setup. */
  EEPROM.write(gripStartAddress, 0xFF);       
  debug_print("Written to EEPROM");
   
  /* Calculate how many steps the stepper has to take to get to the new position */
  int stepsToTake = (gripperDist-desiredGripDist) * 60;
  int i = 0;
  debug_print(stepsToTake);
  
  /* If negative number of steps the spacing should incerased and vice versa */
  if(stepsToTake < 0 ) {
    
    /* Set rotation to CCW */    
    digitalWrite(STEPPERdirPIN, LOW);      
    i = stepsToTake * (-1);
    
  } else if(stepsToTake > 0) {
    
    /* Set rotation to CW */
    digitalWrite(STEPPERdirPIN, HIGH);     
    i = stepsToTake;
    
  }  
    /* Take the amount of steps calculated */
    for(i;i > 0; i--) {
      /* Delay 2 ms (total 4 ms) to get a step duty cycle of aprox 4 ms -> 312 RPM */      
      digitalWrite(STEPPERstepPIN, HIGH);   
      delay(3);                            
      digitalWrite(STEPPERstepPIN, LOW);
      delay(3);
      
    }
    /* update gripperDist to represent the new spacing */
    gripperDist = desiredGripDist;
    debug_print("End of gripper");
  /* Write the current distance between the pads to non-volatile memory. (EEPROM) */
  EEPROM.write(gripStartAddress, desiredGripDist);
}
//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value) {
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address) {
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

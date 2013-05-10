#include <Servo.h> 
#include <EEPROM.h>
#include <Serial.h>

#define STEPPERstepPIN 42
#define STEPPERdirPIN  43
#define SERIALbaud  9600
#define SERVOGRIPPERPIN 11
#define SERVOUNDERPIN 12
#define SERVOOVERPIN 13
#define ENCODERPIN 40

/* Address in EEPROM to store the distance between the gripper pads. */
const int gripStartAddress = 100;

/* Address in EEPROM to store current height. */
const int heightStartAddress = 102;

/* Array with the 3 different servo angles for standby position */
char standbyAngles[3] = {30,120,0};

/* Create a servo object for the three different servos on the arm. */
Servo gripperRot, underArmRot, overArmRot;
Servo servos[3] = {gripperRot, underArmRot, overArmRot};

/* 
  Variable that holds the current distance between the gripper pads in mm. 
  Every step the stepper takes is 7.5 degrees, the gears have a ratio of 12/30 and the threaded rod got a M6 thread. 
  Distance moved = 2 * (steps*(15/720)/(12/30)) * 0,001 = steps/60000 [m]
*/
byte gripperDist;
/*
  Variable that holds the current height of the arm. The motor that controlls the height is an ordinary DC-motor but 
  with an encoder with 8 fields attached to the threaded rod. The rod has a pitch of 4 mm/rev. 
  (or maybe we attach the encoder to another part of the elevation gears...)  
*/
byte currentHeight;

/* Variables for the servo angles. */
char gripperAng, underArmAng, overArmAng;
 
void setup() {
  
  /* Setup the pins for the stepper motor as output and the pin for the encoder as input. */
  pinMode(STEPPERstepPIN, OUTPUT);
  pinMode(STEPPERdirPIN, OUTPUT);
  pinMode(ENCODERPIN, INPUT);  
  
  /* Start serialconnection with PC over usb and with the sabertooth 2x25 on serial1. 
   * Use specified baudrate and 8 data bits, no parity and 1 stop bit. 
  */
  Serial.begin(SERIALbaud, SERIAL_8N1);
  Serial1.begin(SERIALbaud, SERIAL_8N1);
  
  /* Say hello on the serialport to the PC. */
  Serial.println("Serial up and running!");
  
  /* Attach the servos to the pins on the Mega */
  gripperRot.attach(SERVOGRIPPERPIN);
  underArmRot.attach(SERVOUNDERPIN);
  overArmRot.attach(SERVOOVERPIN);
  
  /* Fetch the old distance between the gripper pads from EEPROM */
  gripperDist = EEPROM.read(gripStartAddress);
  
  /* Check if an error occured during last run and the gripper spacing is unknown, if so calibrate! */
  if (gripperDist == 0xFF){             
    //calibrateGripper();
    gripperDist = 175;
  }
  
  /* Fetch current arm height from EEPROM and check if it is valid or if an error occured during last run. */
  currentHeight = EEPROM.read(heightStartAddress);
  if (currentHeight == 0xFF) {
    //calibrateHeight();
    currentHeight = 120;
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
      Serial.println("Function code f recived.");
        /* Execute an complete series of commands specified by the opcode in messageBuffer[1]. */
        operate(messageBuffer[1]);
        break;
      case 's':
        /* Move a servo, specified by messageBuffer[1] (gripperRot -> 0, underArmRot -> 1, overArmRot -> 2) to the angle given in messageBuffer[2] */
        Serial.println("Function code s recived.");
        Serial.println(messageBuffer[2],DEC);
        if(messageBuffer[1] == '0') { 
          Serial.println("Writing to servo.");
          servos[0].write('messageBuffer[2]');
        }
        break;       
      case 'h':
        /* Run the elevation engine with the speed specified in the messageBuffer[1] byte. (1 -> full reverse, 64 -> stop, 127 -> full forward) */
        Serial.print("Function code h recived. Writing ");
        Serial.print(messageBuffer[1], DEC);
        Serial.println(" to elevation motor.");
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
     Serial.println("Opcode 0.");
      /* Load the carparts from the table. */
      int val;
      val = digitalRead(39);
      Serial.println(val, DEC);
      break;
    case '1':
     Serial.println("Opcode 1.");
      /* Unload the carparts from the AGV */
      int val2;
      val2 = EEPROM.read(heightStartAddress);
      Serial.println(val2, DEC);
      break;
    case '2':
     Serial.println("Opcode 2.");
      /* Enter transport mode (the arm positioned to take as lite place as possible) */
      moveServos(servos, standbyAngles);
      break;
    case '3':
     Serial.println("Opcode 3.");
      /* Calibrate gripper */      
      Serial1.write(0x7f);
      //gripperGrip(150);
      break;
    case '4':
      Serial.println("Opcode 4.");
      /* Calibrate the heightsensor/encoder */
      Serial1.write(0x40);
      //gripperGrip(200);
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
/*
 * Function for changing the height of the arm. A PID-regulator or atleast PI.regulator should be implemented here.
*/ 
void modifyAltitude(int desiredHeight) {
  
  /* Write 0xFF to currentHeights EEPROM address so that incase of a power failure when moving up or down we will know that we are in an unknown state. */
  EEPROM.write(heightStartAddress, 0xFF);
  int heightDelta;
  char blackOrWhite, oldBlackOrWhite;
  oldBlackOrWhite = digitalRead(ENCODERPIN);
  /* Loop while we need to move */
  while(desiredHeight != currentHeight) {
    heightDelta = currentHeight-desiredHeight;
    /* Going up */
    if(heightDelta < 0){
      Serial1.write(0x01);
      blackOrWhite = digitalRead(ENCODERPIN);
      if(oldBlackOrWhite != blackOrWhite){
        currentHeight++;
        oldBlackOrWhite = blackOrWhite; 
      }   
    } 
    /* Going down */
    else {
      Serial1.write(0x7F);
      blackOrWhite = digitalRead(ENCODERPIN);
      if(oldBlackOrWhite != blackOrWhite){
        currentHeight--;
        oldBlackOrWhite = blackOrWhite; 
      }  
    }
  }
  Serial1.write(0x40);
  /* Write the currentHeight to EEPROM for reusage after powerloop */
  EEPROM.write(heightStartAddress, currentHeight);
}

/* 
 * Function that open/closes the gripper to the desired spacing given as the input 
 * argument desiredGripDist [mm] 
 */
void gripperGrip(int desiredGripDist) {
  Serial.print("Enter gripperGrip with desired dist ");
  Serial.println(desiredGripDist, DEC);
  /* Write 0xFF to EEPROM so that we can check if we actually know how far appart the pads are at setup. */
  EEPROM.write(gripStartAddress, 0xFF);       
  Serial.println("Written to EEPROM");
   
  /* Calculate how many steps the stepper has to take to get to the new position */
  int stepsToTake = (gripperDist-desiredGripDist) * 60;
  int i = 0;
  Serial.println(stepsToTake, DEC);
  
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
      delay(2);                            
      digitalWrite(STEPPERstepPIN, LOW);
      delay(2);
      
    }
    gripperDist = desiredGripDist;
  /* Write the current distance between the pads to non-volatile memory. (EEPROM) */
  EEPROM.write(gripStartAddress, desiredGripDist);
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


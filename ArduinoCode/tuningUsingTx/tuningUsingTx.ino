/*
 * Author : Kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 02 Jun 2018
 * 
 * External interrupts are used to read ch5 and ch6 of tx
 * 
 * Arduino Pin      Tx Channel
 *    2 (INT0)       ch5-VRA
 *    3 (INT1)       ch6-VRB
 * 
 * firstItwill check is default values stored in eeprom - if not store them
 * then based on received tx values are changed accordingly
 * 
 */

#include<EEPROM.h>

const byte pidDefaultValues[] =  {
                                    0.5*100,0,0.25*100, //Pitch p,i,d
                                    0.5*100,0,0.25, //Roll p,i,d
                                    0,0,0 //Yaw p,i,d
                                };

//EEPRPOM Locations for PID Values
#define verAddress 9 //to write default values
#define ppAddress 0
#define piAddress 1
#define pdAddress 2

#define rpAddress 3
#define riAddress 4
#define rdAddress 5

#define ypAddress 6
#define yiAddress 7
#define ydAddress 8

const byte verDefaultValue =  200; //some random value iam not sure i can use this

byte ppValue,piValue,pdValue,rpValue,riValue,rdValue,ypValue,yiValue,ydValue;


const byte EEPROM_ADDRESS[] = {ppAddress,piAddress,pdAddress,rpAddress,riAddress,rdAddress,
                             ypAddress,yiAddress,ydAddress};

byte pidValues[] = {ppValue,piValue,pdValue,rpValue,riValue,rdValue,ypValue,yiValue,ydValue};



//Variables to read ch5 and ch6
#define VRA_CHANNEL 2
#define VRB_CHANNEL 3

volatile int pwmValue[2];
volatile int prevTime[2];

void setup() {
  Serial.begin(115200);
  //interrupts for pin2 and pin3 for ch5 and ch6
  attachInterrupt(digitalPinToInterrupt(VRA_CHANNEL), risingVRA, RISING);
  attachInterrupt(digitalPinToInterrupt(VRB_CHANNEL),risingVRB,RISING);
  writeDefaultValues();//write default if haven't already
}//end of setup Fcn

void loop() {
  checkForTuningInput();   
}//end of loop Fcn

//ProceessTuningInput
void checkForTuningInput(){
  
}


//Function used for interrupts
void risingVRA() {
  attachInterrupt(digitalPinToInterrupt(VRA_CHANNEL), fallingVRA, FALLING);
  prevTime[0] = micros();
}
 
void fallingVRA() {
  attachInterrupt(digitalPinToInterrupt(VRA_CHANNEL), risingVRA, RISING);
  
  pwmValue[0] = micros()-prevTime[0];
}

void risingVRB(){
  attachInterrupt(digitalPinToInterrupt(VRB_CHANNEL),fallingVRB,FALLING);
  prevTime[1] = micros();
}

void fallingVRB(){
  attachInterrupt(digitalPinToInterrupt(VRB_CHANNEL),risingVRB,RISING);
  pwmValue[1] = micros() - prevTime[1];
}

//Functions used for EEPROM read & write
/*
 * This function will check for ver value 
 * if stored default val not equal to verDefaultValue then it will write default values to it
 */
void writeDefaultValues(){
  int defaultVal = EEPROM.read(verAddress);
  Serial.println(defaultVal);
  if (defaultVal != verDefaultValue){
    for (int i=0; i < sizeof(EEPROM_ADDRESS); i++){
      EEPROM.write(EEPROM_ADDRESS[i],pidDefaultValues[i]);
    }
    Serial.println("stored the default values");
    //once done store the version number so it wont repeat next
    EEPROM.write(verAddress,verDefaultValue);
  }//end of if
}//end pf writeDefaultValues


void readStoredEEPROMValues(){
  for (int i = 0 ; i<sizeof(EEPROM_ADDRESS); i++){
    Serial.println(EEPROM.read(EEPROM_ADDRESS[i]));
  }
}//end of checkStoredEEPROMValues Fcn

void writeEEPROMValue(byte addr, byte val){
  
}//end of readEEPROMValues()


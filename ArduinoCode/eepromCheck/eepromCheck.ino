/*
 * Author : Kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 02 June 2018
 * 
 * Storing data in eeprom and retriving it
 * 
 * PID data will be stored in EEPROM
 * 
 * version is used to check the is it firsttime to write default values
 * if version is 0 then write the default values
 */

#include<EEPROM.h>

//PID default values
//#define ppDefaultValue 0.5*100 //multiplication allow us to store float as byte
//#define piDefaultValue 0.0*100
//#define pdDefaultValue 0.25*100
//
//#define rpDefaultValue 0.5*100
//#define riDefaultValue 0.0*100
//#define rdDefaultValue 0.25*100
//
//#define ypDefaultValue 0
//#define yiDefaultValue 0
//#define ydDefaultValue 0

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

int ppValue,piValue,pdValue,rpValue,riValue,rdValue,ypValue,yiValue,ydValue;


const byte EEPROM_ADDRESS[] = {ppAddress,piAddress,pdAddress,rpAddress,riAddress,rdAddress,
                             ypAddress,yiAddress,ydAddress};

//int pp,pi,pd,rp,ri,rd,yp,yi,yd; //variabeles to store the pid values i.e readings from eeprom

byte pidValues[9]; // array to store pid values

int address = 0;
 
void setup() {
  Serial.begin(9600);
  Serial.println("EEPROM check");
  writeDefaultValues();
}//end of setup

void loop() {
  //do nothing
}//end of loop


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


void checkStoredEEPROMValues(){
  for (int i = 0 ; i<sizeof(EEPROM_ADDRESS); i++){
    Serial.println(EEPROM.read(EEPROM_ADDRESS[i]));
  }
}//end of checkStoredEEPROMValues Fcn

void writeEEPROMValues(){
  
}//end of readEEPROMValues()


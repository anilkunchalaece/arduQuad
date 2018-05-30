/*
 * Author : Kunchala Anil
 * Date : Mar 15 2018
 * Checking Motor Direction
 */
/*
 * To Arm ESC use value 60 
 * Max Speed can be acheived using 130
 * ESC - Afro ESC - 20A
 * Connect Motor to ESC 
 * Connect ESC Signal Connector 
 *      S (Yellow) - D3
 *      G (Brown) - Gnd
 *      R (Red)  - UnConnected
 *      
 *    Note : Dont Connect ESC Red Wire To arduino - cause it may consume more current than 20ma which may damage your
 *    Arduino or Computer USB Port3
 *    
 *    
 *    Motor Directions For Quad + Configuration 
 *    
 *    m0 Front - CW
 *    m1 Right - CCW
 *    m2 Back - CW
 *    m3 Left - CCW
 *        
 */
 
#include<Servo.h>


#define m0 4
#define m1 5
#define m2 6
#define m3 7

#define motorArmValue 60
#define motorStartValue 70
#define motorStopValue 50

Servo currentMotor;

int currentMotorPin = m0;

int recvValue ;

void setup() {
  Serial.begin(115200);
  currentMotor.attach(currentMotorPin);  
}

void loop() {

  if(Serial.available()){
    recvValue = Serial.parseInt();
  }//end of If
  Serial.println(recvValue);
   currentMotor.write(recvValue);
}//end of loop


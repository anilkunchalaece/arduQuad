/*
 * Author : Kunchala Anil
 * Date : Mar 3 2018
 * Controlling ESC using Arduino
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
 *    Arduino or Computer USB Port
 */
 
#include<Servo.h>

Servo myServo;
int recvValue = 0;
void setup() {
  Serial.begin(9600);
  myServo.attach(3);
}

void loop() {

  if(Serial.available()){
    recvValue = Serial.parseInt();
    Serial.println(recvValue);
    
  }
  myServo.write(recvValue);
  
}

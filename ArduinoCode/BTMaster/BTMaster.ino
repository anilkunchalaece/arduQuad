
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(3,4); // TX, RX

void setup() {
Serial.begin(9600);
//Serial.println("AT MODE");
BTSerial.begin(38400); //HC-5 Default Speed in AT Mode

}

void loop(){
 
  if(BTSerial.available()){
    Serial.write(BTSerial.read());
  }
  
}


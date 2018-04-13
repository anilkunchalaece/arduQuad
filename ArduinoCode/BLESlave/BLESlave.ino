
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(5,6); // RX, TX

void setup() {
//Serial.begin(9600);
//Serial.println("AT MODE");
BTSerial.begin(38400); //HC-5 Default Speed in AT Mode

}

void loop(){
 BTSerial.println("hi");
 delay(100);
}


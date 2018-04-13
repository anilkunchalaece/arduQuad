
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(A0,A1); // TX, RX

void setup() {
//Serial.begin(9600);
//Serial.println("AT MODE");
BTSerial.begin(38400); //HC-5 Default Speed in AT Mode

}

void loop(){
 BTSerial.println("<1.2,1.3,1.4,1,1,1,1,1,1>");
 delay(100);
}


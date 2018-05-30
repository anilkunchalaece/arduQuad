
/*
 * Author : Kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 13 Apr 2018
 * This sketch is used to send the data from BLE Slave to Master
 */
#include <SoftwareSerial.h>

//SoftwareSerial BTSerial(A0,A1); // TX, RX
SoftwareSerial BTSerial(3,4); // TX, RX

#define p 0.7
#define d 0.20
#define i 0.0

#define pp p
#define pi i
#define pd d

#define rp p
#define ri i
#define rd d

#define yp 0
#define yi 0
#define yd 0

void setup() {
Serial.begin(115200);
Serial.println("AT MODE");
BTSerial.begin(38400); //HC-5 Default Speed in AT Mode

BTSerial.print("<");
BTSerial.print(pp);
BTSerial.print(",");
BTSerial.print(pd);
BTSerial.print(",");
BTSerial.print(pi);
BTSerial.print(",");
BTSerial.print(rp);
BTSerial.print(",");
BTSerial.print(rd);
BTSerial.print(",");
BTSerial.print(ri);
BTSerial.print(",");
BTSerial.print(yp);
BTSerial.print(",");
BTSerial.print(yd);
BTSerial.print(",");
BTSerial.print(yi);
BTSerial.print(">");

}

void loop(){
// BTSerial.println("<1.2,1.3,1.4,1,1,1,1,1,1>");
if(BTSerial.available()){
  Serial.write(BTSerial.read());
}

if(Serial.available()){
  BTSerial.print(Serial.read());
}
}


/*
 * Configuring the HC-05 Bluetooth Module as Master
 * Connection To Arduino 
 *  HC-05 GND --- Arduino GND Pin
    HC-05 VCC (5V) --- Arduino 5V
    HC-05 TX --- Arduino Pin 3 (soft RX)
    HC-05 RX --- Arduino Pin 4 (soft TX)

  To Get into Command Mode Press the Little button on Right Hand Side of HC-05
  and power connect Arduino Cable to USB 
 ref - http://forum.arduino.cc/index.php?topic=284789.0
 */

#include <SoftwareSerial.h>

SoftwareSerial BTSerial(3,4); // RX, TX

void setup() {
Serial.begin(9600);
Serial.println("AT MODE");
BTSerial.begin(38400); //HC-5 Default Speed in AT Mode

}

void loop(){

  //If Data Available from HC-05 send it to the Arduino
  if(BTSerial.available()){
    Serial.write(BTSerial.read());
  }

  //IF Data Available from Arduino Send it to HC-05
  if(Serial.available()){
    BTSerial.write(Serial.read());
  }
  
}


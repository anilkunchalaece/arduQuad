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
 
 AT Commands :
AT                  :  To enter command mode
AT+NAME     :  Set/check the name of the Bluetooth module
AT+ROLE      :  set/check the role of the module; 0 – Slave, 1 – Master
AT+PSWD     :  set/check the password/PIN code
AT+ADDR     :  get module address
AT+RMAAD  :  Delete all authenticated previous paired devices
AT+CMOD    :  Set/Check connect mode
AT+UART      :  Set communication baudrate
AT+ORGL     :  Restore default
AT+RESET     :  Reset the module
AT+INIT         :  Initialize the SPP profile
AT+INQ         :  Inquiry Bluetooth Device
AT+RNAME  :  Get the Bluetooth device name
AT+LINK       :  Connect Device



 Master Configuration ::
 1. Put the module in AT mode :  To Get into Command Mode Press the Little button on Right Hand Side of HC-05 and power connect Arduino Cable to USB (HC05 bink with 2s interval)
 2. Clear all previously paired devices: AT + RMAAD
 3. Set PIN :  AT+PSWD = 1234
 4. Set Role as 1 i.e Master : AT+ROLE=1
 5. Set connect mode as “1” - This will make module to connect any module within area : AT+MODE = 1
 6. Reset : AT+ RESET

 Slave Configuration ::
 1. Put the module in AT mode 
 2. Clear all previously paired devices : AT+RMADD
 3. Set role as “0” i.e slave : AT+ROLE=0
 4. Set connect mode as “1” : AT+CMODE=1
 5. Reset : AT+RESET

 After this, if both modules are powered on, they will connect and transfer data to each other transparently i.e. It would be as if the Rx and Tx connections of the two microcontrollers were directly connected to each other
 
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


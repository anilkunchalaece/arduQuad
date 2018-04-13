
/*
 * Author : Kunchala Anil
 * Date : 13 Apr 2018
 * This sketch is used to parse the command Received via Bluetooth
 */
#include <SoftwareSerial.h>
#define MAX_DATA_LENGTH 100

SoftwareSerial BTSerial(3,4); // TX, RX

char startingChar = '<';
char endingChar = '>';

boolean storeRecvData = false;
char recvDataBuffer[MAX_DATA_LENGTH];

char strtokDelimiter[] = ",";

int index = 0; 

float pp,pd,pi,rp,rd,ri,yp,yd,yi; //pid constants for pitch,roll,yaw

void setup() {
Serial.begin(9600);
//Serial.println("AT MODE");
BTSerial.begin(38400); //HC-5 Default Speed in AT Mode

}//end of setup


void loop(){
 
  if(BTSerial.available()){
    //Serial.write(BTSerial.read());
    char recvChar = BTSerial.read();

    if(recvChar == startingChar){
      storeRecvData = true;
      index = 0; //set the index back to starting
    }//end of if
    
    if(recvChar == endingChar){
      storeRecvData = false;
      recvDataBuffer[index] = 0; //null terminating the string
      //Serial.println(recvDataBuffer);
      processReceivedData();
    }//end of if

    if(storeRecvData == true){
      if(recvChar != startingChar) {
      recvDataBuffer[index] = recvChar; //store the received char in buffer
      index = index + 1; //increment the index
    }//end of if Dumb Workaroung
    }
    
  }//end of If
  
}//end of Loop

void processReceivedData(){
  char * strtokIndex ; //this is used by strtok() as index
    strtokIndex = strtok(recvDataBuffer,strtokDelimiter);// get the first part - pp
    pp = atof(strtokIndex); // convert to float and store
    strtokIndex = strtok(NULL,strtokDelimiter); // this continues where the previous call left off - get pd
    pd = atof(strtokIndex);
    Serial.println(pd);
    strtokIndex = strtok(NULL,strtokDelimiter); // get pi
    pi = atof(strtokIndex);
    strtokIndex = strtok(NULL,strtokDelimiter); // get rp
    rp = atof(strtokIndex);
    strtokIndex = strtok(NULL,strtokDelimiter); // get rd
    rd = atof(strtokIndex);
    strtokIndex = strtok(NULL,strtokDelimiter); // get ri
    ri = atof(strtokIndex); 
    strtokIndex = strtok(NULL,strtokDelimiter); // get yp
    yp = atof(strtokIndex); 
    strtokIndex = strtok(NULL,strtokDelimiter); // get yd
    yd = atof(strtokIndex);
    strtokIndex = strtok(NULL,strtokDelimiter); // get yi
    yi = atof(strtokIndex); 
}//end of processReceivedData



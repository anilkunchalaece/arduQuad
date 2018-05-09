import serial
import threading

arduinoData = serial.Serial('/dev/ttyACM0', 115200) #Creating our serial object named arduinoData

def handleData(data) :
    print (data)

def readFromPort(ser):
    while True:
        reading = ser.readline().decode()
        handleData(reading)

#thread = threading.Thread(target=readFromPort,args=(arduinoData))
#thread.start()
readFromPort(arduinoData)

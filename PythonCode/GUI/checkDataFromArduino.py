import serial
import re
import schedule,time

arduinoData = serial.Serial('/dev/ttyACM1', 115200) #Creating our serial object named arduinoData



def checkForReadings(data):
    y = re.findall(r"y([-+]?\d*\.\d+|\d+)",data)
    p = re.findall(r"p([-+]?\d*\.\d+|\d+)",data)
    r = re.findall(r"r([-+]?\d*\.\d+|\d+)",data)
    ps = re.findall(r"ps([-+]?\d*\.\d+|\d+)",data)
    rs = re.findall(r"rs([-+]?\d*\.\d+|\d+)",data)
    po = re.findall(r"po([-+]?\d*\.\d+|\d+)",data)
    ro = re.findall(r"ro([-+]?\d*\.\d+|\d+)",data)
    m0 = re.findall(r"m0-([-+]?\d*\.\d+|\d+)",data)
    m1 = re.findall(r"m1-([-+]?\d*\.\d+|\d+)",data)
    m2 = re.findall(r"m2-([-+]?\d*\.\d+|\d+)",data)
    m3 = re.findall(r"m3-([-+]?\d*\.\d+|\d+)",data)
    print [y,p,r,ps,rs,po,ro,m0,m1,m2,m3]

def checkData():
    # print "check data"
    recvString = '' # to store the data
    while (arduinoData.inWaiting() > 0):
        # print arduinoData.read()
        recvString = recvString + arduinoData.read()
    checkForReadings(recvString)

schedule.every(0.1).seconds.do(checkData)

while True:
    schedule.run_pending()


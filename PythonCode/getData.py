import serial
from matplotlib import pyplot as plt

yaw = []
pitch = []
roll = []
frontMotorLeft = []
frontMotorRight = []
backMotorLeft = []
backMotorRight = []

arduinoData = serial.Serial('/dev/ttyACM0', 115200) #Creating our serial object named arduinoData
plt.ion() #matplotlib interactive mode
def drawNow():
    plt.plot(yaw)
    plt.plot(pitch)
    plt.plot(roll)
    plt.show()
    # plt.pause(0.0000000001)
fHandler = open('data.csv','a')
while True :
    while arduinoData.inWaiting() == 0:
        # print "waiting for Data"
        pass
    recvString = arduinoData.readline()
    print recvString
    startingLetterIndex = recvString.find("<")
    endingLetterIndex = recvString.find(">")
    if startingLetterIndex != -1 and endingLetterIndex != -1 :
        recvString = recvString[startingLetterIndex+1:endingLetterIndex]
        fHandler.write(recvString+'\n')
        # data = recvString.split(";")
        # yaw.append(data[0])
        # pitch.append(data[1])
        # roll.append(data[2])
        # drawNow()
    
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
import serial

yaw = []
pitch = []
roll = []
frontMotorLeft = []
frontMotorRight = []
backMotorLeft = []
backMotorRight = []

arduinoData = serial.Serial('/dev/ttyACM0', 115200) #Creating our serial object named arduinoData

# def drawNow():
#     pg.plot(yaw)
#     pg.plot(pitch)
#     pg.plot(roll)
pgWidget = pg.plot(title="yaw pitch roll")    
pgWidget.plot(yaw,pen=(1,3))
pgWidget.plot(pitch,pen=(2,3))
pgWidget.plot(roll,pen=(3,3))


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
        data = recvString.split(",")
        yaw.append(float(data[4]))
        pitch.append(float(data[5]))
        roll.append(float(data[6]))
        pg.QtGui.QApplication.processEvents()
pgWidget.QtGui.QApplication.exec_()
        # drawNow()


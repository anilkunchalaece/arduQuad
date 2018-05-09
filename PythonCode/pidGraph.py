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
counter = 0
arduinoData = serial.Serial('/dev/ttyACM0', 115200) #Creating our serial object named arduinoData

pw = pg.plot()

while True :
    while arduinoData.inWaiting() == 0:
        # print "waiting for Data"
        pass
    counter = counter+1;
    recvString = arduinoData.readline()
    print recvString + "=========="
    startingLetterIndex = recvString.find("<")
    endingLetterIndex = recvString.find(">")
    if startingLetterIndex != -1 and endingLetterIndex != -1 :
        recvString = recvString[startingLetterIndex+1:endingLetterIndex]
        data = recvString.split(",")
        pitch.append(float(data[5]))
        print "received pitch value"+data[5]
        pw.plot(pitch,clear=True)
        pw.plot([0]*len(pitch))
        pg.QtGui.QApplication.processEvents()
pw.QtGui.QApplication.exec_()
        # drawNow()


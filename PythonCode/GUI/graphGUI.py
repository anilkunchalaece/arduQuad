'''
Author : Kunchala Anil
Email : anilkunchalaece@gmail.com
Creating simple gui to visualize the data from QuadCopter
It basically consist of PID graphs for Pitch and Roll

Ref : https://stackoverflow.com/questions/18080170/what-is-the-easiest-way-to-achieve-realtime-plotting-in-pyqtgraph?

'''
from PyQt4 import QtGui,QtCore
from PyQt4.Qt import *
import pyqtgraph as pg
import numpy as np
import serial
import re

arduinoData = serial.Serial('/dev/ttyACM0', 115200) #Creating our serial object named arduinoData


pitchVal = []
rollVal = []
yawVal = []

pitchSetPoint = []
rollSetPoint = []
yawSetpoint = []


m0Val = 0
m1Val = 0
m2Val = 0
m3Val = 0



#Always start by initializing Qt
app = QtGui.QApplication([])

#Define a Top level widget to hold everything
w = QtGui.QWidget()

#Create some widgets to place inside
btn = QtGui.QPushButton('Press me')
recvText = QtGui.QLabel()
recvText.wordWrap()

listw = QtGui.QListWidget()
txWidget = QtGui.QWidget()
pitchPlot = pg.PlotWidget()
rollPlot = pg.PlotWidget()
motorWidget = QtGui.QWidget()

#parts in throttle widget
txThrottleLable = QtGui.QLabel("Throttle")
txThrottleLable.setAlignment(Qt.AlignCenter)
txThrottle = QtGui.QLabel()
txThrottle.setAlignment(Qt.AlignCenter)
#txHBoxLayout = layout(QHBoxLayout)
#txHBoxLayout.

txPitchLable = QtGui.QLabel("Pitch")
txPitchLable.setAlignment(Qt.AlignCenter)
txPitch = QtGui.QLabel()
txPitch.setAlignment(Qt.AlignCenter)

txRollLable = QtGui.QLabel("Roll")
#txRoll.setAlignment(Qt.AlignCenter)
txRoll = QtGui.QLabel()
txRoll.setAlignment(Qt.AlignCenter)


##Parts in motor Widget
frontMotorLable = QtGui.QLabel("Front")
frontMotorLable.setAlignment(Qt.AlignCenter)
frontMotor = QtGui.QProgressBar()
frontMotor.setRange(1200,1800)
frontMotor.text()
frontMotor.setTextVisible (True)


rightMotorLable = QtGui.QLabel("Right")
rightMotorLable.setAlignment(Qt.AlignCenter)
rightMotor = QtGui.QProgressBar()
rightMotor.setRange(1200,1800)
rightMotor.text()
rightMotor.setTextVisible (True)

leftMotorLable = QtGui.QLabel("Left")
leftMotorLable.setAlignment(Qt.AlignCenter)
leftMotor = QtGui.QProgressBar()
leftMotor.setRange(1200,1800)
leftMotor.text()
leftMotor.setTextVisible (True)

rearMotorLabel = QtGui.QLabel("Rear")
rearMotorLabel.setAlignment(Qt.AlignCenter)
rearMotor = QtGui.QProgressBar()
rearMotor.setRange(1200,1800)
rearMotor.text()
rearMotor.setTextVisible (True)


##Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)
w.setWindowTitle("KSRMQuad Monitor By Kunchala Anil")

#Layout for Motor Widget
motorLayout = QtGui.QGridLayout()
motorWidget.setLayout(motorLayout)

#addwidgets to motorLayout
motorLayout.addWidget(frontMotorLable,0,0)
motorLayout.addWidget(frontMotor,1,0)
motorLayout.addWidget(rightMotorLable,2,0)
motorLayout.addWidget(rightMotor,3,0)

motorLayout.addWidget(leftMotorLable,0,1)
motorLayout.addWidget(leftMotor,1,1)
motorLayout.addWidget(rearMotorLabel,2,1)
motorLayout.addWidget(rearMotor,3,1)

# add widgets to layout in their proper postion
#layout.addWidget(btn,0,0)
layout.addWidget(recvText,2,0)
layout.addWidget(listw,1,0)
layout.addWidget(pitchPlot,0,1)
layout.addWidget(rollPlot,1,1)
layout.addWidget(motorWidget,2,1)
#Display widget as a new window
w.show()

timer = pg.QtCore.QTimer()

def update():
    	checkData()
        x1 = np.arange(len(pitchVal))
        pitchPlot.addLegend()
        pitchPlot.plot(x1,pitchVal,clear=True,pen=(1,2),name="Pitch")
		#ref : https://stackoverflow.com/questions/48590354/pyqtgraph-plotwidget-multiple-y-axis-plots-in-wrong-area?
        pitchPlot.addItem(pg.PlotCurveItem(x1,pitchSetPoint,clear=True,pen=(2,2),name="Pitch setPoints"))

        x2 = np.arange(len(rollVal))
        rollPlot.addLegend()
        rollPlot.plot(x2,rollVal,clear=True,pen=(1,2),name="Roll")
        rollPlot.addItem(pg.PlotCurveItem(x2,rollSetPoint,clear=True,pen=(2,2),name="Roll setPoint"))

timer.timeout.connect(update)
timer.start(15)

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
    # rr = re.findall(r"rr([-+]?\d*\.\d+|\d+)",data)
    # rp = re.findall(r"rp([-+]?\d*\.\d+|\d+)",data)
    # rt = re.findall(r"rt([-+]?\d*\.\d+|\d+)",data)
    
    if len(p) > 0 and len(ps) > 0:
    	pitchVal.append(float(p[0]))
        pitchSetPoint.append(float(ps[0]))
        while len(pitchVal) > 100 :
            pitchVal.pop(0) #remove the first element
            pitchSetPoint.pop(0)

    if len(r) > 0 and len(rs) > 0:
    	rollVal.append(float(r[0]))
        rollSetPoint.append(float(rs[0]))
        while len(rollVal) > 100 :
            rollVal.pop(0)
            rollSetPoint.pop(0)

    if len(m0) > 0 :
        m0Val = float(m0[0])
        frontMotor.setValue(m0Val)
    if len(m1) > 0 :
    	m1Val = float(m1[0])
        rightMotor.setValue(m1Val)
    if len(m2) > 0 :
    	m2Val = float(m2[0])
        rearMotor.setValue(m2Val)
    if len(m3) > 0 :
    	m3Val = float(m3[0])
        leftMotor.setValue(m3Val)
    # if len(rr) > 0 :
    #     txRollVal = float(rr[0])
    # if len(rp) > 0 :
    #     txPitchVal = float(rp[0])
    # if len(rt) > 0 :
    #     txThrottleVal = float(rt[0])
    # print [y,p,r,ps,rs,po,ro,m0,m1,m2,m3]

def checkData():
    # print "check data"
    recvString = '' # to store the data
    while (arduinoData.inWaiting() > 0):
        # print arduinoData.read()
        recvString = recvString + arduinoData.read()
        #recvText.setText(recvString)
        print recvString
    checkForReadings(recvString)

#start qt event loop
app.processEvents()
app.exec_()

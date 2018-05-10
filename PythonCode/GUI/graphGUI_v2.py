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


m0Val = [] # Front 
m1Val = [] # Right
m2Val = [] # Rear
m3Val = [] # Left



#Always start by initializing Qt
app = QtGui.QApplication([])

#Define a Top level widget to hold everything
w = QtGui.QWidget()

#Create some widgets to place inside
recvText = QtGui.QLabel()
recvText.wordWrap()

instantData = QtGui.QWidget()

pitchPlot = pg.PlotWidget()
rollPlot = pg.PlotWidget()
yawPlot = pg.PlotWidget()

motorPlot = pg.PlotWidget()


##Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)
w.setWindowTitle("KSRMQuad Monitor By Kunchala Anil")

# add widgets to layout in their proper postion
#layout.addWidget(btn,0,0)
layout.addWidget(pitchPlot,0,1)
layout.addWidget(rollPlot,1,1)
layout.addWidget(yawPlot,2,1)
layout.addWidget(motorPlot,2,0)
layout.addWidget(instantData,0,0,2,1)

## create  a grid layout to display instantaneous data
instLayout = QtGui.QGridLayout()
instantData.setLayout(instLayout)

rollInstLable = QtGui.QLabel("Roll")
rollInstLable.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
rollInstVal = QtGui.QLabel()
rollInstVal.setFrameStyle(QFrame.WinPanel | QFrame.Raised)
rollInstVal.setText("45")
pitchInstLable = QtGui.QLabel("Pitch")
pitchInstLable.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
pitchInstVal = QtGui.QLabel()
pitchInstVal.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
pitchInstVal.setText("23")
yawInstLable = QtGui.QLabel("Yaw")
yawInstLable.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
yawInstVal = QtGui.QLabel()
yawInstVal.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)

rollPIDOutInstLable = QtGui.QLabel("Roll PID Output")
rollPIDOutInstLable.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
rollPIDOutInstVal = QtGui.QLabel()
rollPIDOutInstVal.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
pitchPIDOutInstLable = QtGui.QLabel("Pitch PID Output")
pitchPIDOutInstLable.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
pitchPIDOutInstVal = QtGui.QLabel()
pitchInstVal.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
yawPIDOutInstLable = QtGui.QLabel("Yaw PID Output")
yawPIDOutInstLable.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
yawPIDOutInstVal = QtGui.QLabel()
yawPIDOutInstVal.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)

rollSetPointInstLable = QtGui.QLabel("Roll SetPoint")
rollSetPointInstVal = QtGui.QLabel()
pitchSetPointInstLable = QtGui.QLabel("Pitch SetPoint")
pitchSetPointInstVal = QtGui.QLabel()
yawSetPointInstLable = QtGui.QLabel("Yaw SetPoint")
yawSetPointInstVal = QtGui.QLabel()

frontMotorInstLable = QtGui.QLabel("Front")
frontMotorInstVal = QtGui.QLabel()
rightMotorInstLable = QtGui.QLabel("Right")
rightMotorInstVal = QtGui.QLabel()
rearMotorInstLable = QtGui.QLabel("Rear")
rearMotorInstVal = QtGui.QLabel()
leftMotorInstLable = QtGui.QLabel("Left")
leftMotorInstVal = QtGui.QLabel()

ch1InstLable = QtGui.QLabel("CH1-Roll")
ch1InstVal = QtGui.QLabel()
ch2InstLable = QtGui.QLabel("CH2-Pitch")
ch2InstVal = QtGui.QLabel()
ch3InstLable = QtGui.QLabel("CH3-Throttle")
ch3InstVal = QtGui.QLabel()
ch4InstLable = QtGui.QLabel("CH4-Yaw")
ch4InstVal = QtGui.QLabel()

instLayout.addWidget(rollInstLable,0,0)
instLayout.addWidget(rollInstVal,0,1)
instLayout.addWidget(pitchInstLable,1,0)
instLayout.addWidget(pitchInstVal,1,1)
instLayout.addWidget(yawInstLable,2,0)
instLayout.addWidget(yawInstVal,2,1)

instLayout.addWidget(rollPIDOutInstLable,3,0)
instLayout.addWidget(rollPIDOutInstVal,3,1)
instLayout.addWidget(pitchPIDOutInstLable,4,0)
instLayout.addWidget(pitchPIDOutInstVal,4,1)
instLayout.addWidget(yawPIDOutInstLable,5,0)
instLayout.addWidget(yawPIDOutInstVal,5,1)

instLayout.addWidget(frontMotorInstLable,6,0)
instLayout.addWidget(frontMotorInstVal,6,1)
instLayout.addWidget(rightMotorInstLable,7,0)
instLayout.addWidget(rightMotorInstVal,7,1)
instLayout.addWidget(rearMotorInstLable,8,0)
instLayout.addWidget(rearMotorInstVal,8,1)
instLayout.addWidget(leftMotorInstLable,9,0)
instLayout.addWidget(leftMotorInstVal,9,1)

instLayout.addWidget(ch1InstLable,10,0)
instLayout.addWidget(ch1InstVal,10,1)
instLayout.addWidget(ch2InstLable,11,0)
instLayout.addWidget(ch2InstVal,11,1)
instLayout.addWidget(ch3InstLable,12,0)
instLayout.addWidget(ch3InstVal,12,1)


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

        x3 = np.arange(len(m0Val))
        motorPlot.addLegend()
        motorPlot.plot(x3,m0Val,clear=True,pen=(1,4),name="Front")
        motorPlot.addItem(pg.PlotCurveItem(x3,m1Val,clear=True,pen=(2,4),name="Right"))
        motorPlot.addItem(pg.PlotCurveItem(x3,m2Val,clear=True,pen=(3,4),name="Rear"))
        motorPlot.addItem(pg.PlotCurveItem(x3,m3Val,clear=True,pen=(4,4),name="Left"))
        

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

    if len(m0) > 0  and len(m1) > 0 and len(m2) > 0 and len(m3) > 0:
        m0Val.append(float(m0[0]))
        m1Val.append(float(m1[0]))
        m2Val.append(float(m2[0]))
        m3Val.append(float(m3[0]))     
        while len(m0Val) > 100 :
            m0Val.pop(0)
            m1Val.pop(0)
            m2Val.pop(0)
            m3Val.pop(0) 

    # print [y,p,r,ps,rs,po,ro,m0,m1,m2,m3]

def checkData():
    # print "check data"
    recvString = '' # to store the data
    while (arduinoData.inWaiting() > 0):
        # print arduinoData.read()
        recvString = recvString + arduinoData.read()
        #recvText.setText(recvString)
    checkForReadings(recvString)

#start qt event loop
app.processEvents()
app.exec_()

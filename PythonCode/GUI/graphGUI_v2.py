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
layout.addWidget(instantData,0,0)
layout.addWidget(motorPlot,1,0)

## create  a grid layout to display instantaneous data
instLayout = QtGui.QGridLayout()
instantData.setLayout(instLayout)

rollInstLable = QtGui.QLabel("Roll")
pitchInstLable = QtGui.QLabel("Pitch")
yawInstLable = QtGui.QLabel("Yaw")

rollPIDOutInstLable = QtGui.QLabel("Roll PID Out")
pitchPIDOutInstLable = QtGui.QLabel("Pitch PID Out")
yawPIDOutInstLable = QtGui.QLabel("Yaw PID Out")

rollSetPointInstLable = QtGui.QLabel("Roll SetPoint")
pitchSetPointInstLable = QtGui.QLabel("Pitch SetPoint")
yawSetPointInstLable = QtGui.QLabel("Yaw SetPoint")

frontMotorInstLable = QtGui.QLabel("Front")
rightMotorInstLable = QtGui.QLabel("Right")
rearMotorInstLable = QtGui.QLabel("Rear")
leftMotorInstLable = QtGui.QLabel("Left")

ch1InstLable = QtGui.QLabel("CH1-Roll")
ch2InstLable = QtGui.QLabel("CH2-Pitch")
ch3InstLable = QtGui.QLabel("CH3-Throttle")
ch4InstLable = QtGui.QLabel("CH4-Yaw")

instLayout.addWidget(rollInstLable,0,0)
instLayout.addWidget(pitchInstLable,1,0)
instLayout.addWidget(yawInstLable,2,0)

instLayout.addWidget(rollPIDOutInstLable,3,0)
instLayout.addWidget(pitchPIDOutInstLable,4,0)
instLayout.addWidget(yawPIDOutInstLable,5,0)

instLayout.addWidget(rollSetPointInstLable,6,0)
instLayout.addWidget(pitchSetPointInstLable,7,0)


instLayout.addWidget(frontMotorInstLable,0,1)
instLayout.addWidget(rightMotorInstLable,1,1)
instLayout.addWidget(rearMotorInstLable,2,1)
instLayout.addWidget(leftMotorInstLable,3,1)

instLayout.addWidget(ch1InstLable,4,1)
instLayout.addWidget(ch2InstLable,5,1)
instLayout.addWidget(ch3InstLable,6,1)
instLayout.addWidget(yawSetPointInstLable,7,1)

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
timer.start(100)

def checkForReadings(data):
    y = re.findall(r"y([-+]?\d*\.\d+|\d+)",data)
    p = re.findall(r"p([-+]?\d*\.\d+|\d+)",data)
    r = re.findall(r"r([-+]?\d*\.\d+|\d+)",data)
    ps = re.findall(r"ps([-+]?\d*\.\d+|\d+)",data)
    rs = re.findall(r"rs([-+]?\d*\.\d+|\d+)",data)
    ys = re.findall(r"ys([-+]?\d*\.\d+|\d+)",data)
    po = re.findall(r"po([-+]?\d*\.\d+|\d+)",data)
    ro = re.findall(r"ro([-+]?\d*\.\d+|\d+)",data)
    m0 = re.findall(r"m0-([-+]?\d*\.\d+|\d+)",data)
    m1 = re.findall(r"m1-([-+]?\d*\.\d+|\d+)",data)
    m2 = re.findall(r"m2-([-+]?\d*\.\d+|\d+)",data)
    m3 = re.findall(r"m3-([-+]?\d*\.\d+|\d+)",data)
    rr = re.findall(r"rr([-+]?\d*\.\d+|\d+)",data)
    rp = re.findall(r"rp([-+]?\d*\.\d+|\d+)",data)
    rt = re.findall(r"rt([-+]?\d*\.\d+|\d+)",data)
    
    if len(p) > 0 and len(ps) > 0:
    	pitchVal.append(float(p[0]))
        pitchSetPoint.append(float(ps[0]))
        pitchInstLable.setText('Pitch --> '+p[0])
        pitchSetPointInstLable.setText('Pitch SetPoint --> '+ps[0])
        while len(pitchVal) > 100 :
            pitchVal.pop(0) #remove the first element
            pitchSetPoint.pop(0)

    if len(r) > 0 and len(rs) > 0:
        rollInstLable.setText('Roll --> ' + r[0])
        rollSetPointInstLable.setText('Roll SetPoint --> '+rs[0])
    	rollVal.append(float(r[0]))
        rollSetPoint.append(float(rs[0]))
        while len(rollVal) > 100 :
            rollVal.pop(0)
            rollSetPoint.pop(0)


    if len(m0) > 0  and len(m1) > 0 and len(m2) > 0 and len(m3) > 0:
        frontMotorInstLable.setText('Front --> '+m0[0])
        rightMotorInstLable.setText('Right --> '+m1[0])
        rearMotorInstLable.setText('Rear  --> '+m2[0])
        leftMotorInstLable.setText('Left  --> '+m3[0])
        m0Val.append(float(m0[0]))
        m1Val.append(float(m1[0]))
        m2Val.append(float(m2[0]))
        m3Val.append(float(m3[0]))     
        while len(m0Val) > 100 :
            m0Val.pop(0)
            m1Val.pop(0)
            m2Val.pop(0)
            m3Val.pop(0) 

    if len(rr) > 0 :
        ch1InstLable.setText('CH1 --> '+rr[0])
    if len(rp) > 0 :
        ch2InstLable.setText('CH2 --> '+rp[0])
    if len(rt) > 0 :
        ch3InstLable.setText('CH3 --> '+rt[0])
    if len(po) > 0 :
        pitchPIDOutInstLable.setText('Pitch PID Out --> '+po[0])
    if len(ro) > 0 :
        rollPIDOutInstLable.setText('Roll PID Out --> '+ro[0])
    # print [y,p,r,ps,rs,po,ro,m0,m1,m2,m3]

def checkData():
    # print "check data"
    recvString = '' # to store the data
    while (arduinoData.inWaiting() > 0):
        # print arduinoData.read()
        recvString = recvString + arduinoData.read()
        #recvText.setText(recvString)
    checkForReadings(recvString)
    app.processEvents()

#start qt event loop
app.processEvents()
app.exec_()

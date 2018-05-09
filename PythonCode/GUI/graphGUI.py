'''
Author : Kunchala Anil
Email : anilkunchalaece@gmail.com
Creating simple gui to visualize the data from QuadCopter
It basically consist of PID graphs for Pitch and Roll

Ref : https://stackoverflow.com/questions/18080170/what-is-the-easiest-way-to-achieve-realtime-plotting-in-pyqtgraph?

'''
from PyQt4 import QtGui,QtCore
import pyqtgraph as pg
import numpy as np
import serial
import re

arduinoData = serial.Serial('/dev/ttyACM1', 115200) #Creating our serial object named arduinoData


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
text = QtGui.QLineEdit('enter text')
listw = QtGui.QListWidget()
pitchPlot = pg.PlotWidget()
rollPlot = pg.PlotWidget()


##Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)


# add widgets to layout in their proper postion
layout.addWidget(btn,0,0)
layout.addWidget(text,1,0)
layout.addWidget(listw,2,0)
layout.addWidget(pitchPlot,0,1)
layout.addWidget(rollPlot,1,1)

#Display widget as a new window
w.show()

timer = pg.QtCore.QTimer()

def update():
    	checkData()
        x1 = np.arange(len(pitchVal))
        pitchPlot.plot(x1,pitchVal,clear=True,pen=(1,2))
		#ref : https://stackoverflow.com/questions/48590354/pyqtgraph-plotwidget-multiple-y-axis-plots-in-wrong-area?
        pitchPlot.addItem(pg.PlotCurveItem(x1,pitchSetPoint,clear=True,pen=(2,2)))

        x2 = np.arange(len(rollVal))
        rollPlot.plot(x2,rollVal,clear=True,pen=(1,2))
        rollPlot.addItem(pg.PlotCurveItem(x2,rollSetPoint,clear=True,pen=(2,2)))

timer.timeout.connect(update)
timer.start(16)

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
    if len(p) > 0 and len(ps) > 0:
    	pitchVal.append(float(p[0]))
        pitchSetPoint.append(float(ps[0]))

    if len(r) > 0 and len(rs) > 0:
    	rollVal.append(float(r[0]))
        rollSetPoint.append(float(rs[0]))

    if len(m0) > 0 :
        m0Val = float(m0[0])
    if len(m1) > 0 :
    	m1Val = float(m1[0])
    if len(m2) > 0 :
    	m2Val = float(m2[0])
    if len(m3) > 0 :
    	m3Val = float(m3[0])
    # print [y,p,r,ps,rs,po,ro,m0,m1,m2,m3]

def checkData():
    # print "check data"
    recvString = '' # to store the data
    while (arduinoData.inWaiting() > 0):
        # print arduinoData.read()
        recvString = recvString + arduinoData.read()
    checkForReadings(recvString)


#start qt event loop
app.exec_()

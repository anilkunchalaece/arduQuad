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
import datetime

arduinoData = serial.Serial('/dev/ttyACM0', 115200) #Creating our serial object named arduinoData

pidPPValue = input("Enter PitchP Val \t")
pidPDValue = input("Enter PitchD Val \t")
pidPIValue = input("Enter pitchI Val \t")

pidRPValue = input("Enter RollP Val \t")
pidRDValue = input("Enter RollD Val \t")
pidRIValue = input("Enter RollI Val \t")

fileName = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
fHandler = open('debug/'+fileName+'.csv','w')
fHandler.write('p,r,po,ro,ps,rs,m0,m1,m2,m3 \t')
fHandler.write('pp'+str(pidPPValue)+'pd'+str(pidPDValue)+'pi'+str(pidPIValue)+'rp'+str(pidRPValue)+'rd'+str(pidRDValue)+'ri'+str(pidRIValue)+'\n')

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

x1 = []
x2 = []
x3 = []
x4 = []

#Always start by initializing Qt
app = QtGui.QApplication([])

#Define a Top level widget to hold everything
w = QtGui.QWidget()

#Create some widgets to place inside
recvText = QtGui.QLabel()
recvText.wordWrap()

pitchPlot = pg.PlotWidget()
rollPlot = pg.PlotWidget()
yawPlot = pg.PlotWidget()

instantData = QtGui.QWidget()
motorPlot = pg.PlotWidget()
pidData = QtGui.QWidget()


pitchPlot.addLegend()
pitchPlotPlt = pitchPlot.plot(x1,pitchVal,clear=True,pen=(1,2),name="Pitch")
#ref : https://stackoverflow.com/questions/48590354/pyqtgraph-plotwidget-multiple-y-axis-plots-in-wrong-area?
#pitchPlot.addItem(pg.PlotCurveItem(x1,pitchSetPoint,clear=True,pen=(2,2),name="Pitch setPoints"))
pitchPlotPlt2 = pg.PlotCurveItem(x1,pitchSetPoint,clear=True,pen=(2,2),name="Pitch setPoints")
pitchPlot.addItem(pitchPlotPlt2)
pitchPlot.setYRange(-32,32)

rollPlot.addLegend()
rollPlotPlt = rollPlot.plot(x2,rollVal,clear=True,pen=(1,2),name="Roll")
rollPlotPlt2 = pg.PlotCurveItem(x2,rollSetPoint,clear=True,pen=(2,2),name="Roll setPoint")
rollPlot.addItem(rollPlotPlt2)
rollPlot.setYRange(-32,32)

yawPlot.addLegend()
yawPlotPlt = yawPlot.plot(x4,yawVal,clear=True,pen=(1,2),name="Yaw")
yawPlotPlt2 = pg.PlotCurveItem(x4,yawSetpoint,clear=True,pen=(2,2),name="Yaw setPoint")
yawPlot.addItem(yawPlotPlt2)
yawPlot.setYRange(-32,32)

motorPlot.addLegend()
motorPlotPlt = motorPlot.plot(x3,m0Val,clear=True,pen=(1,4),name="Right Front")
motorPlotPlt2 = pg.PlotCurveItem(x3,m1Val,clear=True,pen=(2,4),name="Right Rear")
motorPlot.addItem(motorPlotPlt2)
motorPlotPlt3 = pg.PlotCurveItem(x3,m2Val,clear=True,pen=(3,4),name="Left Rear")
motorPlot.addItem(motorPlotPlt3)
motorPlotPlt4 = pg.PlotCurveItem(x3,m3Val,clear=True,pen=(4,4),name="Left Front")
motorPlot.addItem(motorPlotPlt4)

#pitchPlotPlt = pitchPlot.plot(x1,pitchVal,clear=True,pen=(1,2),name="Pitch")


credits = QtGui.QLabel("KSRM ArduQuad Parameter Monitor By Kunchala Anil")

##Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)
w.setWindowTitle("KSRMQuad Monitor")

# add widgets to layout in their proper postion
#layout.addWidget(btn,0,0)
layout.addWidget(pitchPlot,0,1)
layout.addWidget(rollPlot,1,1)
layout.addWidget(yawPlot,2,1)
layout.addWidget(instantData,0,0)
layout.addWidget(motorPlot,1,0)
layout.addWidget(pidData,2,0)
layout.addWidget(credits,3,0)

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
#yawSetPointInstLable = QtGui.QLabel("Yaw SetPoint")

frontMotorInstLable = QtGui.QLabel("Right Front")
rightMotorInstLable = QtGui.QLabel("Right Rear")
rearMotorInstLable = QtGui.QLabel("Left Rear")
leftMotorInstLable = QtGui.QLabel("Left Front")

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
instLayout.addWidget(ch4InstLable,7,1)

##Create a layout for the pidData
pidDataLayout = QtGui.QGridLayout()
pidData.setLayout(pidDataLayout)

p = QtGui.QLabel('Proportional')
p.setAlignment(Qt.AlignCenter)
i = QtGui.QLabel('Integral')
i.setAlignment(Qt.AlignCenter)
d = QtGui.QLabel('Derivative')
d.setAlignment(Qt.AlignCenter)

plabel = QtGui.QLabel('Pitch')
plabel.setAlignment(Qt.AlignBottom)
pp = QtGui.QDoubleSpinBox()
pp.setSingleStep(0.01)
pi = QtGui.QDoubleSpinBox()
pi.setSingleStep(0.01)
pd = QtGui.QDoubleSpinBox()
pd.setSingleStep(0.01)

rlabel = QtGui.QLabel('Roll')
rlabel.setAlignment(Qt.AlignBottom)
rp = QtGui.QDoubleSpinBox()
rp.setSingleStep(0.01)
ri = QtGui.QDoubleSpinBox()
ri.setSingleStep(0.01)
rd = QtGui.QDoubleSpinBox()
rd.setSingleStep(0.01)

ylabel = QtGui.QLabel('Yaw')
ylabel.setAlignment(Qt.AlignBottom)
yp = QtGui.QDoubleSpinBox()
yp.setSingleStep(0.01)
yi = QtGui.QDoubleSpinBox()
yi.setSingleStep(0.01)
yd = QtGui.QDoubleSpinBox()
yd.setSingleStep(0.01)

def setTuningsBtnClicked():
    print("set tunings clicked")
    valToSend = "<"+str(pp.value())+","+str(pd.value())+","+str(pi.value())+","
    valToSend = valToSend + str(rp.value()) + ","+str(rd.value())+","+str(ri.value())+","
    valToSend = valToSend + str(yp.value()) + ","+str(yd.value())+","+str(yi.value())+ ">\r\n"
    print valToSend
    arduinoData.write(valToSend)

setTuningsBtn = QtGui.QPushButton("Set PID")
setTuningsBtn.clicked.connect(setTuningsBtnClicked)

rawData = QtGui.QLabel("Raw Data From Quad")
rawDataString = QtGui.QLabel()
rawDataString.setMaximumSize(500,50)
rawDataString.setStyleSheet('background-color : black ; color : white')


pidDataLayout.addWidget(p,1,0)
pidDataLayout.addWidget(i,2,0)
pidDataLayout.addWidget(d,3,0)
pidDataLayout.addWidget(plabel,0,1)

pidDataLayout.addWidget(pp,1,1)
pidDataLayout.addWidget(pi,2,1)
pidDataLayout.addWidget(pd,3,1)
pidDataLayout.addWidget(rlabel,0,2)
pidDataLayout.addWidget(rp,1,2)
pidDataLayout.addWidget(ri,2,2)
pidDataLayout.addWidget(rd,3,2)
pidDataLayout.addWidget(ylabel,0,3)
pidDataLayout.addWidget(yp,1,3)
pidDataLayout.addWidget(yi,2,3)
pidDataLayout.addWidget(yd,3,3)

pidDataLayout.addWidget(setTuningsBtn,1,5,3,1,Qt.AlignCenter)

pidDataLayout.addWidget(rawData,4,0,1,3)
pidDataLayout.addWidget(rawDataString,5,0,1,3)
#Display widget as a new window
w.show()

timer = pg.QtCore.QTimer()



def update():
    	checkData()
        x1 = np.arange(len(pitchVal))
        pitchPlotPlt.setData(x=x1,y=pitchVal)
        pitchPlotPlt2.setData(x=x1,y=pitchSetPoint)
        # pitchPlot.addLegend()
        # pitchPlot.plot(x1,pitchVal,clear=True,pen=(1,2),name="Pitch")
		# #ref : https://stackoverflow.com/questions/48590354/pyqtgraph-plotwidget-multiple-y-axis-plots-in-wrong-area?
        # pitchPlot.addItem(pg.PlotCurveItem(x1,pitchSetPoint,clear=True,pen=(2,2),name="Pitch setPoints"))
        # pitchPlot.setYRange(-32,32)

        x2 = np.arange(len(rollVal))
        rollPlotPlt.setData(x=x2,y=rollVal)
        rollPlotPlt2.setData(x=x2,y=rollSetPoint)        
        # rollPlot.addLegend()
        # rollPlot.plot(x2,rollVal,clear=True,pen=(1,2),name="Roll")
        # rollPlot.addItem(pg.PlotCurveItem(x2,rollSetPoint,clear=True,pen=(2,2),name="Roll setPoint"))
        # rollPlot.setYRange(-32,32)

        x4 = np.arange(len(yawVal))
        yawPlotPlt.setData(x=x4,y=yawVal)
        yawPlotPlt2.setData(x=x4,y=yawSetpoint)
        # yawPlot.addLegend()
        # yawPlot.plot(x4,yawVal,clear=True,pen=(1,2),name="Yaw")
        # yawPlot.addItem(pg.PlotCurveItem(x4,yawSetpoint,clear=True,pen=(2,2),name="Yaw setPoint"))
        # yawPlot.setYRange(-32,32)

        x3 = np.arange(len(m0Val))
        motorPlotPlt.setData(x=x3,y=m0Val)
        motorPlotPlt2.setData(x=x3,y=m1Val)
        motorPlotPlt3.setData(x=x3,y=m2Val)
        motorPlotPlt4.setData(x=x3,y=m3Val)
        # motorPlot.addLegend()
        # motorPlot.plot(x3,m0Val,clear=True,pen=(1,4),name="Front")
        # motorPlot.addItem(pg.PlotCurveItem(x3,m1Val,clear=True,pen=(2,4),name="Right"))
        # motorPlot.addItem(pg.PlotCurveItem(x3,m2Val,clear=True,pen=(3,4),name="Rear"))
        # motorPlot.addItem(pg.PlotCurveItem(x3,m3Val,clear=True,pen=(4,4),name="Left"))
#        motorPlot.setYRange(1200,1800)

timer.timeout.connect(update)
timer.start(200)

def checkForReadings(data):
    y = re.findall(r"y([-+]?\d*\.\d+|\d+)>",data)
    p = re.findall(r"p([-+]?\d*\.\d+|\d+)>",data)
    r = re.findall(r"r([-+]?\d*\.\d+|\d+)>",data)
    ps = re.findall(r"ps([-+]?\d*\.\d+|\d+)>",data)
    rs = re.findall(r"rs([-+]?\d*\.\d+|\d+)>",data)
    ys = re.findall(r"ys([-+]?\d*\.\d+|\d+)>",data)
    po = re.findall(r"po([-+]?\d*\.\d+|\d+)>",data)
    ro = re.findall(r"ro([-+]?\d*\.\d+|\d+)>",data)
    yo = re.findall(r"yo([-+]?\d*\.\d+|\d+)>",data)    
    m0 = re.findall(r"m0-([-+]?\d*\.\d+|\d+)>",data)
    m1 = re.findall(r"m1-([-+]?\d*\.\d+|\d+)>",data)
    m2 = re.findall(r"m2-([-+]?\d*\.\d+|\d+)>",data)
    m3 = re.findall(r"m3-([-+]?\d*\.\d+|\d+)>",data)
    rR = re.findall(r"c0([-+]?\d*\.\d+|\d+)>",data)
    rP = re.findall(r"c1([-+]?\d*\.\d+|\d+)>",data)
    rT = re.findall(r"c2([-+]?\d*\.\d+|\d+)>",data)
    rY = re.findall(r"c3([-+]?\d*\.\d+|\d+)>",data)

    ppVal = re.findall(r"@pp([-+]?\d*\.\d+|\d+)>",data)
    piVal = re.findall(r"@pi([-+]?\d*\.\d+|\d+)>",data)
    pdVal = re.findall(r"@pd([-+]?\d*\.\d+|\d+)>",data)
    
    rpVal = re.findall(r"@rp([-+]?\d*\.\d+|\d+)>",data)
    riVal = re.findall(r"@ri([-+]?\d*\.\d+|\d+)>",data)
    rdVal = re.findall(r"@rd([-+]?\d*\.\d+|\d+)>",data)
    
    ypVal = re.findall(r"@yp([-+]?\d*\.\d+|\d+)>",data)
    yiVal = re.findall(r"@yi([-+]?\d*\.\d+|\d+)>",data)
    ydVal = re.findall(r"@yd([-+]?\d*\.\d+|\d+)>",data)

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
    
    if len(y) > 0 and len(ys) > 0:
        yawInstLable.setText('Yaw --> '+y[0])
        #yawSetPointInstLable.setText('Yaw SetPoint --> '+ys[0])
        yawVal.append(float(y[0]))
        yawSetpoint.append(float(ys[0]))
        while len(yawVal) > 100 :
            yawVal.pop(0)
            yawSetpoint.pop(0)

    if len(m0) > 0  and len(m1) > 0 and len(m2) > 0 and len(m3) > 0:
        frontMotorInstLable.setText('Right Front --> '+m0[0])
        rightMotorInstLable.setText('Right Rear --> '+m1[0])
        rearMotorInstLable.setText('Left Rear  --> '+m2[0])
        leftMotorInstLable.setText('Left Front  --> '+m3[0])
        m0Val.append(float(m0[0]))
        m1Val.append(float(m1[0]))
        m2Val.append(float(m2[0]))
        m3Val.append(float(m3[0]))     
        while len(m0Val) > 100 :
            m0Val.pop(0)
            m1Val.pop(0)
            m2Val.pop(0)
            m3Val.pop(0) 

    if len(rR) > 0 :
        ch1InstLable.setText('CH1 --> '+rR[0])
    if len(rP) > 0 :
        ch2InstLable.setText('CH2 --> '+rP[0])
    if len(rT) > 0 :
        ch3InstLable.setText('CH3 --> '+rT[0])
    if len(rY) > 0 :
        ch4InstLable.setText('CH4 --> '+rY[0])
    if len(po) > 0 :
        pitchPIDOutInstLable.setText('Pitch PID Out --> '+po[0])
    if len(ro) > 0 :
        rollPIDOutInstLable.setText('Roll PID Out --> '+ro[0])
    if len(yo) > 0:
        yawPIDOutInstLable.setText('Yaw PID Out --> '+yo[0])

    if len(ppVal) > 0:
        print "pp-" + str(ppVal[0]) + " ", # comma is used to not to print new line
        pp.setValue(float(ppVal[0]))
    if len(piVal) > 0:
        print "pi-" + str(piVal[0]) + " ", # comma is used to not to print new line
        pi.setValue(float(piVal[0]))
    if len(pdVal) > 0:
        print "pd-" + str(pdVal[0]) + " ", # comma is used to not to print new line
        pd.setValue(float(pdVal[0]))
    if len(rpVal) > 0:
        print "rp-" + str(rpVal[0]) + " ", # comma is used to not to print new line
        rp.setValue(float(rpVal[0]))
    if len(riVal) > 0:
        print "ri-" + str(riVal[0]) + " ", # comma is used to not to print new line
        ri.setValue(float(riVal[0]))
    if len(rdVal) > 0:
        print "rd-" + str(rdVal[0]) + " ", # comma is used to not to print new line
        rd.setValue(float(rdVal[0]))
    if len(ypVal) > 0:
        print "yp-" + str(ypVal[0]) + " ", # comma is used to not to print new line
        yp.setValue(float(ypVal[0]))
    if len(yiVal) > 0:
        print "ri-" + str(yiVal[0]) + " ", # comma is used to not to print new line
        yi.setValue(float(yiVal[0]))
    if len(ydVal) > 0:
        print "yd-" + str(ydVal[0]) + " ", # comma is used to not to print new line
        yd.setValue(float(ydVal[0]))                 
    # print [y,p,r,ps,rs,po,ro,m0,m1,m2,m3]
    # fHandler.write('p,r,po,ro,ps,rs,m0,m1,m2,m3 \n')
    if len(p) > 0 and len(r) > 0 and len(ps) > 0 and len(rs) > 0 and len(po) > 0 and len(ro) > 0 and len(m0) > 0 and len(m1) > 0 and len(m2) > 0 and len(m3) > 0 :
        fHandler.write(str(p[0])+','+str(r[0])+','+str(ps[0])+','+str(rs[0])+','+str(po[0])+','+str(ro[0])+','+str(m0[0])+','+str(m1[0])+','+str(m2[0])+','+str(m3[0])+'\n')

def checkData():
    # print "check data"
    recvString = '' # to store the data
    while (arduinoData.inWaiting() > 0):
        # print arduinoData.read()
        recvString = recvString + arduinoData.read()
        #recvText.setText(recvString)
    rawDataString.setText(recvString) 
    checkForReadings(recvString)
    app.processEvents()


#start qt event loop
app.processEvents()
app.exec_()

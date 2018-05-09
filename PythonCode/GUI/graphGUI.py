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

arduinoData = serial.Serial('/dev/ttyACM0', 115200) #Creating our serial object named arduinoData

noOfDataSamples = 200

#Create a Empty Array to hold the graph parameters
pitchSetPoint = np.empty(noOfDataSamples)
rollSetPoint = np.empty(noOfDataSamples)

pitchVal = np.empty(noOfDataSamples)
rollVal = np.empty(noOfDataSamples)

recvString = ""


#Always start by initializing Qt
app = QtGui.QApplication([])

#Define a Top level widget to hold everything
w = QtGui.QWidget()

#Create some widgets to place inside
btn = QtGui.QPushButton('Press me')
text = QtGui.QLineEdit('enter text')
listw = QtGui.QListWidget()
plot = pg.PlotWidget()

##Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)


# add widgets to layout in their proper postion
layout.addWidget(btn,0,0)
layout.addWidget(text,1,0)
layout.addWidget(listw,2,0)
layout.addWidget(plot,0,1,3,1)

#Display widget as a new window
w.show()


#x = np.random.normal(1000)
#y = np.random.normal(1000)

def checkData():
    	noOfBytes = arduinoData.inWaiting()
    	if(noOfBytes > 0) :
    			recvString = arduinoData.readline();


timer = pg.QtCore.QTimer()

def update():
    	checkData()
        plot.plot(x,y,clear=True)

timer.timeout.connect(update)
timer.start(16)


#start qt event loop
app.exec_()

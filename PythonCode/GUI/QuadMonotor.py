from PyQt4 import QtCore, QtGui
from PyQt4.Qt import *
import pyqtgraph as pg
import numpy as np
import serial
import re
import sys

class MyApp(QtGui.QWidget) :
    def __init__(self):
        super(MyApp,self).__init__()
        self.initUI()

    def initUI(self):
        g = Graphs()
        self.setGeometry(0,0,1800,1000)
        self.setWindowTitle("KSRMQuad Monitor By Kunchala Anil")
        layout =  QtGui.QGridLayout()
        layout.addWidget(g.pitchPlot,0,0)
        self.setLayout(layout)

        self.show()

class Graphs() :
    def __init__(self):
        self.pitchPlot = pg.PlotWidget()
        self.rollPlot = pg.PlotWidget()

class Motors() :

def main():
    app = QtGui.QApplication(sys.argv)
    gui = MyApp()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
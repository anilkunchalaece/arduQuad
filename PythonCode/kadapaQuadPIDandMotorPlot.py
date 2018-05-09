'''
Author : Kunchala Anil
Date : 20 Mar 2018
plot the IMU data and Motor Power 
values will be received via Arduino Serial Communication
'''
import serial
from matplotlib import pyplot as plt

yaw = []
pitch = []
roll = []

frontLeftMotor = []
frontRightMotor = []
backLeftMotor = []
backRightMotor = []

ref = [0]

arduinoData = serial.Serial('/dev/ttyACM0', 115200) #Creating our serial object named arduinoData
plt.ion() #Tell matplotlib you want interactive mode to plot live data
cnt=0

def makeFig():
    plt.subplot(5,1,1)
    plt.plot(ref,yaw,label="yaw",linestyle="steps",linewidth="2")
    plt.plot(ref,pitch,label="pitch",linestyle="-",linewidth="3")
    plt.plot(ref,roll,label="roll",linestyle=":",linewidth="3")
    plt.title("Yaw Pitch Roll")
    plt.ylabel("YPR")
    plt.xlabel("noOfSamples")

    plt.subplot(5,1,2)
    plt.bar(ref,frontLeftMotor)
    plt.ylabel("frontLeftMotor")
    plt.subplot(5,1,3)
    plt.bar(ref,frontRightMotor)
    plt.ylabel("frontRightMotor")
    plt.subplot(5,1,4)
    plt.bar(ref,backLeftMotor)
    plt.ylabel("backLeftMotor")
    plt.subplot(5,1,5)
    plt.bar(ref,backRightMotor)
    plt.ylabel("backRightMotor")
    plt.show()

while True: # While loop that loops forever
    while (arduinoData.inWaiting()==0): #Wait here until there is data
        print "waiting"
        pass #do nothing
    arduinoString = arduinoData.readline() #read the line of text from the serial port
    print arduinoString
    if arduinoString.find('y') and arduinoString.find('@') !=- 0:
        print "yaw"+arduinoString[:arduinoString.find('y')]
        yawF = float(arduinoString[:arduinoString.find('y')])
        yaw.append(yawF)
    elif  arduinoString.find('p') and arduinoString.find('@') !=- 0:
        pitchF = float(arduinoString[:arduinoString.find('p')])
        pitch.append(pitchF)
    elif arduinoString.find('r') and arduinoString.find('@') !=- 0:
        rollF = float(arduinoString[:arduinoString.find('r')])
        roll.append(rollF)
    elif arduinoString.find('fl') and arduinoString.find('@') !=- 0:
        frontLeftMotorF = float(arduinoString[:arduinoString.find('fl')])
        frontLeftMotor.append(frontLeftMotorF)
    elif arduinoString.find('fr') and arduinoString.find('@') !=- 0:
        frontRightMotorF = float(arduinoString[:arduinoString.find('fr')])
        frontRightMotor.append(frontRightMotorF)
    elif arduinoString.find('bl') and arduinoString.find('@') !=- 0:
        backLeftMotorF = float(arduinoString[:arduinoString.find('bl')])
        backLeftMotor.append(backLeftMotorF)
    elif arduinoString.find('br') and arduinoString.find('@') !=- 0:
        backRightMotorF = float(arduinoString[:arduinoString.find('br')])
        backRightMotor.append(backRightMotorF)
    try :
        makeFig()                            #Call drawnow to update our live graph
    except :
        print "unable to plot"
    plt.pause(.000001)
    ref.append(ref[-1]+1)                     #Pause Briefly. Important to keep drawnow from crashing
    print cnt
    # if(cnt>20):
    #     print "removing"                            #If you have 50 or more points, delete the first one from the array
    #     yaw.pop(0)                       #This allows us to just see the last 50 data points
    #     pitch.pop(0)
    #     roll.pop(0)
    #     frontRightMotor.pop(0)
    #     frontLeftMotor.pop(0)
    #     backLeftMotor.pop(0)
    #     backRightMotor.pop(0)
    #     ref.pop(0)
    

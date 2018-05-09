from matplotlib import pyplot as plt
yaw = [10,20,30] 
pitch = [0,10,10]
roll = [0,10,-10]

frontLeftMotor = [1200,1400,1300]
frontRightMotor = [1000,1300,1100]
backLeftMotor = [1400,1000,1200]
backRightMotor = [1200,1400,1000]

ref = [1,2,3]

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
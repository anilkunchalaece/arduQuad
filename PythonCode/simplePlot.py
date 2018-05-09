from matplotlib import pyplot as plt
x1 = [1,2,3]
y1 = [4,5,1]

x2 = [1,2,3]
y2 = [4,2,5]

plt.plot(x1,y1,'g',label="line1");
plt.plot(x2,y2,'c',label="line2");

plt.bar(x1,y1,color="y")
plt.title("simple graph")
plt.ylabel("Y Axis")
plt.xlabel("x Axis")
plt.legend()
plt.show()
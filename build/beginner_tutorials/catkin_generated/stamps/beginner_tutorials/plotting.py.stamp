#%%
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import math
import numpy
import matplotlib.pyplot as plt



class plotting():
    def __init__(self):
        self.angle = []
        self.posx = []
        self.posy = []

        rospy.Subscriber("Car_Telemetries_pos", Float64MultiArray, self.telemetryPos)
        self.inputPos = Float64MultiArray()
        rospy.Subscriber("Car_Telemetries_angle", Float64MultiArray, self.telemetryAngle)
        self.inputAngle = Float64MultiArray()

    def telemetryPos(self, data):
        self.inputPos = data

        self.posx.append(float(data.data[0]))
        self.posy.append(float(data.data[1]))

    def telemetryAngle(self,data):
        self.inputAngle = data
        self.angle.append(float(data.data[0]))

    def plot(self, event = None):
        plt.ion()
        plt.axis([-13, 13, -13, 13])
        plt.plot(self.posx, self.posy, 'b')
        plt.pause(0.05)

if __name__ == '__main__':
    try:
        rospy.init_node('Plotting', anonymous=True)
        plot = plotting()
        rospy.Timer(rospy.Duration(0.1), plot.plot)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# %%

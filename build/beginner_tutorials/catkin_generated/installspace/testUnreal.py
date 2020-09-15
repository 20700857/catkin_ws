import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import airsim
import numpy as np
import os
import time
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
class Unreal():

    def __init__(self):
        self.subscriber = rospy.Subscriber('/Unreal', Twist, self.input_Callback)
        # connect to the AirSim simulator 
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

    def input_Callback(self,data):
        if data.linear.x < 0 :
            self.car_controls.brake = 1
        else:
            self.car_controls.brake = 0
            self.car_controls.throttle = data.linear.x
        self.car_controls.steering = -data.angular.z
        self.client.setCarControls(self.car_controls)

if __name__ == '__main__':
    try:
        rospy.init_node('Control', anonymous=True)
        unreal = Unreal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
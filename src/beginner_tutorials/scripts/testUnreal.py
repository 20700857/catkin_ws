import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import airsim
import cv2
import numpy as np
import os
import time

class Unreal():

    def __init__(self):
        self.subscriber = rospy.Subscriber('/Unreal', Twist, self.input_Callback)
        # connect to the AirSim simulator 
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
    
    def printPos(self, event = None):

        car_state = self.client.getCarState()
        print(car_state.kinematics_estimated.position)

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
        rospy.Timer(rospy.Duration(1), unreal.printPos)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
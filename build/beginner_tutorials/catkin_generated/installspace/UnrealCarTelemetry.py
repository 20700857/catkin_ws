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
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.publisher = rospy.Publisher('/Car', Marker, queue_size=10)
    def publishCar(self,event =None):
        
        self.car_state = self.client.simGetVehiclePose()

        colourSet = ColorRGBA()
        colourSet.a = 1.0
        colourSet.r = 1.0
        colourSet.g = 0.0
        colourSet.b = 0.0

        addingMarker = Marker()
        addingMarker.id = 0
        addingMarker.header.frame_id = 'map'
        addingMarker.pose.position.x = self.car_state.position.x_val - 20
        addingMarker.pose.position.y = -self.car_state.position.y_val
        addingMarker.pose.position.z = self.car_state.position.z_val
        addingMarker.pose.orientation.x = self.car_state.orientation.x_val
        addingMarker.pose.orientation.y = self.car_state.orientation.y_val
        addingMarker.pose.orientation.z = self.car_state.orientation.z_val
        addingMarker.pose.orientation.w = -self.car_state.orientation.w_val
        addingMarker.scale.x = 10
        addingMarker.scale.y = 10
        addingMarker.scale.z = 5
        addingMarker.action = addingMarker.ADD
        addingMarker.type = addingMarker.CUBE
        addingMarker.color = colourSet

        self.publisher.publish(addingMarker)

if __name__ == '__main__':
    try:
        rospy.init_node('Control', anonymous=True)
        unreal = Unreal()
        rospy.Timer(rospy.Duration(0.05), unreal.publishCar)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
import math
import tf
from tf import TransformBroadcaster
from random import random

class TestingMap():

    def __init__(self):
        self.objects = MarkerArray()
        self.unrealObjects = rospy.Publisher('/Obstacles', MarkerArray, queue_size=10)

        quaternionSet = Quaternion()
        quaternionSet = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        colourSet = ColorRGBA()
        colourSet.a = 1.0
        colourSet.r = 0.0
        colourSet.g = 0.5
        colourSet.b = 0.5

        self.objectPostions = []
        self.createMap()
        count = 0

        for x in self.objectPostions:
            addingMarker = Marker()
            addingMarker.id = count
            addingMarker.header.frame_id = 'map'
            addingMarker.pose.position = x
            addingMarker.pose.orientation.x = quaternionSet[0]
            addingMarker.pose.orientation.y = quaternionSet[1]
            addingMarker.pose.orientation.z = quaternionSet[2]
            addingMarker.pose.orientation.w = quaternionSet[3]
            addingMarker.scale.x = 10
            addingMarker.scale.y = 10
            addingMarker.scale.z = 10
            addingMarker.action = addingMarker.ADD
            addingMarker.type = addingMarker.SPHERE
            addingMarker.color = colourSet
            self.objects.markers.append(addingMarker)
            count += 1

    def createMap(self):
        runs = 0
        while runs < 100:
            x = (random()-0.5)*600
            y = (random()-0.5)*600
            temp = Vector3(x,y,0.0)
            self.objectPostions.append(temp)
            x += 1



    def outputMap(self, event=None):

        self.unrealObjects.publish(self.objects)


if __name__ == '__main__':
    try:
        rospy.init_node('Basic_World', anonymous=True)
        output = TestingMap()
        

        rospy.Timer(rospy.Duration(0.01), output.outputMap)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


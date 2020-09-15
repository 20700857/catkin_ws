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
import random
import math
import tf
from tf import TransformBroadcaster

class UnrealWorld():

    def __init__(self):
        self.objects = MarkerArray()
        self.unrealObjects = rospy.Publisher('/UnrealObjects', MarkerArray, queue_size=10)

        quaternionSet = Quaternion()
        quaternionSet = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        colourSet = ColorRGBA()
        colourSet.a = 1.0
        colourSet.r = 0.0
        colourSet.g = 0.5
        colourSet.b = 0.5

        objectPostions = []
        #inside
        for x in range(0,2):
            for y in range(33):
                temp = Vector3(90*x,-102 + 6*y,0)
                objectPostions.append(temp)
        for y in range(0,2):
            for x in range(0,16):
                temp = Vector3(6*x,-102 + 198*y,0)
                objectPostions.append(temp)

        #outside
        for x in range(0,2):
            for y in range(48):
                temp = Vector3(-42 + 180*x,-144 + 6*y,0)
                objectPostions.append(temp)
        for y in range(0,2):
            for x in range(0,29):
                temp = Vector3(-36 + 6*x,-144 + 282*y,0)
                objectPostions.append(temp)

        count = 0

        for x in objectPostions:
            addingMarker = Marker()
            addingMarker.id = count
            addingMarker.header.frame_id = 'map'
            addingMarker.pose.position = x
            addingMarker.pose.orientation.x = quaternionSet[0]
            addingMarker.pose.orientation.y = quaternionSet[1]
            addingMarker.pose.orientation.z = quaternionSet[2]
            addingMarker.pose.orientation.w = quaternionSet[3]
            addingMarker.scale.x = 5
            addingMarker.scale.y = 5
            addingMarker.scale.z = 10
            addingMarker.action = addingMarker.ADD
            addingMarker.type = addingMarker.SPHERE
            addingMarker.color = colourSet
            self.objects.markers.append(addingMarker)
            count += 1


    def outputMap(self, event=None):

        self.unrealObjects.publish(self.objects)


if __name__ == '__main__':
    try:
        rospy.init_node('UnrealWorld', anonymous=True)
        output = UnrealWorld()

        rospy.Timer(rospy.Duration(1), output.outputMap)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


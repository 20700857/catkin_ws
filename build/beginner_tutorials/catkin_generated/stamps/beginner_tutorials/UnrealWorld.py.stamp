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
        #group 1
        temp = Vector3(25,-27.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(35,-27.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(25,-37.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(35,-37.5,2.5)
        objectPostions.append(temp)

        #group 2
        temp = Vector3(-15,-57.5,2.5)
        objectPostions.append(temp)

        #group 3
        temp = Vector3(-35,-107.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(-35,-117.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(-25,-107.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(-25,-117.5,2.5)
        objectPostions.append(temp)

        #group 4
        temp = Vector3(115,-107.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(105,-107.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(105,-117.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(115,-117.5,2.5)
        objectPostions.append(temp)

        #group 5
        temp = Vector3(75,-57.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(75,-67.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(85,-57.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(85,-67.5,2.5)
        objectPostions.append(temp)

        #group 6
        temp = Vector3(25,22.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(25,32.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(35,22.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(35,32.5,2.5)
        objectPostions.append(temp)

        #group 7
        temp = Vector3(75,52.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(75,62.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(85,52.5,2.5)
        objectPostions.append(temp)
        temp = Vector3(85,62.5,2.5)
        objectPostions.append(temp)

        #group 8
        for x in range(3):
            for y in range(3):
                temp = Vector3(115- x*10,-17.5 + y*10,2.5)
                objectPostions.append(temp)
        
        #group 9
        temp = Vector3(-15,52.5,2.5)
        objectPostions.append(temp)

        #group 10
        for x in range(1):
            for y in range(1):
                temp = Vector3(-25- x*10,102.5 + y*10,2.5)
                objectPostions.append(temp)
        #group 11
        for x in range(1):
            for y in range(1):
                temp = Vector3(115- x*10,102.5 + y*10,2.5)
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
            addingMarker.scale.x = 10
            addingMarker.scale.y = 10
            addingMarker.scale.z = 5
            addingMarker.action = addingMarker.ADD
            addingMarker.type = addingMarker.CUBE
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
            


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

class BasicWorld():

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
        for z1 in range(0,2):
            for z2 in range(90 +90*z1):
                x = math.sin(math.radians(z2*(4-2*z1)))*(150.0+150.0*z1)
                y = math.cos(math.radians(z2*(4-2*z1)))*(150.0+150.0*z1)
                temp = Vector3(x,y,0.0)
                self.objectPostions.append(temp)


    def createSide(self, X1,X2,Y1,Y2, step):
        for x in range(X1,X2 + step,step):
            for y in range(Y1,Y2 + step,step):
                temp = Vector3(x,y,0)
                self.objectPostions.append(temp)

    def outputMap(self, event=None):

        self.unrealObjects.publish(self.objects)


if __name__ == '__main__':
    try:
        rospy.init_node('Basic_World', anonymous=True)
        output = BasicWorld()
        

        rospy.Timer(rospy.Duration(0.01), output.outputMap)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


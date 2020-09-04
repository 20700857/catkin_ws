import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import geometry_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import random
import math
import tf
from tf import TransformBroadcaster
import tf2_ros
import random
class mapNode():
    def __init__(self, position):
        self.location = position

class fullMap():

    def __init__(self,start):
        self.explored = []
        self.explored.append(mapNode(start))
        self.graphics = MarkerArray()

    def addNode(self, event = None):
        temp = self.explored[len(self.explored)]
        out = Point()
        out.x = temp.x + random.random()
        out.y = temp.y + random.random()
        out.z = temp.z
        self.explored.append(mapNode(out))
    
    
    def updateGraphics(self, event = None):
        temp = MarkerArray()
        for x in range(len(self.explored)):
            tempMarker = Marker()
            tempMarker.id = x
            
            tempMarker.type = tempMarker.CUBE
            tempMarker.action = tempMarker.ADD
            tempMarker.scale.x = 0.2
            tempMarker.scale.y = 0.2
            tempMarker.scale.z = 0.2
            tempMarker.pose.position = self.explored[x].location
            tempMarker.color.a = 1.0
            tempMarker.color.r = 0.0
            tempMarker.color.g = 0.0
            tempMarker.color.b = 0.0
            tempOri = tf.transformations.quaternion_from_euler(0.0, 0.0, 0)
            tempMarker.pose.orientation.x = tempOri[0]
            tempMarker.pose.orientation.y = tempOri[1]
            tempMarker.pose.orientation.z = tempOri[2]
            tempMarker.pose.orientation.w = tempOri[3]
            temp.markers.append()
        

if __name__ == '__main__':
    try:
        rospy.init_node('Map', anonymous=True)
        start = Vector3()
        start.x = 0
        start.y = 0
        start.z = 0
        rrtMap = fullMap(start)
        rospy.Timer(rospy.Duration(1), rrtMap.addNode)
        rospy.Timer(rospy.Duration(1), rrtMap.updateGraphics)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import geometry_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import random
import math
import tf
from tf import TransformBroadcaster
import tf2_ros
class mapNode():
    def __init__(self, position, indexX, indexY):
        self.position = position
        self.stateViable = True
        self.stateObstacle = False
        self.stateChecked = False
        self.stateStart = False
        self.stateEnd = False
        self.fScore = 9999999.99
        self.parent = None
        self.indexX = indexX
        self.indexY = indexY
        self.statePath = False
        self.open = False
        self.movementSteps = 0.0

        self.score = self.fScore = self.movementSteps

class fullMap():

    def __init__(self):
        self.map = []
        self.map.append([])
        self.size = 0
        self.openList = []
        self.finish = Vector3()
        self.closedList = []
        self.pathPublisher = rospy.Publisher('/Path', Path, queue_size=10)
        self.pathPointPublisher = rospy.Publisher('/Path_Points', MarkerArray, queue_size=10)
        self.publisher = rospy.Publisher('/MapPath', MarkerArray, queue_size=10)

        self.path = Path()
        self.pathMarkers = MarkerArray()

        self.pathfound = False

        self.quaternionSet = Quaternion()
        self.quaternionSet = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        self.worldObstacles = MarkerArray()

        rospy.Subscriber("/world", MarkerArray, self.getWorldObstacles)

        self.current = mapNode(Vector3(0,0,0),0,0)

        self.objectsReceived = False

if __name__ == '__main__':
    try:
        rospy.init_node('Map', anonymous=True)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


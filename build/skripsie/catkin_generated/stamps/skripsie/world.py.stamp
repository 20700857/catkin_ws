import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import random
import math
import tf
from tf import TransformBroadcaster

class worldNode():
    def __init__(self, position):
        self.position = position
        self.state = 0

class world():
    def __init__(self, size):
        self.worldPublisher = rospy.Publisher('/world', MarkerArray, queue_size=10)
        self.size = size
        self.worldState = []
        self.createWorld()
        self.markers = MarkerArray()


    def createWorld(self):
        for x in range(self.size):
            self.worldState.append([])
            for y in range(self.size):
                temp = Vector3()
                temp.z = 0
                temp.x = x - self.size/2
                temp.y = y - self.size/2
                tempNode = worldNode(temp)
                self.worldState[x].append(tempNode)
    
    def createTrack(self, inner, outer):
        
        angle = 0
        while angle < math.pi*2:
            tempInner = Vector3()
            tempInner.x = int(round(math.cos(angle)*inner))
            tempInner.y = int(round(math.sin(angle)*inner))
            self.worldState[tempInner.x][tempInner.y].state = 1

            tempOuter = Vector3()
            tempOuter.x = int(round(math.cos(angle)*outer))
            tempOuter.y = int(round(math.sin(angle)*outer))
            self.worldState[tempOuter.x][tempOuter.y].state = 1

    def publishTrack(self):

        br = TransformBroadcaster()

        obstacleCount = 0

        for x in range(self.size):
            for y in range(self.size):
                temp = self.worldState[x][y]
                if temp.state == 1:
                    br.sendTransform((temp.position.x , temp.position.y, temp.position.z),tf.transformations.quaternion_from_euler(0.0, 0.0, 0),rospy.Time.now(),"Obstacle: " + str(obstacleCount),"map")
                    obstacleCount += 1
                    
                    mapMarker = Marker()
                    mapMarker.id = obstacleCount
                    mapMarker.header.frame_id = "Obstacle: " + str(obstacleCount)
                    mapMarker.type = mapMarker.SPHERE
                    mapMarker.action = mapMarker.ADD
                    mapMarker.scale.x = 1
                    mapMarker.scale.y = 1
                    mapMarker.scale.z = 1
                    mapMarker.pose.position.z = 0
                    mapMarker.pose.position.x = 0
                    mapMarker.pose.position.y = 0
                    mapMarker.color.a = 1.0
                    mapMarker.color.r = 0.0
                    mapMarker.color.g = 0.0
                    mapMarker.color.b = 1.0

                    self.markers.markers.append(mapMarker)
                    pass

        self.worldPublisher



if __name__ == '__main__':
    try:
        rospy.init_node('Map', anonymous=True)
        worldState = world(50)
        worldState.createTrack(10,20)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


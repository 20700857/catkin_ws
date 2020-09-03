import rospy
import geometry_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import random
import math
import tf
import tf2_ros
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
        self.markers = MarkerArray()
        self.br = tf2_ros.TransformBroadcaster()
        self.transforms = []
        self.createWorld()

        self.staticBr = tf2_ros.StaticTransformBroadcaster()


    def createWorld(self):

        for x in range(self.size):
            self.worldState.append([])
            for y in range(self.size):
                temp = geometry_msgs.msg.Vector3()
                temp.z = 0
                temp.x = x - self.size/2
                temp.y = y - self.size/2
                tempNode = worldNode(temp)
                self.worldState[x].append(tempNode)
    
    def createTrack(self, inner, outer):

        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "Obstacles"

        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(0,0,0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.staticBr.sendTransform(static_transformStamped)
        
        angle = 0
        while angle < math.pi*2:
            tempInner = geometry_msgs.msg.Vector3()
            tempInner.x = int(round(math.cos(angle)*inner - self.size/2))
            tempInner.y = int(round(math.sin(angle)*inner - self.size/2))
            self.worldState[tempInner.x][tempInner.y].state = 1

            tempOuter = geometry_msgs.msg.Vector3()
            tempOuter.x = int(round(math.cos(angle)*outer - self.size/2))
            tempOuter.y = int(round(math.sin(angle)*outer - self.size/2))
            self.worldState[tempOuter.x][tempOuter.y].state = 1
            angle += math.pi/2048
        

        obstacleCount = 0

        self.markers.markers.clear()

        for x in range(self.size):
            for y in range(self.size):
                temp = self.worldState[x][y]
                if temp.state == 1:

                    mapMarker = Marker()
                    mapMarker.id = obstacleCount
                    mapMarker.header.frame_id = "Obstacles"
                    mapMarker.type = mapMarker.SPHERE
                    mapMarker.action = mapMarker.ADD
                    mapMarker.scale.x = 1
                    mapMarker.scale.y = 1
                    mapMarker.scale.z = 1
                    mapMarker.pose.position.z = 0
                    mapMarker.pose.position.x = temp.position.x
                    mapMarker.pose.position.y = temp.position.y
                    tempOri = tf.transformations.quaternion_from_euler(0.0, 0.0, 0)
                    mapMarker.pose.orientation.x = tempOri[0]
                    mapMarker.pose.orientation.y = tempOri[1]
                    mapMarker.pose.orientation.z = tempOri[2]
                    mapMarker.pose.orientation.w = tempOri[3]
                    mapMarker.color.a = 1.0
                    mapMarker.color.r = 0.0
                    mapMarker.color.g = 0.0
                    mapMarker.color.b = 1.0
                    obstacleCount += 1

                    self.markers.markers.append(mapMarker)


    def publishTrack(self, event = None):

        self.createTrack(10,20)

        self.worldPublisher.publish(self.markers)


if __name__ == '__main__':
    try:
        rospy.init_node('Map', anonymous=True)
        worldState = world(50)

        rospy.Timer(rospy.Duration(0.01), worldState.publishTrack)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


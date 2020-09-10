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
from scipy.stats import norm
import numpy as np
class mapNode():
    def __init__(self, position, parent):
        self.location = position
        self.parent = parent

class fullMap():

    def __init__(self,start):
        self.goal = Point(4,4,0)
        self.explored = []
        self.explored.append(mapNode(start, None))
        self.RRTPublisher = rospy.Publisher('/RRT', MarkerArray, queue_size=10)
        self.RRTLinesPublisher = rospy.Publisher('/RRTLine', MarkerArray, queue_size=10)
        self.RRTPath = rospy.Publisher('/RRTPath', MarkerArray, queue_size=10) 
        self.randPoint = rospy.Publisher('/randPoint', MarkerArray, queue_size=10) 
        self.delta = 0.2
        self.pathFound = False

    def addNode(self, event = None):
        if not self.pathFound:
            out = Point()
            mean = [self.goal.x, self.goal.y]
            cov = [[2, 0], [0, 2]]  # diagonal covariance
            out.x , out.y = np.random.multivariate_normal(mean, cov, 1).T
            out.z = 0
            nearest = self.nearestNode(out)
            angle = self.getAngle(nearest.location, out)
            nodeAdded = Point()
            nodeAdded.x = nearest.location.x + math.cos(angle)*self.delta
            nodeAdded.y = nearest.location.y + math.sin(angle)*self.delta
            nodeAdded.z = 0

            if True:
                outPoint = MarkerArray()

                newPoint = Marker()
                newPoint.id = 1
                newPoint.header.frame_id = 'map'
                newPoint.type = newPoint.SPHERE
                newPoint.action = newPoint.ADD
                newPoint.scale.x = 0.2
                newPoint.scale.y = 0.2
                newPoint.scale.z = 0.2
                newPoint.pose.position = out
                newPoint.color.a = 1.0
                newPoint.color.r = 0.5
                newPoint.color.g = 0.5
                newPoint.color.b = 0.5
                tempOri = tf.transformations.quaternion_from_euler(0.0, 0.0, 0)
                newPoint.pose.orientation.x = tempOri[0]
                newPoint.pose.orientation.y = tempOri[1]
                newPoint.pose.orientation.z = tempOri[2]
                newPoint.pose.orientation.w = tempOri[3]
                newPoint.lifetime = rospy.Time(1)
                outPoint.markers.append(newPoint)

                marker3 = Marker()
                marker3.id = 1
                marker3.header.frame_id = 'map'
                marker3.type = Marker.LINE_STRIP
                marker3.ns = 'Testline'
                marker3.action = 0
                marker3.scale.x = 0.05
                marker3.points.append(out)
                marker3.points.append(nearest.location)
                marker3.color.a = 1.0
                marker3.color.r = 0.0
                marker3.color.g = 0.0
                marker3.color.b = 1.0
                marker3.lifetime = rospy.Time(1)
                outPoint.markers.append(marker3)
                self.randPoint.publish(outPoint)

            self.explored.append(mapNode(nodeAdded, nearest))
            if self.getDistToGoal(nodeAdded) < 0.1:
                self.pathFound = True
        else:
            temp = MarkerArray()
            idNum = 0
            current = self.explored[len(self.explored)-1]
            condition = True
            while condition:
                if not current.parent == None:
                    marker3 = Marker()
                    marker3.id = idNum
                    marker3.header.frame_id = 'map'
                    marker3.type = Marker.LINE_STRIP
                    marker3.ns = 'Testline'
                    marker3.action = 0
                    marker3.scale.x = 0.05
                    marker3.points.append(current.location)
                    marker3.points.append(current.parent.location)
                    marker3.color.a = 1.0
                    marker3.color.r = 1.0
                    marker3.color.g = 1.0
                    marker3.color.b = 0.0
                    marker3.lifetime = rospy.Time(1)
                    idNum += 1
                    temp.markers.append(marker3)
                    current = current.parent
                else:
                    condition = False
            
            self.RRTPath.publish(temp)

    def getDistToGoal(self,point):
        return math.sqrt((point.x - self.goal.x)**2 + (point.y - self.goal.y)**2)

    def nearestNode(self, point):
        temp = None
        tempDistNearest = 999999
        for x in self.explored:
            tempDist = self.getDist(point,x.location)
            if tempDist < tempDistNearest:
                temp = x
                tempDistNearest = tempDist
        return temp
    
    def getAngle(self, point1, point2):
        return math.atan2(point2.y - point1.y,point2.x - point1.x)

    def getDist(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    
    def collisionCheck(self):

        return None

    def updateGraphics(self, event = None):
        temp = MarkerArray()
        idNum = 1
        for x in self.explored:
            tempMarker = Marker()
            tempMarker.id = idNum

            tempMarker.header.frame_id = "map"
            
            tempMarker.type = tempMarker.SPHERE
            tempMarker.action = tempMarker.ADD
            tempMarker.scale.x = 0.1
            tempMarker.scale.y = 0.1
            tempMarker.scale.z = 0.1
            tempMarker.pose.position = x.location
            tempMarker.color.a = 1.0
            tempMarker.color.r = 1.0
            tempMarker.color.g = 0.0
            tempMarker.color.b = 0.0
            tempOri = tf.transformations.quaternion_from_euler(0.0, 0.0, 0)
            tempMarker.pose.orientation.x = tempOri[0]
            tempMarker.pose.orientation.y = tempOri[1]
            tempMarker.pose.orientation.z = tempOri[2]
            tempMarker.pose.orientation.w = tempOri[3]
            tempMarker.lifetime = rospy.Time(1)
            temp.markers.append(tempMarker)
            idNum += 1

        if(True):
            tempMarker = Marker()
            tempMarker.id = 0
            tempMarker.header.frame_id = "map"
            tempMarker.type = tempMarker.SPHERE
            tempMarker.action = tempMarker.ADD
            tempMarker.scale.x = 0.1
            tempMarker.scale.y = 0.1
            tempMarker.scale.z = 0.1
            tempMarker.pose.position = self.goal
            tempMarker.color.a = 1.0
            tempMarker.color.r = 0.0
            tempMarker.color.g = 1.0
            tempMarker.color.b = 0.0
            tempOri = tf.transformations.quaternion_from_euler(0.0, 0.0, 0)
            tempMarker.pose.orientation.x = tempOri[0]
            tempMarker.pose.orientation.y = tempOri[1]
            tempMarker.pose.orientation.z = tempOri[2]
            tempMarker.pose.orientation.w = tempOri[3]
            tempMarker.lifetime = rospy.Time(1)
            temp.markers.append(tempMarker)

        self.RRTPublisher.publish(temp)

        temp = MarkerArray()
        idNum = 0
        for x in self.explored:
            if not x.parent == None:
                marker3 = Marker()
                marker3.id = idNum
                marker3.header.frame_id = 'map'
                marker3.type = Marker.LINE_STRIP
                marker3.ns = 'Testline'
                marker3.action = 0
                marker3.scale.x = 0.01
                marker3.points.append(x.location)
                marker3.points.append(x.parent.location)
                marker3.color.a = 1.0
                marker3.color.r = 0.0
                marker3.color.g = 1.0
                marker3.color.b = 0.0
                marker3.lifetime = rospy.Time(1)
                idNum += 1
                temp.markers.append(marker3)
        self.RRTLinesPublisher.publish(temp)

if __name__ == '__main__':
    try:
        rospy.init_node('Map', anonymous=True)
        start = Vector3()
        start.x = -5
        start.y = -5
        start.z = 0
        rrtMap = fullMap(start)
        rospy.Timer(rospy.Duration(0.01), rrtMap.addNode)
        rospy.Timer(rospy.Duration(0.01), rrtMap.updateGraphics)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


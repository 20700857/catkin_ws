#! /usr/bin/python
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
import math
import tf
from tf import TransformBroadcaster
import tf2_ros
from random import random
import numpy as np
class mapNode():
    def __init__(self, position, parent, cost):
        self.location = position
        self.parent = parent
        self.cost = float(cost)

class fullMap():

    def __init__(self,start):
        self.goals = []
        self.goals.append(Point(60,60,0))
        self.rangeX = 500
        self.rangeY = 500

        self.currentGoal = 0
        self.explored = []
        self.explored.append(mapNode(start, None, 0))

        self.RRTPublisher = rospy.Publisher('/RRT_Searching', MarkerArray, queue_size=10)
        self.RRTLinesPublisher = rospy.Publisher('/RRTLine', MarkerArray, queue_size=10)
        self.RRTPathPoints = rospy.Publisher('/RRTPath_points', MarkerArray, queue_size=10) 
        self.RRTPath = rospy.Publisher('/RRTPath', Path, queue_size=10) 
        self.randPoint = rospy.Publisher('/randPoint', MarkerArray, queue_size=10) 

        self.delta = 5
        self.pathFound = False
        self.pathPoints = MarkerArray()
        self.path = Path()

        self.runCount = 0
        self.runLimit = 500000

        self.objects = MarkerArray()
        self.subscriber = rospy.Subscriber('/Obstacles', MarkerArray, self.updateObjects)

    def updateObjects(self, data):
        self.objects = data

    def rescore(self):
        for x in reversed(self.explored):
            temp = self.nearestNode(x.location)
            x.parent = temp
            if x.parent == None:
                x.cost = 0
            else:
                x.cost = x.parent.cost + self.getDist(x.parent.location,x.location)

    def addNode(self, event = None):
        if not self.pathFound:
            if self.runCount < self.runLimit:
                #self.rescore()
                out = Point()
                #mean = [self.goal.x, self.goal.y]
                #cov = [[3, 0], [3, 2]]  # diagonal covariance
                
                while True:
                    out.x = random()*self.rangeX - self.rangeX/2
                    out.y = random()*self.rangeY - self.rangeY/2
                    #out.x , out.y = np.random.multivariate_normal(mean, cov, 1).T
                    out.z = 0
                    nearest = self.nearestNode(out)
                    angle = self.getAngle(nearest.location, out)
                    nodeAdded = Point()
                    nodeAdded.x = nearest.location.x + math.cos(angle)*self.delta
                    nodeAdded.y = nearest.location.y + math.sin(angle)*self.delta
                    nodeAdded.z = 0
                    self.addRandomMarker(out, nearest)
                    if not self.checkCollision(nodeAdded):
                        break
                    
                    
                newNode = mapNode(nodeAdded, nearest, nearest.cost + self.delta)
                self.explored.append(newNode)
                self.checkForBetter(newNode)
                if self.getDistToGoal(nodeAdded) < 3:
                    self.pathFound = True
                    self.generatePath()

            self.runCount += 1
            

    def generatePath(self):
        idNum = 0
        current = self.explored[len(self.explored)-1]
        condition = True
        self.path.header.frame_id = 'map'
        self.path.header.stamp = rospy.Time(0)
        while condition:
            if not current.parent == None:
                tempMarker = Marker()
                tempMarker.id = idNum
                tempMarker.header.frame_id = "map"
                tempMarker.type = tempMarker.SPHERE
                tempMarker.action = tempMarker.ADD
                tempMarker.scale.x = 0.5
                tempMarker.scale.y = 0.5
                tempMarker.scale.z = 0.5
                tempMarker.pose.position.x = current.location.x
                tempMarker.pose.position.y = current.location.y
                tempMarker.pose.position.z = current.location.z
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
                self.pathPoints.markers.append(tempMarker)
                idNum += 1
                

                temp = PoseStamped()
                tempPose = Pose()
                tempPose.position.z = 0
                tempPose.position.x = current.location.x
                tempPose.position.y = current.location.y
                temp.header.frame_id = "Path: " + str(idNum)
                temp.header.stamp = rospy.Time(0)
                temp.pose = tempPose
                self.path.poses.append(temp)

                current = current.parent
            else:
                condition = False

    def checkForBetter(self, newNode):
        for node in self.explored:
            dist = self.getDist(newNode.location, node.location)
            if dist <= self.delta:
                if not node.parent == None:
                    if node.parent.cost > newNode.cost + dist:
                        node.parent = newNode
                        node.cost = newNode.cost + dist    

    def addRandomMarker(self, out, nearest):
        outPoint = MarkerArray()
        newPoint = Marker()
        newPoint.id = 1
        newPoint.header.frame_id = 'map'
        newPoint.type = newPoint.SPHERE
        newPoint.action = newPoint.ADD
        newPoint.scale.x = 1
        newPoint.scale.y = 1
        newPoint.scale.z = 1
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
        marker3.scale.x = 0.2
        marker3.points.append(out)
        marker3.points.append(nearest.location)
        marker3.color.a = 1.0
        marker3.color.r = 0.0
        marker3.color.g = 0.0
        marker3.color.b = 1.0
        marker3.lifetime = rospy.Time(1)
        outPoint.markers.append(marker3)
        self.randPoint.publish(outPoint)

    def checkCollision(self, pointTo):
        for x in self.objects.markers:
            if self.getDist(pointTo,x.pose.position) < 7:
                return True
        return False

    def getDistToGoal(self,point):
        return math.sqrt((point.x - self.goals[self.currentGoal].x)**2 + (point.y - self.goals[self.currentGoal].y)**2)

    def nearestNode(self, point):
        temp = None
        tempDistNearest = 999999
        for x in self.explored:
            if x.location.x != point.x and x.location.y != point.y:
                
                tempDist = self.getDist(point,x.location)*2 + x.cost
                if tempDist < tempDistNearest:
                    temp = x
                    tempDistNearest = tempDist
        return temp
    
    def getAngle(self, point1, point2):
        return math.atan2(point2.y - point1.y,point2.x - point1.x)

    def getDist(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def updateGraphics(self, event = None):
        if not self.pathFound:
            
            temp = MarkerArray()
            idNum = 1
            for x in self.explored:
                tempMarker = Marker()
                tempMarker.id = idNum

                tempMarker.header.frame_id = "map"
                
                tempMarker.type = tempMarker.SPHERE
                tempMarker.action = tempMarker.ADD
                tempMarker.scale.x = 0.5
                tempMarker.scale.y = 0.5
                tempMarker.scale.z = 0.5
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
                tempMarker.scale.x = 0.5
                tempMarker.scale.y = 0.5
                tempMarker.scale.z = 0.5
                tempMarker.pose.position = self.goals[self.currentGoal]
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
            
            tempMarker = Marker()
            tempMarker.id = 0
            tempMarker.header.frame_id = "map"
            tempMarker.type = tempMarker.SPHERE
            tempMarker.action = tempMarker.ADD
            tempMarker.scale.x = 1.5
            tempMarker.scale.y = 1.5
            tempMarker.scale.z = 1.5
            tempMarker.pose.position = self.goals[0]
            tempMarker.color.a = 1.0
            tempMarker.color.r = 0.5
            tempMarker.color.g = 1.0
            tempMarker.color.b = 1.0
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
                    marker3.scale.x = 0.05
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
        else:
            self.RRTPathPoints.publish(self.pathPoints)
            self.RRTPath.publish(self.path)

if __name__ == '__main__':
    try:
        rospy.init_node('Map', anonymous=True)
        start = Vector3()
        start.x = -25.0
        start.y = 0.0
        start.z = 0.0
        rrtMap = fullMap(start)
        rospy.sleep(1)
        rospy.Timer(rospy.Duration(0.001), rrtMap.addNode)
        rospy.Timer(rospy.Duration(0.1), rrtMap.updateGraphics)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            

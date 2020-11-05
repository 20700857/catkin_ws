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
import tf2_ros
from random import random
import numpy as np
import time

distanceLog = []
timeLog = []

class mapNode():
    def __init__(self, position):
        self.position = position
        self.fScore = 9999999.99
        self.parent = None
        self.movementSteps = 0

class fullMap():

    def __init__(self, goals):
        self.resetCounter = -1
        self.goals = goals
        self.reset()

        self.openList = []
        self.finish = Vector3()
        self.start = Vector3()
        self.closedList = []
        

        self.pathSearchPublisher = rospy.Publisher('/Path_Searching', MarkerArray, queue_size=10)
        self.pathSolutionPublisherMarkers = rospy.Publisher('/Path_Solution_Points', MarkerArray, queue_size=10)
        self.pathSolutionPublisher = rospy.Publisher('/Path_Solution', Path, queue_size=10)

        self.path = Path()
        self.path.header.frame_id = 'map'
        self.path.header.stamp = rospy.Time(0)
        self.pathMarkers = MarkerArray()


        self.quaternionSet = Quaternion()
        self.quaternionSet = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        self.objects = MarkerArray()
        self.subscriber = rospy.Subscriber('/Obstacles', MarkerArray, self.updateObjects)

        self.delta = 5.0

        self.lastPoint = self.goals[self.goalIncrement]

        
    def updateObjects(self, data):
        if not self.received:
            self.objects = data
            self.received = True

    def findPath(self, start, finish):  

        while True:
            if self.received:   
                self.finish = finish
                self.start = start
                firstNode = mapNode(start)
                self.openList.append(firstNode)
                self.current = firstNode
                self.started = True
                break

    def getPath(self,current):
        
        # rospy.loginfo("Got path")
        self.started = False

        pathTemp = Path()
        pathTemp.header.frame_id = 'map'
        pathTemp.header.stamp = rospy.Time(0)

        pathMarkersTemp = MarkerArray()

        backTrack = current

        while backTrack.parent != None:

            mapMarker = Marker()
            mapMarker.id = self.pathIncrement
            mapMarker.header.frame_id = "map"
            mapMarker.type = mapMarker.SPHERE
            mapMarker.action = mapMarker.ADD
            mapMarker.scale.x = 1
            mapMarker.scale.y = 1
            mapMarker.scale.z = 1
            mapMarker.pose.position.z = 0
            mapMarker.pose.position.x = backTrack.position.x
            mapMarker.pose.position.y = backTrack.position.y
            mapMarker.color.a = 1.0
            mapMarker.color.r = 0.0
            mapMarker.color.g = 0.0
            mapMarker.color.b = 1.0

            self.distanceSum(backTrack.position)

            pathMarkersTemp.markers.append(mapMarker)

            temp = PoseStamped()
            tempPose = Pose()
            tempPose.position.z = 0
            tempPose.position.x = backTrack.position.x
            tempPose.position.y = backTrack.position.y
            temp.header.frame_id = "Path: " + str(self.pathIncrement)
            temp.header.stamp = rospy.Time(0)
            self.pathIncrement += 1
            temp.pose = tempPose
            pathTemp.poses.append(temp)
            backTrack = backTrack.parent
        pathTemp.poses.reverse()
        pathMarkersTemp.markers.reverse()

        for markers in pathMarkersTemp.markers:
            self.pathMarkers.markers.append(markers)

        for pose in pathTemp.poses:
            self.path.poses.append(pose)
        

        
        # mapMarker = Marker()
        # mapMarker.id = self.pathIncrement
        # mapMarker.header.frame_id = "map"
        # mapMarker.type = mapMarker.SPHERE
        # mapMarker.action = mapMarker.ADD
        # mapMarker.scale.x = 5
        # mapMarker.scale.y = 5
        # mapMarker.scale.z = 5
        # mapMarker.pose.position.z = 0
        # mapMarker.pose.position.x = self.start.x
        # mapMarker.pose.position.y = self.start.y
        # mapMarker.color.a = 1.0
        # mapMarker.color.r = 1.0
        # mapMarker.color.g = 0.0
        # mapMarker.color.b = 0.0
        # mapMarker.lifetime = rospy.Time(10)

        # self.pathMarkers.markers.append(mapMarker)

        # mapMarker2 = Marker()
        # mapMarker2.id = self.pathIncrement + 1
        # mapMarker2.header.frame_id = "map"
        # mapMarker2.type = mapMarker2.SPHERE
        # mapMarker2.action = mapMarker2.ADD
        # mapMarker2.scale.x = 5
        # mapMarker2.scale.y = 5
        # mapMarker2.scale.z = 5
        # mapMarker2.pose.position.z = 0
        # mapMarker2.pose.position.x = self.finish.x
        # mapMarker2.pose.position.y = self.finish.y
        # mapMarker2.color.a = 1.0
        # mapMarker2.color.r = 1.0
        # mapMarker2.color.g = 0.0
        # mapMarker2.color.b = 0.0
        # mapMarker2.lifetime = rospy.Time(10)

        # self.pathMarkers.markers.append(mapMarker2)

        if not self.fullPathFound:
            self.increment()

    def increment(self):
        
        self.publishPath()
        self.openList.clear()
        self.closedList.clear()
        if self.goalIncrement == 3:
            self.findPath(self.goals[self.goalIncrement], self.goals[0])
            self.fullPathFound = True
            self.algorithmTest()
            self.reset()
        else:
            self.findPath(self.goals[self.goalIncrement], self.goals[self.goalIncrement + 1])
            self.pathfound = False

        self.goalIncrement += 1

    def reset(self):
        if self.resetCounter < 0:
            self.startTime = time.time()
            self.goalIncrement = -1
            self.size = 0
            self.pathIncrement = 0
            self.distSum = 0.0
            self.fullPathFound = False
            self.pathfound = False
            self.pathDrawn = False
            self.received = False
            self.resetCounter += 1
        else:
            rospy.loginfo(distanceLog)
            # rospy.loginfo(distanceLog)

    
    def algorithmTest(self):
        # rospy.loginfo(self.distSum)
        distanceLog.append(self.distSum)
        totalTime = time.time() - self.startTime
        timeLog.append(totalTime)
        # rospy.loginfo("Seconds: " + str(totalTime.secs) + "." + str(totalTime.nsecs))


    def distanceSum(self, nextPoint):
        self.distSum += self.getDist(self.lastPoint, nextPoint)
        self.lastPoint = nextPoint

    def findPathLoop(self, event=None):

        if self.started:
            if not self.current is None:
                if not self.pathfound:
                    self.current = self.getMin()
                    self.addAdjacent(self.current)
                    

    def publishPath(self):
        if len(self.pathMarkers.markers) > 0:
            self.pathSolutionPublisherMarkers.publish(self.pathMarkers)
            # old = self.pathMarkers.markers[0].pose.position
            # for marker in self.pathMarkers.markers:
            #     distance = math.sqrt((old.x - marker.pose.position.x)**2 + (old.y - marker.pose.position.y)**2)
            #     print(distance) 
            #     old = marker.pose.position

        self.pathSolutionPublisher.publish(self.path)
        # br = TransformBroadcaster()
        # temp = 0

        # for x in self.pathMarkers.markers:
        #     if temp > 0:
        #         br.sendTransform((x.pose.position.x , x.pose.position.y, 0),tf.transformations.quaternion_from_euler(0.0, 0.0, 0),rospy.Time.now(),"WayPoint: " + str(temp),"WayPoint: " + str(temp-1))
        #     else:
        #         br.sendTransform((x.pose.position.x , x.pose.position.y, 0),tf.transformations.quaternion_from_euler(0.0, 0.0, 0),rospy.Time.now(),"WayPoint: " + str(temp),"map")
        #     temp += 1


    def getF(self,x,y):
            out = math.sqrt((x-self.finish.x)**2 + (y-self.finish.y)**2)
            return out
        
    def getDist(self,pos1,pos2):
        temp = math.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2)
        return temp

    def checkViability(self,position):

        for obstacle in self.objects.markers:
            if self.getDist(obstacle.pose.position,position) < self.delta*9:
                return False
        for point in self.openList:
            if self.getDist(point.position, position) < self.delta:
                return False
        for point in self.closedList:
            if self.getDist(point.position, position) < self.delta:
                return False
        return True


    def addAdjacent(self,currentIn):
        #Action space
        # rospy.loginfo(len(self.openList))
        
        if not currentIn.fScore <= self.delta:

            #up
            up = Vector3(currentIn.position.x, currentIn.position.y + self.delta, currentIn.position.z)

            if self.checkViability(up):
                temp = mapNode(up)
                temp.parent = currentIn
                temp.fScore = self.getF(up.x,up.y)
                temp.movementSteps = currentIn.movementSteps + self.delta
                self.openList.append(temp)
                # rospy.loginfo("appended up")
            
            #down
            down = Vector3(currentIn.position.x, currentIn.position.y - self.delta, currentIn.position.z)
            if self.checkViability(down):
                temp = mapNode(down)
                temp.parent = currentIn
                temp.fScore = self.getF(down.x,down.y)
                temp.movementSteps = currentIn.movementSteps + self.delta
                self.openList.append(temp)
                # rospy.loginfo("appended down")
            
            #right
            right = Vector3(currentIn.position.x + self.delta, currentIn.position.y , currentIn.position.z)
            if self.checkViability(right):
                temp = mapNode(right)
                temp.parent = currentIn
                temp.fScore = self.getF(right.x,right.y)
                temp.movementSteps = currentIn.movementSteps + self.delta
                self.openList.append(temp)
                # rospy.loginfo("appended right")
            
            #left
            left = Vector3(currentIn.position.x - self.delta, currentIn.position.y , currentIn.position.z)
            if self.checkViability(left):
                temp = mapNode(left)
                temp.parent = currentIn
                temp.fScore = self.getF(left.x,left.y)
                temp.movementSteps = currentIn.movementSteps + self.delta
                self.openList.append(temp)
                # rospy.loginfo("appended left")


            #right-top
            RT = Vector3(currentIn.position.x + self.delta, currentIn.position.y + self.delta , currentIn.position.z)
            if self.checkViability(RT):
                temp = mapNode(RT)
                temp.parent = currentIn
                temp.fScore = self.getF(RT.x,RT.y)
                temp.movementSteps = currentIn.movementSteps + self.delta*math.sqrt(2)
                self.openList.append(temp)
                # rospy.loginfo("appended right-top")
            
            #right-bottom
            RB = Vector3(currentIn.position.x + self.delta, currentIn.position.y - self.delta , currentIn.position.z)
            if self.checkViability(RB):
                temp = mapNode(RB)
                temp.parent = currentIn
                temp.fScore = self.getF(RB.x,RB.y)
                temp.movementSteps = currentIn.movementSteps + self.delta*math.sqrt(2)
                self.openList.append(temp)
                # rospy.loginfo("appended right-bottom")

            #left-bottom
            LB = Vector3(currentIn.position.x - self.delta, currentIn.position.y - self.delta , currentIn.position.z)
            if self.checkViability(LB):
                temp = mapNode(LB)
                temp.parent = currentIn
                temp.fScore = self.getF(LB.x,LB.y)
                temp.movementSteps = currentIn.movementSteps + self.delta*math.sqrt(2)
                self.openList.append(temp)
                # rospy.loginfo("appended left-bottom")
            
            #left-top
            LT = Vector3(currentIn.position.x - self.delta, currentIn.position.y + self.delta , currentIn.position.z)
            if self.checkViability(LT):
                temp = mapNode(LT)
                temp.parent = currentIn
                temp.fScore = self.getF(LT.x,LT.y)
                temp.movementSteps = currentIn.movementSteps + self.delta*math.sqrt(2)
                self.openList.append(temp)
                # rospy.loginfo("appended left-top")

            self.closedList.append(currentIn)
            self.openList.remove(currentIn)
        else:
            self.pathfound = True
            self.getPath(currentIn)

    def getMin(self):
        if len(self.openList) > 0:
            minNode = self.openList[0]

            for x in self.openList:
                if x.fScore < minNode.fScore:
                    minNode = x
            return minNode
        return None
            

    def outputMap(self, event=None):

        mapArray = MarkerArray()
        count = 0

        for node in self.openList:
            
            mapMarker = Marker()
            mapMarker.id = count
            mapMarker.header.frame_id = "map"
            mapMarker.type = mapMarker.CUBE
            mapMarker.action = mapMarker.ADD
            mapMarker.scale.x = 0.5
            mapMarker.scale.y = 0.5
            mapMarker.scale.z = 0.5

            mapMarker.pose.position.x = node.position.x
            mapMarker.pose.position.y = node.position.y
            mapMarker.pose.position.z = 0

            mapMarker.color.a = 1.0
            mapMarker.color.r = 0.0
            mapMarker.color.g = 0.0
            mapMarker.color.b = 0.0

            mapMarker.color.b = 1.0
            mapMarker.color.g = 1.0
            mapMarker.lifetime = rospy.Time(10)

            mapArray.markers.append(mapMarker)
            count += 1

        for node in self.closedList:
            
            mapMarker = Marker()
            mapMarker.id = count
            mapMarker.header.frame_id = "map"
            mapMarker.type = mapMarker.CUBE
            mapMarker.action = mapMarker.ADD
            mapMarker.scale.x = 0.5
            mapMarker.scale.y = 0.5
            mapMarker.scale.z = 0.5

            mapMarker.pose.position.x = node.position.x
            mapMarker.pose.position.y = node.position.y
            mapMarker.pose.position.z = 0

            mapMarker.color.a = 1.0
            mapMarker.color.r = 0.0
            mapMarker.color.g = 0.0
            mapMarker.color.b = 0.0

            mapMarker.color.b = 0.5
            mapMarker.color.g = 0.5
            mapMarker.color.r = 0.5
            mapMarker.lifetime = rospy.Time(10)

            mapArray.markers.append(mapMarker)
            count += 1

        self.pathSearchPublisher.publish(mapArray)
        self.publishPath()


if __name__ == '__main__':
    try:
        rospy.init_node('Map', anonymous=True)
        goals = []
        # # Basic map
        goals.append(Vector3(200.0,0.0,0.0))
        goals.append(Vector3(0.0,-200.0,0.0))
        goals.append(Vector3(-200.0,0.0,0.0))
        goals.append(Vector3(0.0,200.0,0.0))

        # Advanced map

        # goals.append(Vector3(0.0,180.0,0.0))
        # goals.append(Vector3(780.0,150.0,0.0))
        # goals.append(Vector3(675.0,0.0,0.0))
        # goals.append(Vector3(780.0,-150.0,0.0))
        # goals.append(Vector3(0.0,-180.0,0.0))
        # goals.append(Vector3(-180.0,0.0,0.0))

        # A* implementation testing
        # goals.append(Vector3(-500.0,-500.0,0.0))
        # goals.append(Vector3(500.0,500.0,0.0))

        full_Map = fullMap(goals)

        full_Map.increment()
        
        rospy.Timer(rospy.Duration(0.1), full_Map.outputMap)

        rospy.Timer(rospy.Duration(0.0001), full_Map.findPathLoop)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


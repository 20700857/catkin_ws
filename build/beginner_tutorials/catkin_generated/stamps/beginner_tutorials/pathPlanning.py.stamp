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

    def mapCreate(self, size):

        self.size = size

        for x in range(size):
            self.map.append([])
            for y in range(size):
                pos = Vector3()
                pos.x = x - size/2
                pos.y = y - size/2
                pos.z = 0

                node = mapNode(pos,x ,y)
                self.map[x].append(node)

    def getWorldObstacles(self, data):
        self.worldObstacles = data

    def mapdiscretization(self, event = None):
        self.objectsReceived = True
        for x in self.worldObstacles.markers:
            
            self.map[int(x.pose.position.x - self.size/2 - 1)][int(x.pose.position.y - self.size/2)].stateObstacle = True

    def getMinOpenNode(self):
        if len(self.openList) > 0:
            minNode = self.openList[0]

            for x in self.openList:
                if x.fScore < minNode.fScore:
                    minNode = x
            return minNode

    def addAdjacents(self):
        tempX = self.current.indexX
        tempY = self.current.indexY
        #up
        if tempY + 1 < self.size:
            if not self.map[tempX][tempY + 1].stateChecked and not self.map[tempX][tempY + 1].stateObstacle:
                self.addAdjacent(tempX, tempY + 1)
        
        #down
        if tempY - 1 > 0:
            if not self.map[tempX][tempY - 1].stateChecked and not self.map[tempX][tempY - 1].stateObstacle:
                self.addAdjacent(tempX, tempY - 1)
        
        #left
        if tempX - 1 > 0:
            if not self.map[tempX - 1][tempY].stateChecked and not self.map[tempX - 1][tempY].stateObstacle:
                self.addAdjacent(tempX - 1, tempY)

        #right
        if tempX + 1 < self.size:
            if not self.map[tempX + 1][tempY].stateChecked and not self.map[tempX + 1][tempY].stateObstacle:
                self.addAdjacent(tempX + 1, tempY,)
        
        #right-up
        if tempX + 1 < self.size and tempY + 1 < self.size:
            if not self.map[tempX + 1][tempY + 1].stateChecked and not self.map[tempX + 1][tempY + 1].stateObstacle:
                self.addDiagonal(tempX + 1, tempY + 1)
        #right-down
        if tempX + 1 < self.size and tempY - 1 > 0:
            if not self.map[tempX + 1][tempY - 1].stateChecked and not self.map[tempX + 1][tempY - 1].stateObstacle:
                self.addDiagonal(tempX + 1, tempY - 1)
        #left-down
        if tempX - 1 > 0 and tempY - 1 > 0:
            if not self.map[tempX - 1][tempY - 1].stateChecked and not self.map[tempX - 1][tempY - 1].stateObstacle:
                self.addDiagonal(tempX - 1, tempY - 1)
        #left-up
        if tempX - 1 > 0 and tempY + 1 < self.size:
            if not self.map[tempX - 1][tempY + 1].stateChecked and not self.map[tempX - 1][tempY + 1].stateObstacle:
                self.addDiagonal(tempX - 1, tempY + 1)

        self.closedList.append(self.current)
        self.openList.remove(self.current)
        self.map[tempX ][tempY].stateChecked = True
    
    def addDiagonal(self,x, y):
        self.map[x][y].parent = self.current
        self.map[x][y].fScore = self.getF(x,y)
        self.map[x][y].open = True
        self.map[x][y].movementSteps = self.current.movementSteps + 1.5
        self.openList.append(self.map[x][y])

    def addAdjacent(self,x, y):
        self.map[x][y].parent = self.current
        self.map[x][y].fScore = self.getF(x,y)
        self.map[x][y].open = True
        self.map[x][y].movementSteps = self.current.movementSteps + 1
        self.openList.append(self.map[x][y])
    
    def getF(self,x,y): 
        out = math.sqrt((x-self.finish.x)**2 + (y-self.finish.y)**2)
        return out
    
    def findPath(self, start, finish):  

        while True:
            if self.objectsReceived:
                self.finish = finish
    
                self.map[start.x][start.y].stateStart = True
                self.map[finish.x][finish.y].stateEnd = True
                self.openList.append(self.map[start.x][start.y])
                break
            

    def getPath(self):
        
        self.path.header.frame_id = 'map'
        self.path.header.stamp = rospy.Time(0)

        backTrack = self.current

        tempIn = 0

        while backTrack.parent != None:

            mapMarker = Marker()
            mapMarker.id = tempIn
            mapMarker.header.frame_id = "map"
            mapMarker.type = mapMarker.SPHERE
            mapMarker.action = mapMarker.ADD
            mapMarker.scale.x = 0.5
            mapMarker.scale.y = 0.5
            mapMarker.scale.z = 0.5
            mapMarker.pose.position.z = 0
            mapMarker.pose.position.x = self.map[backTrack.indexX][backTrack.indexY].position.x
            mapMarker.pose.position.y = self.map[backTrack.indexX][backTrack.indexY].position.y
            mapMarker.color.a = 1.0
            mapMarker.color.r = 0.0
            mapMarker.color.g = 0.0
            mapMarker.color.b = 1.0

            self.pathMarkers.markers.append(mapMarker)


            self.map[backTrack.indexX][backTrack.indexY].statePath = True

            temp = PoseStamped()
            tempPose = Pose()
            tempPose.position.z = 0
            tempPose.position.x = self.map[backTrack.indexX][backTrack.indexY].position.x
            tempPose.position.y = self.map[backTrack.indexX][backTrack.indexY].position.y
            temp.header.frame_id = "Path: " + str(tempIn)
            temp.header.stamp = rospy.Time(0)
            tempIn += 1
            temp.pose = tempPose
            self.path.poses.append(temp)
            backTrack = backTrack.parent
        
        self.pathfound = True

    def getPathValues(self, event = None):

        br = TransformBroadcaster()

        if self.pathfound:

            self.pathPublisher.publish(self.path)
            self.pathPointPublisher.publish(self.pathMarkers)

            temp = 0

            for x in self.pathMarkers.markers:
                br.sendTransform((x.pose.position.x , x.pose.position.y, 0),tf.transformations.quaternion_from_euler(0.0, 0.0, 0),rospy.Time.now(),"WayPoint: " + str(temp),"map")
                temp += 1

    def findPathLoop(self, event=None):
        if self.objectsReceived:
            if not self.current.stateEnd:
                self.current = self.getMinOpenNode()
                self.addAdjacents()
            else:
                if not self.pathfound:
                    self.getPath()
                    rospy.loginfo("Got path")

    def outputMap(self, event=None):   

        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "world_Map"

        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(0,0,0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        staticBr = tf2_ros.StaticTransformBroadcaster()

        staticBr.sendTransform(static_transformStamped)

        mapArray = MarkerArray()

        for x in range(len(self.map)):
            for y in range(len(self.map[x])):
                mapMarker = Marker()
                mapMarker.id = x*len(self.map) + y
                mapMarker.header.frame_id = "world_Map"
                mapMarker.type = mapMarker.CUBE
                mapMarker.action = mapMarker.ADD
                mapMarker.scale.x = 0.8
                mapMarker.scale.y = 0.8
                mapMarker.scale.z = 0.01
                mapMarker.pose.position = self.map[x][y].position
                mapMarker.color.a = 1.0
                mapMarker.color.r = 0.0
                mapMarker.color.g = 0.0
                mapMarker.color.b = 0.0
                tempOri = tf.transformations.quaternion_from_euler(0.0, 0.0, 0)
                mapMarker.pose.orientation.x = tempOri[0]
                mapMarker.pose.orientation.y = tempOri[1]
                mapMarker.pose.orientation.z = tempOri[2]
                mapMarker.pose.orientation.w = tempOri[3]

                if self.map[x][y].stateChecked:
                    mapMarker.color.b = 0.5
                    mapMarker.color.g = 0.5
                if self.map[x][y].stateViable:
                    mapMarker.color.r = 1.0           
                if self.map[x][y].open:
                    mapMarker.color.b = 0.5
                    mapMarker.color.r = 0.5
                if self.map[x][y].stateObstacle:
                    mapMarker.color.g = 1.0
                if self.map[x][y].statePath:
                    mapMarker.color.r = 0.0
                    mapMarker.color.g = 0.0
                    mapMarker.color.b = 0.0
                if self.map[x][y].stateEnd:
                    mapMarker.color.b = 1.0
                if self.map[x][y].stateStart:
                    mapMarker.color.b = 1.0
                    mapMarker.color.g = 1.0

                mapArray.markers.append(mapMarker)

        self.publisher.publish(mapArray)


if __name__ == '__main__':
    try:
        rospy.init_node('Map', anonymous=True)
        full_Map = fullMap()
        full_Map.mapCreate(50)
        rospy.Timer(rospy.Duration(1), full_Map.mapdiscretization)        
        
        rospy.Timer(rospy.Duration(0.1), full_Map.outputMap)

        start = Vector3()
        start.x = 13
        start.y = 13
        start.z = 0

        finish = Vector3()
        finish.x = 37
        finish.y = 37
        finish.z = 0
        full_Map.findPath(start,finish)

        rospy.Timer(rospy.Duration(0.1), full_Map.findPathLoop)
        rospy.Timer(rospy.Duration(0.1), full_Map.getPathValues)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


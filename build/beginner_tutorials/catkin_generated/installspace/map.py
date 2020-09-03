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
        self.movementSteps = 0

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

    def getPathValues(self, event = None):

        br = TransformBroadcaster()

        if self.pathfound:

            self.pathPublisher.publish(self.path)
            self.pathPointPublisher.publish(self.pathMarkers)

            temp = 0

            for x in self.pathMarkers.markers:
                br.sendTransform((x.pose.position.x , x.pose.position.y, 0),tf.transformations.quaternion_from_euler(0.0, 0.0, 0),rospy.Time.now(),"WayPoint: " + str(temp),"map")
                temp += 1
                
    def mapCreate(self, size):

        self.size = size

        for x in range(size):
            self.map.append([])
            for y in range(size):
                pos = Vector3()
                pos.x = x - size/2 + 0.5
                pos.y = y - size/2 + 0.5
                pos.z = 0

                node = mapNode(pos,x ,y)
                self.map[x].append(node)
    def obstacleCreate(self, num):

        for z in range(num):
            x = random.randint(0,self.size-1)
            y = random.randint(0,self.size-1)
            self.map[x][y].stateObstacle = True
    
    def obstacleRange(self, leftTop, rightBottom):
        for x in range(leftTop.x, rightBottom.x):
            for y in range(leftTop.y, rightBottom.y):
                self.map[x][y].stateObstacle = True

    def findPath(self, start, finish):  

        self.finish = finish
  
        self.map[start.x][start.y].stateStart = True
        self.map[finish.x][finish.y].stateEnd = True
        self.openList.append(self.map[start.x][start.y])

        self.current = mapNode(Vector3(),0,0)

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



    def findPathLoop(self, event=None):

        if not self.current.stateEnd:
            self.current = self.getMin()
            self.addAdjacent(self.current)
        else:
            if not self.pathfound:
                self.getPath()


    def getF(self,x,y):
            out = math.sqrt((x-self.finish.x)**2 + (y-self.finish.y)**2)
            return out

    def addAdjacent(self,currentIn):
        #up
        if currentIn.indexY + 1 < self.size:
            if not self.map[currentIn.indexX][currentIn.indexY + 1].stateChecked and not self.map[currentIn.indexX][currentIn.indexY + 1].stateObstacle:
                self.map[currentIn.indexX][currentIn.indexY + 1].parent = currentIn
                self.map[currentIn.indexX][currentIn.indexY + 1].fScore = self.getF(currentIn.indexX,currentIn.indexY + 1)
                self.map[currentIn.indexX][currentIn.indexY + 1].open = True
                self.map[currentIn.indexX][currentIn.indexY + 1].movementSteps = currentIn.movementSteps + 1
                self.openList.append(self.map[currentIn.indexX][currentIn.indexY + 1])
        
        #down
        if currentIn.indexY - 1 > 0:
            if not self.map[currentIn.indexX][currentIn.indexY - 1].stateChecked and not self.map[currentIn.indexX][currentIn.indexY - 1].stateObstacle:
                self.map[currentIn.indexX][currentIn.indexY - 1].parent = currentIn
                self.map[currentIn.indexX][currentIn.indexY - 1].fScore = self.getF(currentIn.indexX,currentIn.indexY - 1)
                self.map[currentIn.indexX][currentIn.indexY - 1].open = True
                self.map[currentIn.indexX][currentIn.indexY - 1].movementSteps = currentIn.movementSteps + 1
                self.openList.append(self.map[currentIn.indexX][currentIn.indexY - 1])
        
        #left
        if currentIn.indexX - 1 > 0:
            if not self.map[currentIn.indexX -1][currentIn.indexY].stateChecked and not self.map[currentIn.indexX -1][currentIn.indexY].stateObstacle:
                self.map[currentIn.indexX -1][currentIn.indexY].parent = currentIn
                self.map[currentIn.indexX -1][currentIn.indexY].fScore = self.getF(currentIn.indexX -1,currentIn.indexY )
                self.map[currentIn.indexX -1][currentIn.indexY].open = True
                self.map[currentIn.indexX -1][currentIn.indexY].movementSteps = currentIn.movementSteps + 1
                self.openList.append(self.map[currentIn.indexX -1][currentIn.indexY])

        #right
        if currentIn.indexX + 1 < self.size:
            if not self.map[currentIn.indexX + 1][currentIn.indexY].stateChecked and not self.map[currentIn.indexX + 1][currentIn.indexY].stateObstacle:
                self.map[currentIn.indexX + 1][currentIn.indexY].parent = currentIn
                self.map[currentIn.indexX + 1][currentIn.indexY].fScore = self.getF(currentIn.indexX + 1,currentIn.indexY)
                self.map[currentIn.indexX + 1][currentIn.indexY].open = True
                self.map[currentIn.indexX + 1][currentIn.indexY].movementSteps = currentIn.movementSteps + 1
                self.openList.append(self.map[currentIn.indexX + 1][currentIn.indexY])

        self.closedList.append(currentIn)
        self.openList.remove(currentIn)
        self.map[currentIn.indexX ][currentIn.indexY].stateChecked = True


    def getMin(self):
        if len(self.openList) > 0:
            minNode = self.openList[0]

            for x in self.openList:
                if x.fScore < minNode.fScore:
                    minNode = x
            return minNode
            

    def outputMap(self, event=None):

        

        mapArray = MarkerArray()

        for x in range(len(self.map)):
            for y in range(len(self.map[x])):
                mapMarker = Marker()
                mapMarker.id = x*len(self.map) + y
                mapMarker.header.frame_id = "map"
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
        #full_Map.obstacleCreate(100)
        leftTop = Vector3()
        leftTop.x = 5
        leftTop.y = 5

        rightBottom = Vector3()
        rightBottom.x = 45
        rightBottom.y = 45

        full_Map.obstacleRange(leftTop,rightBottom)
        
        rospy.Timer(rospy.Duration(0.1), full_Map.outputMap)

        start = Vector3()
        start.x = 0
        start.y = 0
        start.z = 0

        finish = Vector3()
        finish.x = 49
        finish.y = 49
        finish.z = 0
        full_Map.findPath(start,finish)

        rospy.Timer(rospy.Duration(0.0001), full_Map.findPathLoop)
        rospy.Timer(rospy.Duration(0.1), full_Map.getPathValues)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            


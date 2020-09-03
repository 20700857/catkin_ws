
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import math
import tf
from tf import TransformBroadcaster

headingTimeStep = 0.001
positionTimeStep = 0.01

class Control():

    def __init__(self):
        self.distanceDifference = 0
        self.angleDifference = 0
        self.pathIndex = 1
        self.publisher = rospy.Publisher('Car_Input', Twist, queue_size=10)

        rospy.Subscriber("Car_Telemetries_pos", Float64MultiArray, self.telemetryPos)
        self.inputPos = Float64MultiArray()
        rospy.Subscriber("Car_Telemetries_angle", Float64MultiArray, self.telemetryAngle)
        self.inputAngle = Float64MultiArray()

        self.controlPublisherPos = rospy.Publisher("Car_Control_Position", Float64, queue_size=10)
        self.controlPublisherHead = rospy.Publisher("Car_Control_Heading", Float64, queue_size=10)
        self.listener = tf.TransformListener()
        self.timeStart = rospy.get_time()
        
        self.path = Path()

        self.subscriber = rospy.Subscriber("/Path", Path, self.receivePath)

        self.desiredAngle = 0.0
        self.absoluteDistance = 0.0

        self.distanceThresh = 0.5

        self.vConst = 0.1
        self.wConst = 0.1

        self.headingTime = rospy.get_time()
        self.positionTime = rospy.get_time()

        self.direction = True

    def telemetryPos(self, data):

        self.inputPos = data

    def telemetryAngle(self,data):

        self.inputAngle = data

    def receivePath(self, pathIn):
        self.path = pathIn

    def updateReference(self):

        self.listener.waitForTransform('Car', "WayPoint: " + str(self.pathIndex),rospy.Time(), rospy.Duration(1.0))   

        (trans,rot) = self.listener.lookupTransform( 'Car', "WayPoint: " + str(self.pathIndex), rospy.Time(0))
        
        self.desiredAngle = math.atan2(trans[1],trans[0])
        self.absoluteDistance = math.sqrt(trans[0]**2 + trans[1]**2)

    def updateControlHead(self, event = None):
        
        self.updateReference()

        self.listener.waitForTransform('Car', "WayPoint: " + str(self.pathIndex),rospy.Time(), rospy.Duration(1.0))

        if len(self.path.poses) > 0:

            self.controlPublisherHead.publish(self.desiredAngle)
           


    def updateControlPos(self, event = None):

        self.updateReference()
        
        if len(self.path.poses) > 0:

            if abs(self.desiredAngle) < math.pi/6:

                self.controlPublisherPos.publish(1)
            else:
                self.controlPublisherPos.publish(0)
                
            if self.absoluteDistance < 0.5:
                if self.direction:
                    self.pathIndex = self.pathIndex +1
                    if self.pathIndex == 39:
                        self.direction = not self.direction
                else:
                    self.pathIndex = self.pathIndex -1
                    if self.pathIndex == 1:
                        self.direction = not self.direction
                rospy.loginfo(self.pathIndex)
                pass

if __name__ == '__main__':
    try:
        rospy.init_node('Control', anonymous=True)
        control = Control()

        rospy.Timer(rospy.Duration(headingTimeStep), control.updateControlHead)
        rospy.Timer(rospy.Duration(positionTimeStep), control.updateControlPos)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#! /usr/bin/python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
import math
import tf
from tf import TransformBroadcaster

timeConstant = 100

class Centre(object):

    def __init__(self):

        self.first = Twist()
        self.first.linear.x = 13
        self.first.linear.y = 13
        self.first.linear.z = 0.0
        self.first.angular.x = 0.0
        self.first.angular.y = 0.0
        self.first.angular.z = math.pi

        self.acceleration = 0
        self.velocity = 0

        self.angularAcceleration = 0
        self.angularVelocity = 0

    def update(self, positionInput, headingInput, axil_len):

        #angles
        angularAccelOld = self.angularAcceleration
        self.angularAcceleration = -0.2*self.angularVelocity + headingInput # rad/s^2
        angularVelOld = self.angularVelocity
        self.angularVelocity = headingInput # self.angularVelocity + angularAccelOld/timeConstant # rad/s
        anularOld = self.first.angular.z
        self.first.angular.z = self.first.angular.z + angularVelOld/timeConstant # rad

        angleOut = Float64MultiArray()
        angleOut.data.append(self.first.angular.z)
        angleOut.data.append(self.angularVelocity)
        angleOut.data.append(self.angularAcceleration)

        #Positions
        accelOld = self.acceleration
        self.acceleration = -0.2*self.velocity + positionInput
        velOld = self.velocity
        self.velocity = positionInput #self.velocity + accelOld/timeConstant
        self.first.linear.x = self.first.linear.x + velOld*math.cos(anularOld)/timeConstant
        self.first.linear.y = self.first.linear.y + velOld*math.sin(anularOld)/timeConstant

        posOut = Float64MultiArray()
        posOut.data.append(self.first.linear.x)
        posOut.data.append(self.first.linear.y)
        posOut.data.append(self.velocity) 
        posOut.data.append(self.acceleration) 

        return angleOut, posOut



class Car():

    def __init__(self):

        #car parameters
        self.axel_Length = 0.3
        self.mass = 1

        self.centre = Centre()

        self.marker_Array = MarkerArray()

        left = Marker()
        left.id = 0
        left.header.frame_id = "Wheel_Left"
        left.type = left.CUBE
        left.action = left.ADD
        left.scale.x = 0.25
        left.scale.y = 0.25
        left.scale.z = 0.1
        left.color.a = 1.0
        left.color.r = 1.0
        left.color.g = 0.0
        left.color.b = 0.0
        left.pose.position.x = 0
        left.pose.position.y = 0
        left.pose.position.z = 0
        left.pose.orientation.x = 0.0
        left.pose.orientation.y = 0.0
        left.pose.orientation.z = 0.0
        left.pose.orientation.w = 1.0

        right = Marker()
        right.id = 1
        right.header.frame_id = "Wheel_Right"
        right.type = right.CUBE
        right.action = right.ADD
        right.scale.x = 0.25
        right.scale.y = 0.25
        right.scale.z = 0.1
        right.color.a = 1.0
        right.color.r = 1.0
        right.color.g = 0.0
        right.color.b = 0.0
        right.pose.position.x = 0
        right.pose.position.y = 0
        right.pose.position.z = 0
        right.pose.orientation.x = 0.0
        right.pose.orientation.y = 0.0
        right.pose.orientation.z = 0.0
        right.pose.orientation.w = 1.0

        self.marker_Array.markers.append(left)
        self.marker_Array.markers.append(right)

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.positionInput = 0.0
        self.headingInput = 0.0

        self.carPos = rospy.Publisher('Car_Telemetries_pos', Float64MultiArray, queue_size=10)
        self.carAngle = rospy.Publisher('Car_Telemetries_angle', Float64MultiArray, queue_size=10)
        self.publisher = rospy.Publisher('Car', MarkerArray, queue_size=10)

        self.publisherPath = rospy.Publisher('Car_Path', Path, queue_size=10)

        #publisherFirst = rospy.Publisher('Car_Telemetries/First', Twist, queue_size=10)
        #publisherSecond= rospy.Publisher('Car_Telemetries/Second', Twist, queue_size=10)
        #publisherThird = rospy.Publisher('Car_Telemetries/Third', Twist, queue_size=10)
        #meh

        #rospy.Subscriber('Car_Input', ChannelFloat32, callback)
        rospy.Subscriber('Car_Control_Position', Float64, self.callbackPosition)
        rospy.Subscriber('Car_Control_Heading', Float64, self.callbackHeading)

        #Reference
        self.br = TransformBroadcaster()

        global mass
        mass = 1

        self.tempIn = 0


    def update(self):

        return self.centre.update(self.positionInput,self.headingInput,self.axel_Length)   

    def carLoop(self, event = None):
        # Publish the MarkerArray
        tempAngle, tempPos = self.update()
        self.br.sendTransform((self.centre.first.linear.x, self.centre.first.linear.y, 0),tf.transformations.quaternion_from_euler(0.0, 0.0, self.centre.first.angular.z),rospy.Time.now(),"Car","map")
        self.br.sendTransform((0.0,-self.axel_Length/2, 0.0),(0.0,0.0,0.0,1.0),rospy.Time.now(),"Wheel_Left","Car")
        self.br.sendTransform((0.0, self.axel_Length/2, 0.0),(0.0,0.0,0.0,1.0),rospy.Time.now(),"Wheel_Right","Car")

        temp = PoseStamped()
        tempPose = Pose()
        tempPose.position.z = 0
        tempPose.position.x = self.centre.first.linear.x
        tempPose.position.y = self.centre.first.linear.y
        temp.header.frame_id = "Path Travel: " + str(self.tempIn)
        temp.header.stamp = rospy.Time(0)
        self.tempIn += 1
        temp.pose = tempPose
        if len(self.path.poses) > 100:
            self
        self.path.poses.append(temp)       
        

        self.publisher.publish(self.marker_Array)
        self.publisherPath.publish(self.path)
        self.carAngle.publish(tempAngle)
        self.carPos.publish(tempPos)

    def callbackPosition(self,data):

        self.positionInput = float(data.data)

    def callbackHeading(self, data):

        self.headingInput = float(data.data)


if __name__ == '__main__':
    try:
        rospy.init_node('Car', anonymous=True)
        car = Car()
        rospy.Timer(rospy.Duration(1/timeConstant), car.carLoop)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

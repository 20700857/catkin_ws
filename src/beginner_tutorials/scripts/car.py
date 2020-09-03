
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import math
import tf
from tf import TransformBroadcaster
import cv2 # OpenCV library

def carCreate(marker):
    marker.id = 0
    marker.header.frame_id = "Car"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker

def updateTelemetries():
    global telemetry
    global control_input
    absoluteAcceleration = control_input.linear.x*0.0004

    telemetry[2].linear.x = math.cos(telemetry[0].angular.z)*absoluteAcceleration
    telemetry[2].linear.y = math.sin(telemetry[0].angular.z)*absoluteAcceleration

    absoluteVelocity = math.sqrt(telemetry[1].linear.x**2 + telemetry[1].linear.y**2) + absoluteAcceleration
    absoluteVelocity = absoluteVelocity*0.99

    telemetry[1].linear.x = math.cos(telemetry[0].angular.z)*absoluteVelocity
    telemetry[1].linear.y = math.sin(telemetry[0].angular.z)*absoluteVelocity

    telemetry[0].linear.x = telemetry[0].linear.x + telemetry[1].linear.x
    telemetry[0].linear.y = telemetry[0].linear.y + telemetry[1].linear.y

    telemetry[1].angular.z = control_input.angular.z*0.1
    telemetry[0].angular.z = telemetry[0].angular.z + telemetry[1].angular.z

    outTel = Vector3()
    outTel.x = absoluteVelocity
    outTel.y = absoluteAcceleration
    outTel.z = telemetry[0].angular.z % math.pi
    publisherTel.publish(outTel)


def setTelemetry():
    telemetry = []
    for x in range(3):
        telemetrySet = Twist()
        telemetrySet.angular.x = 0.0
        telemetrySet.angular.y = 0.0
        telemetrySet.angular.z = 0.0
        telemetrySet.linear.x = 0.0
        telemetrySet.linear.y = 0.0
        telemetrySet.linear.z = 0.0
        telemetry.append(telemetrySet)

    telemetry[0].angular.z = math.pi/2
    telemetry[0].linear.x = 4.0
    return telemetry


def car():

    #graphics side
    global marker
    marker = Marker()
    marker = carCreate(marker)

    publisher = rospy.Publisher('Car', Marker, queue_size=10)

    global publisherTel 
    publisherTel = rospy.Publisher('Car_Telemetries', Vector3, queue_size=10)

    rospy.Subscriber('Car_Input', Twist, callback)


    global control_input
    control_input = Twist()

    #Reference
    global br
    br = TransformBroadcaster()

    global telemetry # 0 - position/rotation, 1- velocity/angular_rate, 2 - acceleration/angular_acceleration
    telemetry = setTelemetry()

    global mass
    mass = 1

    

    while not rospy.is_shutdown():

        # Publish the MarkerArray
        updateTelemetries()
        br.sendTransform((telemetry[0].linear.x, telemetry[0].linear.y, telemetry[0].linear.z),tf.transformations.quaternion_from_euler(0.0, 0.0, telemetry[0].angular.z),rospy.Time.now(),"Car","map")
        publisher.publish(marker)
        #rospy.loginfo(telemetry)

        rospy.sleep(0.01)

def callback(data):
    global control_input
    control_input.linear.x = data.linear.x
    control_input.angular.z = data.angular.z
    #rospy.loginfo('receive_input')

if __name__ == '__main__':
    try:
        rospy.init_node('Car', anonymous=True)
        car()
    except rospy.ROSInterruptException:
        pass

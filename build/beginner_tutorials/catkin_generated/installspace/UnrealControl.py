
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
import airsim
from std_msgs.msg import ColorRGBA

headingTimeStep = 0.001
positionTimeStep = 0.01

class Control():

    def __init__(self):

        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

        self.publisher = rospy.Publisher('/Car', Marker, queue_size=10)

        self.distanceDifference = 0
        self.angleDifference = 0
        self.pathIndex = 1

        rospy.Subscriber("RRTPath", MarkerArray, self.getPath)
        
        self.path = MarkerArray()

        self.desiredAngle = 0.0
        self.absoluteDistance = 0.0

        self.distanceThresh = 0.5

        self.vConst = 0.1
        self.wConst = 0.1

        self.listener = tf.TransformListener()

        self.direction = True
    def getPath(self,data):
        self.pathIndex = data

    def publishCar(self,event =None):
        
        self.car_state = self.client.simGetVehiclePose()

        colourSet = ColorRGBA()
        colourSet.a = 1.0
        colourSet.r = 1.0
        colourSet.g = 0.0
        colourSet.b = 0.0

        addingMarker = Marker()
        addingMarker.id = 0
        addingMarker.header.frame_id = 'map'
        addingMarker.pose.position.x = self.car_state.position.x_val - 20
        addingMarker.pose.position.y = -self.car_state.position.y_val
        addingMarker.pose.position.z = self.car_state.position.z_val
        addingMarker.pose.orientation.x = self.car_state.orientation.x_val
        addingMarker.pose.orientation.y = self.car_state.orientation.y_val
        addingMarker.pose.orientation.z = self.car_state.orientation.z_val
        addingMarker.pose.orientation.w = -self.car_state.orientation.w_val
        addingMarker.scale.x = 5
        addingMarker.scale.y = 5
        addingMarker.scale.z = 2.5
        addingMarker.action = addingMarker.ADD
        addingMarker.type = addingMarker.CUBE
        addingMarker.color = colourSet

        self.publisher.publish(addingMarker)


    def getDist(self, point1, point2):
        return math.sqrt((point1.x_val - point2.x)**2 + (point1.y_val - point2.y)**2)

    def getAngle(self, point1, point2):
        return math.atan2(point2.y - point1.y_val,point2.x - point1.x_val)

    def updateControl(self, event = None):
        tempDist = self.getDist(self.car_state.position, self.path.markers[self.pathIndex].pose.position)
        angle = self.getAngle(self.car_state.position, self.path.markers[self.pathIndex].pose.position) - self.car_state.orientation.w_val

        if len(self.path.markers) > 0:
            if tempDist < 2:
                self.pathIndex += 1
                self.car_controls.throttle = self.vConst/2
            else:
                self.car_controls.throttle = self.vConst

            if angle > math.pi/18:
                self.car_controls.steering = self.wConst
            if angle < -math.pi/18:
                self.car_controls.steering = -self.wConst

            self.client.setCarControls(self.car_controls)


if __name__ == '__main__':
    try:
        rospy.init_node('Control', anonymous=True)
        control = Control()

        rospy.Timer(rospy.Duration(0.1), control.publishCar)
        rospy.Timer(rospy.Duration(0.01), control.updateControl)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

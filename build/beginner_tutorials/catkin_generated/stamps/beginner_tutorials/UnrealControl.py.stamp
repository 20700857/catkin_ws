
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

class Control():

    def __init__(self):

        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

        self.car_state = self.client.getCarState()
        self.carTrans = Vector3()
        self.carRot = Quaternion()

        self.carTrans.x = 0
        self.carTrans.y = 0
        self.carTrans.z = 0

        self.carRot.x = 0
        self.carRot.y = 0
        self.carRot.z = 0
        self.carRot.w = 1

        self.destination = []

        self.distanceDifference = 0
        self.angleDifference = 0
        self.pathIndex = 1
        self.pathCount = 9999

        rospy.Subscriber("RRTPath", MarkerArray, self.getPath)
        
        self.path = MarkerArray()

        self.desiredAngle = 0.0
        self.absoluteDistance = 0.0

        self.distanceThresh = 0.5

        self.vConst = 1
        self.wConst = 0.5

        self.br = TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.trans = []
        self.rot = []

        self.direction = True
    def getPath(self,data):
        self.path = data
        self.pathCount = len(self.path.markers)


    def getDist(self, first, second):
        return math.sqrt((first.x - second.x)**2 + (first.y - second.y)**2)

    def getAngle(self, first, second):
        return math.atan2((second.y - first.y),(second.x - first.x))

    def updateCarPosition(self):
        self.car_state = self.client.getCarState()
        self.carTrans.x = self.car_state.kinematics_estimated.position.x_val -20
        self.carTrans.y = -self.car_state.kinematics_estimated.position.y_val
        self.carTrans.z = self.car_state.kinematics_estimated.position.z_val

        self.carRot.x = self.car_state.kinematics_estimated.orientation.x_val
        self.carRot.y = self.car_state.kinematics_estimated.orientation.y_val
        self.carRot.z = self.car_state.kinematics_estimated.orientation.z_val
        self.carRot.w = -self.car_state.kinematics_estimated.orientation.w_val

        self.br.sendTransform((self.carTrans.x, self.carTrans.y, 0),(self.carRot.x,self.carRot.y,self.carRot.z, self.carRot.w),rospy.Time.now(),"Car","map")

    def updateTarget(self):
        self.destination = self.path.markers[self.pathCount - self.pathIndex].points[0]
        self.br.sendTransform((self.destination.x, self.destination.y, 0),(self.carRot.x,self.carRot.y,self.carRot.z, self.carRot.w),rospy.Time.now(),"Target","map") 

    def updateControl(self, event = None):  

        while not rospy.is_shutdown():
            self.updateCarPosition()
            self.updateTarget()

            tempDist = self.getDist(self.carTrans, self.destination)
            angle = self.getAngle(self.carTrans, self.destination)
            #rospy.loginfo(self.getAngle(self.carTrans, self.destination))
            #rospy.loginfo("Car")
            #rospy.loginfo(self.carRot.w)
            rospy.loginfo("Difference")
            rospy.loginfo(angle - self.carRot.w)


            if len(self.path.markers) > 0:
                if tempDist < 2:
                    self.pathIndex += 1
                    self.car_controls.throttle = 0 #self.vConst/2
                else:
                    if self.car_state.speed > 2:
                        self.car_controls.throttle = 0
                    else:
                        self.car_controls.throttle = 0.2 #0.2 #self.vConst
                    

                if angle - self.carRot.w > 0:
                    self.car_controls.steering = -self.wConst
                if angle - self.carRot.w < 0:
                    self.car_controls.steering = self.wConst

                self.client.setCarControls(self.car_controls)


if __name__ == '__main__':
    try:
        rospy.init_node('Control', anonymous=True)
        control = Control()
        rospy.sleep(1)
        control.updateControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

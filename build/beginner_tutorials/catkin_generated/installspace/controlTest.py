import rospy
from std_msgs.msg import Float64
from tf import TransformBroadcaster
from geometry_msgs.msg import Twist
import tf
import math


class Control():

    def __init__(self):
        rospy.Subscriber('/User_Input', Twist, self.userCallback)
        self.publisherPos = rospy.Publisher('/Car_Control_Position', Float64, queue_size=10)
        self.publisherHead = rospy.Publisher('/Car_Control_Heading', Float64, queue_size=10)
        self.inTwist = Twist()
        self.pos = Float64()
        self.head = Float64()

    def userCallback(self, data):
        self.inTwist = data

    def control(self):
    
        while not rospy.is_shutdown():

            self.updateControl()

    def updateControl(self):

        self.publisherPos.publish(self.inTwist.linear.x)
        self.publisherHead.publish(self.inTwist.angular.z)

if __name__ == '__main__':
    try:
        rospy.init_node('Control', anonymous=True)
        control = Control()
        control.control()
    except rospy.ROSInterruptException:
        pass

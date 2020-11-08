import rospy
from std_msgs.msg import Float64
from tf import TransformBroadcaster
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import tf
import math


class Control():

    def __init__(self):
        self.data = rospy.wait_for_message('/Path_Solution_Points', MarkerArray)
        self.cyclicPublisher = rospy.Publisher('/Cyclic_Points', Marker, queue_size=10)
        self.cyclicSize = len(self.data.markers)
        self.cycleVal = 0

    def cyclePoints(self, event=None):
        tempMarker = self.data.markers[self.cycleVal]
        tempMarker.lifetime = rospy.Time(1)
        self.cyclicPublisher.publish(tempMarker)
        self.cycleVal += 1
        if self.cycleVal == self.cyclicSize:
            self.cycleVal = 0

        

if __name__ == '__main__':
    try:
        rospy.init_node('ControlTest', anonymous=True)
        control = Control()
        rospy.Timer(rospy.Duration(0.1), control.cyclePoints)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

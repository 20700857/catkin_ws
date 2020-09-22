
import rospy
import numpy as np
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
import airsim
from cv_bridge import CvBridge

class videoIn():

    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.image_pub = rospy.Publisher("/image_topic_2", Image, queue_size=10)
        self.bridge = CvBridge()

    def publishImage(self, event = None):
        responses = self.client.simGetImages([
            # png format
            airsim.ImageRequest(0, airsim.ImageType.Scene), 
            # uncompressed RGB array bytes
            airsim.ImageRequest(1, airsim.ImageType.Scene, False, False),
            # floating point uncompressed image
            airsim.ImageRequest(1, airsim.ImageType.DepthPlanner, True)])
        
        self.image_pub.publish(responses[0])


if __name__ == '__main__':
    try:
        rospy.init_node('video_pub_py', anonymous=True)
        videoClass = videoIn()
        rospy.Timer(rospy.Duration(0.1), videoClass.publishImage)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

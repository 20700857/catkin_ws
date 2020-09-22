
import rospy
import numpy as np
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
import airsim
import matplotlib.pyplot as plt
class videoIn():

    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()

    def inImage(self, event = None):
        png_image = self.client.simGetImage("0", airsim.ImageType.Scene)     
        plt.imshow(png_image)
        plt.show()



if __name__ == '__main__':
    try:
        rospy.init_node('video_pub_py', anonymous=True)
        video = videoIn()
        rospy.Timer(rospy.Duration(1), video.inImage)
    except rospy.ROSInterruptException:
        pass

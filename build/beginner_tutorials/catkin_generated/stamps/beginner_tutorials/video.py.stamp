
import rospy
import numpy as np
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
import airsim
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os.path 

class videoIn():

    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.image_pub = rospy.Publisher("/image_topic_2", Image, queue_size=10)
        self.bridge = CvBridge()

    def publishImage(self, event = None):
        responses = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        response = responses[0]
        
        # get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

        # reshape array to 4 channel image array H X W X 4
        img_rgb = img1d.reshape(response.height, response.width, 3)

        output_image = Image()
        output_image.header.stamp = rospy.Time.now()
        output_image.height = response.height
        output_image.width = response.width
        output_image.encoding = "rgb8"
        output_image.is_bigendian = 0
        output_image.step = 3 * response.height
        output_image.data = img1d
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_rgb, "bgr8"))

        img_CV2 = cv2.imread(response.image_data_uint8)

        plt.imshow(img_rgb)
        plt.show()


if __name__ == '__main__':
    try:
        rospy.init_node('video_pub_py', anonymous=True)
        videoClass = videoIn()
        rospy.Timer(rospy.Duration(1), videoClass.publishImage)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

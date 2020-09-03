
import rospy
import numpy as np
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library



def videoIn():


    pub = rospy.Publisher('video_frames', Image, queue_size=10)

    cap = cv2.VideoCapture(0)

    br = CvBridge()

    face_cascade = cv2.CascadeClassifier('/home/raynhardt/catkin_ws/src/beginner_tutorials/include/beginner_tutorials/haarcascade_frontalface_default.xml')

    while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
        _, img = cap.read()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
         
        # Detect the faces
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)

        # Draw the rectangle around each face
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # Display
        pub.publish(br.cv2_to_imgmsg(img))
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        rospy.init_node('video_pub_py', anonymous=True)
        videoIn()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

#node reads from webcam and publishes under topic "webcam"
import cv2
import numpy as np 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class webcam_node:

    def __init__(self):
        self.image_pub = rospy.Publisher("webcam",Image,queue_size = 10)
        self.bridge = CvBridge()


    def webcam_publisher(self):
        rospy.init_node('webcam_publisher', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        cap = cv2.VideoCapture(1)
        while not rospy.is_shutdown():

            ret, frame = cap.read()
            # displaying output
            # cv2.imshow('frame',frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame))
            except:
                print("cannot publish")
            rate.sleep()

# When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        webcam = webcam_node()
        webcam.webcam_publisher()
    except rospy.ROSInterruptException:
        pass
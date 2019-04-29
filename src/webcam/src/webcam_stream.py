#!/usr/bin/env python

#node reads from webcam and publishes under topic "webcam"
import cv2
import numpy as np 
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class webcam_node:

    def __init__(self):
        rospy.init_node('webcam_publisher', anonymous=True)
        self.image_pub = rospy.Publisher("webcam",Image,queue_size = 1)
        self.bridge = CvBridge()

        self.y = 20
        self.z = 40
        params = cv2.SimpleBlobDetector_Params()
        self.error_pub = rospy.Publisher("pixel_error",Float64MultiArray,queue_size = 1)
        params.minThreshold = 10
        params.maxThreshold = 400


        # Filter by Area.
        # params.filterByArea = True
        # params.minArea = 50

        # # Filter by Circularity
        # params.filterByCircularity = True
        # params.minCircularity = 0.1

        # # Filter by Convexity
        # params.filterByConvexity = True
        # params.minConvexity = 0.87

        # # Filter by Inertia
        # params.filterByInertia = True
        # params.minInertiaRatio = 0.01
        self.detector = cv2.SimpleBlobDetector_create(params)

    def webcam_publisher(self):
        rate = rospy.Rate(10) # 10hz
        cap = cv2.VideoCapture(0)
        while not rospy.is_shutdown():

            ret, frame = cap.read()
            # frame = cv2.GaussianBlur(frame,15,0)
            height,width = frame.shape[0:2]
            keypoints = self.detector.detect(frame)
            if len(keypoints) is not 0:
                print(keypoints[0].pt,"Updated")
                self.y,self.z = keypoints[0].pt
                self.error_pixel = np.array([0,self.y-width/2-5,self.z-height/2,1]).reshape(-1,1)
                error_pixel = Float64MultiArray()
                error_pixel.data = self.error_pixel
                self.error_pub.publish(error_pixel)
            else:
                print("no detection")

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame))
            except:
                print("cannot publish")
            

            im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.imshow("Keypoints", im_with_keypoints)
            cv2.waitKey(1)

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
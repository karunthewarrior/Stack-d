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

def find_block_center(img,mask):
    thresh = cv2.bitwise_and(img, img, mask=mask)
    _,contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    area_max = 0
    box_list = []

    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        rect = cv2.minAreaRect(c)
        area = (w*h)

        if(area > area_max and area> 200):
            box = cv2.boxPoints(rect)
            circle = np.array([np.average(box[:,0]),np.average(box[:,1])]).astype(int)
            area_max = area

    if(area_max > 0):
        return circle

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
        # self.detector = cv2.SimpleBlobDetector_create(params)

    def webcam_publisher(self):
        rate = rospy.Rate(10) # 10hz
        cap = cv2.VideoCapture(4)
        while not rospy.is_shutdown():

            ret, frame = cap.read()
            # frame = cv2.GaussianBlur(frame,15,0)
            height,width = frame.shape[0:2]
            hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            maskred = cv2.inRange(hsv_image,(1,100,0),(14,255,255)) #create mask of colours
            # maskgreen = cv2.inRange(hsv_image, (20,50,0),(70,255,255))
            center = find_block_center(frame,maskred)
            # keypoints = self.detector.detect(frame) 
            # if len(keypoints) is not 0:
            if np.any(center):
                self.y,self.z = center[0],center[1]
                self.error_pixel = np.array([0,self.y-width/2,self.z-height/2,1]).reshape(-1,1)
                print(self.error_pixel)
                error_pixel = Float64MultiArray()
                error_pixel.data = self.error_pixel
                self.error_pub.publish(error_pixel)
                cv2.circle(frame,(center[0],center[1]), 5, (0,0,255), -1)
            else:
                error_pixel = Float64MultiArray()
                error_pixel.data = np.array([0,0,0,1])
                # print("not found")
                self.error_pub.publish(error_pixel)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame))
            except:
                print("cannot publish")
            

            # im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            cv2.imshow("Keypoints", frame)
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
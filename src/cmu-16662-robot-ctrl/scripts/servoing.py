import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pprint import pprint


class Servoing_node():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("webcam",Image,self.callback)
        self.x = 0.0
        self.y = 0.0
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0
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

    def callback(self,image):
        im = self.bridge.imgmsg_to_cv2(image) 
        height,width = im.shape[0:2]
        # print("target:",width/2,height/2) 
        keypoints = self.detector.detect(im)
        
        if len(keypoints) is not 0:
            self.x,self.y = keypoints[0].pt
        else:
            print("no detection")
        # print("x=",self.x,"y=",self.y)
        error = np.abs(np.array([self.x-width/2-5,self.y-height/2]))
        print(error)
        im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
         
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(1)


    def getting_marker(self):
        rospy.init_node("getting_marker", anonymous=True)
        rospy.spin()



if __name__ == '__main__':
    try:
        servoing = Servoing_node()
        servoing.getting_marker()
    except rospy.ROSInterruptException:
        pass
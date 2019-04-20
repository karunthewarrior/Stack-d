#!/usr/bin/env python

#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
#http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
#https://alloyui.com/examples/color-picker/hsv

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import scipy.ndimage
from patch_detection.msg import detections
from patch_surf import surf
# from ssearch import selectsearch

block_detect = detections()

block_detect.x = 0
block_detect.y = 0
block_detect.h = 0
block_detect.w = 0
block_detect.detected = False

def find_block(im):
    global detections
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
    cv_image = scipy.ndimage.gaussian_filter(cv_image,sigma=0.7)
    # hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # x = selectsearch(cv_image)
    # x = cv2.resize(x,(cv_image.shape[1],cv_image.shape[0]))
    circ,avg = surf(cv_image)
    for point in circ:
    	cv2.circle(img,(point[0], point[1]), 5, (0,255,0), -1)
    cv2.imshow("realsense_window", x)
    cv2.waitKey(1)
    # #Dark then light colors
    # maskred = cv2.inRange(hsv_image, (1,100,40),(10,255,255)) #create mask of colours
    # maskblue = cv2.inRange(hsv_image, (80,100,40),(130,255,255)) #create mask of colours
    # maskgreen = cv2.inRange(hsv_image, (100,100,40),(110,255,255)) #create mask of colours
    # masksilver = cv2.inRange(hsv_image, (0,0,0),(10,0,100))



    # result = cv2.bitwise_and(cv_image, cv_image, mask=masksilver)
    # list_contours, hierarchy = cv2.findContours(masksilver, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours = np.array(list_contours)
    # result = cv2.drawContours(result, contours, -1, (52, 198, 30))
    
    # area_max = 0
    # p_max =  []
    # for c in contours:
    #     x, y, w, h = cv2.boundingRect(c)
    #     area = (w*h)

    #     if(area>area_max and w > 50 and h > 50):
    #         area_max = area
    #         p_max = [x,y,w,h]
    
    # if(p_max and area_max > 0):
    #     cv2.rectangle(result, (p_max[0], p_max[1]), (p_max[0] + p_max[2], p_max[1] + p_max[3]), (0, 255, 0), 2)
    #     cv2.imshow("realsense_window", result)
    #     cv2.waitKey(1)
    
    #     block_detect.detected = True
    #     block_detect.x = p_max[0]
    #     block_detect.y = p_max[1]
    #     block_detect.w = p_max[2]
    #     block_detect.h = p_max[3]

    #     print (p_max[0], p_max[1], p_max[2], p_max[3])
    # else:
    #     block_detect.detected = False
    #     cv2.imshow("realsense_window", hsv_image)
    #     cv2.waitKey(1)



def main():
    rospy.init_node('color_seg', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    
    pub = rospy.Publisher('block_detection', detections, queue_size=10)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, find_block)
    rospy.Subscriber("/camera/depth/depth_rect", Image, find_block)
    while not rospy.is_shutdown():
        
        # hello_str = "hello world %s" % rospy.get_time()
        pub.publish(block_detect)
        rate.sleep()



if __name__ == '__main__':
    main()


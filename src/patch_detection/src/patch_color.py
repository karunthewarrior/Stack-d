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
# import matplotlib.pyplot as plt
import scipy.ndimage
from patch_detection.msg import detections
# from ssearch import selectsearch
from std_msgs.msg import Float64MultiArray

class block_color():
    def __init__(self):
        self.block_detect = detections()
        self.block_detect.x = 0
        self.block_detect.y = 0
        self.block_detect.h = 0
        self.block_detect.w = 0
        self.block_detect.detected = False
        self.flag_block = False
        self.flag_depth = False
        rospy.init_node('color_seg', anonymous=True)
        self.rate = rospy.Rate(30) # 30hz
        self.pub = rospy.Publisher('block_detection', Float64MultiArray, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.find_block)
        # self.depth_sub = rospy.Subscriber("/camera/depth/color/points", Image, self.find_depth)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.find_depth)

        self.im_coord = np.array([0,0])
        self.x_center = 0
        self.y_center = 0
        self.z_center = 0
        self.position = Float64MultiArray()

    def find_block(self,im):
    	#This function takes in rgb image and find the block
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
        cv_image = scipy.ndimage.gaussian_filter(cv_image,sigma=0.7)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      
        #Dark then light colors
        maskred = cv2.inRange(hsv_image, (1,80,30),(10,255,255)) #create mask of colours
        maskblue = cv2.inRange(hsv_image, (80,100,40),(130,255,255)) #create mask of colours
        maskgreen = cv2.inRange(hsv_image, (100,100,40),(110,255,255)) #create mask of colours
        masksilver = cv2.inRange(hsv_image, (0,0,0),(10,0,100))

        result = cv2.bitwise_and(cv_image, cv_image, mask=maskred)
        _,list_contours, hierarchy = cv2.findContours(maskred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = np.array(list_contours)
        # result = cv2.drawContours(result, contours, -1, (52, 198, 30))
        
        area_max = 0
        area_patch = 0
        p_max =  [0,0,0,0]
        p_patch = []
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = (w*h)

            if(area>area_max and w > 30 and h > 30):
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                area_max = area
                p_max = [x,y,w,h]
        
        if(p_max and area_max > 0):
            image_used = result
            cv2.rectangle(image_used, (p_max[0], p_max[1]), (p_max[0] + p_max[2], p_max[1] + p_max[3]), (0, 255, 0), 2)
            cv2.putText(image_used,'Box',(p_max[0]+p_max[2]+10,p_max[1]+p_max[3]),0,1.0,(0,255,0))
            cv2.drawContours(image_used,[box],0,(0,0,255),2)
        
            self.block_detect.detected = True
            self.block_detect.x = p_max[0]
            self.block_detect.y = p_max[1]
            self.block_detect.w = p_max[2]
            self.block_detect.h = p_max[3]
            self.im_coord[0] = np.average(box[:,0])
            self.im_coord[1] = np.average(box[:,1])
            cv2.circle(image_used,(self.im_coord[0],self.im_coord[1]), 5, (0,0,255), -1)
            cv2.imshow("realsense_window", image_used)
            cv2.waitKey(1)

            # print(box)

            # print (p_max[0], p_max[1], p_max[2], p_max[3])
            self.flag_block = True
        else:
            self.block_detect.detected = False
            cv2.imshow("realsense_window", hsv_image)
            cv2.waitKey(1)

    def find_depth(self,im):
    	#This Function finds depth of the center of the block.
    	#K matrix
        # 618.7474975585938, 0.0, 324.06787109375, 0.0, 
        # 0.0, 619.2664184570312, 246.47152709960938, 0.0, 
        # 0.0, 0.0, 1.0, 0.0
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(im, "passthrough")
        print(cv_image.shape)
        f_x = 618.7474975585938
        f_y = 619.2664184570312
        u_0 = 324.06787109375
        v_0 = 246.47152709960938
        window = 3
        if(self.flag_block == True):
            self.z_center = np.average(cv_image[self.im_coord[1]-window:self.im_coord[1]+window,self.im_coord[0]-window:self.im_coord[0]+window])
            print(self.z_center)
            self.x_center = (self.im_coord[0]-u_0)*self.z_center/f_x
            self.y_center = (self.im_coord[1]-v_0)*self.z_center/f_y
            self.position.data = [self.x_center,self.y_center,self.z_center]
            print(self.x_center,self.y_center,self.z_center)
            self.flag_depth = True

    
    def control_loop(self):
        while not rospy.is_shutdown():
            if(self.flag_depth == True and self.flag_block == True):
                self.pub.publish(self.position)
                self.flag_depth = False
                self.flag_block = False
            self.rate.sleep()




if __name__ == '__main__':
    blocker = block_color()
    blocker.control_loop()


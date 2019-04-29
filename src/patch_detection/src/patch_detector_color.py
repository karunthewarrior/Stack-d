#!/usr/bin/env python

#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
#http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
#https://alloyui.com/examples/color-picker/hsv

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
import scipy.ndimage
from patch_detection.msg import blocks_detected 
# import blocks_detected

class block_color():
    def __init__(self):
        self.flag_block = False
        self.flag_depth = False
        rospy.init_node('color_seg', anonymous=True)
        self.rate = rospy.Rate(30) # 30hz
        self.pub = rospy.Publisher('block_detection', blocks_detected, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.find_block)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.find_depth)
        self.flag_pic_taken = False

        self.x_center = 0
        self.y_center = 0
        self.z_center = 0
        self.circle_list = []
        self.position_list = blocks_detected()


    def find_block(self,im):
        #This function takes in rgb image and find the block
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
        cv_image = scipy.ndimage.gaussian_filter(cv_image,sigma=0.5)
        cv_image = cv_image[100:370,200:400]
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        edged = cv2.Canny(cv_image, 140, 300)
        cv2.imshow("edges",edged)
        cv2.waitKey(1)
        self.circle_list = []


        # list_contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _,list_contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = np.array(list_contours)
        result = cv2.drawContours(cv_image, contours, -1, (52, 198, 30))
        
        area_max = 0
        box_list = []
        bound = np.array([0,0,0,0]).reshape(1,-1)
        for c in contours:
            #Approximating contour shape
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)

            x, y, w, h = cv2.boundingRect(c)
            rect = cv2.minAreaRect(c)
            area = (w*h)

            #Bounding box supression
            candidate = np.array([x,y,x+w,y+h]).reshape(1,-1)
            difference = np.sum(np.abs(bound - candidate)**2,axis=-1)**(1./2)
            min_difference = np.min(difference)
            if(min_difference > 5):
                bound = np.append(bound,candidate,0)
                if(w > 10 and h > 10 and w <200 and h<200 and len(approx) ==4 ):
                    print(approx)
                    box = cv2.boxPoints(rect)
                    self.circle_list.append(np.array([np.average(box[:,0]),np.average(box[:,1])]))
                    box_list.append(np.int0(box))
                    area_max = area

        if(area_max > 0):
            image_used = result
            [cv2.drawContours(image_used,[boxes],0,(0,0,255),2) for boxes in box_list]
            [cv2.circle(image_used,(circle[0],circle[1]), 5, (0,0,255), -1) for circle in self.circle_list]

            cv2.imshow("realsense_window", image_used)
            cv2.waitKey(1)

            self.flag_block = True
        else:
            self.flag_block = False
            cv2.imshow("realsense_window", cv_image)
            cv2.waitKey(1)

    def find_depth(self,im):
        #This Function finds depth of the center of the block.
        #K matrix
        # 618.7474975585938, 0.0, 324.06787109375, 0.0, 
        # 0.0, 619.2664184570312, 246.47152709960938, 0.0, 
        # 0.0, 0.0, 1.0, 0.0
        circles = self.circle_list
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(im, "passthrough")
        f_x = 618.7474975585938
        f_y = 619.2664184570312
        u_0 = 324.06787109375
        v_0 = 246.47152709960938
        window = 3
        current_msg = blocks_detected()
        if(self.flag_block == True):
            for points in circles:
                point_int = np.round(points).astype(int)
                self.z_center = np.average(cv_image[point_int[1]-window:point_int[1]+window,point_int[0]-window:point_int[0]+window])
                self.x_center = (points[0]-u_0)*self.z_center/f_x
                self.y_center = (points[1]-v_0)*self.z_center/f_y
                self.position_list.x.append(self.x_center)
                self.position_list.y.append(self.y_center)
                self.position_list.z.append(self.z_center)

            self.flag_depth = True


    def control_loop(self):
        while not rospy.is_shutdown():
            if(self.flag_depth == True and self.flag_block == True):
                self.pub.publish(self.position_list)
                self.position_list = blocks_detected()
                self.flag_depth = False
                self.flag_block = False
            self.rate.sleep()


if __name__ == '__main__':
    blocker = block_color()
    blocker.control_loop()


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
import math



class block_color():
    def __init__(self):
        self.flag_block = False
        self.flag_depth = False
        rospy.init_node('scene_analysis', anonymous=True)
        self.rate = rospy.Rate(30) # 30hz
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", blocks_detected, self.find_depth)

    def arrange(self,bloks):
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
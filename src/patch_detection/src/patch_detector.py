#!/usr/bin/env python

#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
#http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
#https://alloyui.com/examples/color-picker/hsv

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import scipy.ndimage
from patch_detection.msg import detections
from ssearch import selectsearch
import pickle 
import sys,os, rospkg
rospack = rospkg.RosPack()
import time
# script_path = os.path.join(rospack.get_path("patch_detection"), "src", "model.p")
# sys.path.insert(0,script_path)
# print(sys.path)

# with open(os.path.join(rospack.get_path("patch_detection"), "src", "model.p"), 'rb') as model_file:
    # model = model_file


block_detect = detections()

block_detect.x = 0
block_detect.y = 0
block_detect.h = 0
block_detect.w = 0
block_detect.detected = False
query = np.array([0,0])
dep = Float64()

def find_depth(im):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(im, "16UC1")
    cv_image = scipy.ndimage.gaussian_filter(cv_image,sigma=0.7)
    depth = cv_image[query[0],query[1]]
    dep.data = depth
    rospy.loginfo(depth)



def find_block(im):
    st_time = time.time()
    global detections
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
    cv_image = scipy.ndimage.gaussian_filter(cv_image,sigma=0.7)
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    x,rect = selectsearch(cv_image,model)
    x = cv2.resize(x,(cv_image.shape[1],cv_image.shape[0]))
    end_time = time.time()
    cv2.imshow("realsense_window", x)
    cv2.waitKey(1)
    # rospy.loginfo(end_time-st_time)
    query[0] = rect[0]
    query[1] = rect[1]


    
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
    
    block_detect.detected = True
    block_detect.x = rect[0]
    block_detect.y = rect[1]
    block_detect.w = rect[2]
    block_detect.h = rect[3]

    #     print (p_max[0], p_max[1], p_max[2], p_max[3])
    # else:
    #     block_detect.detected = False
    #     cv2.imshow("realsense_window", hsv_image)
    #     cv2.waitKey(1)



def main():
    rospy.init_node('color_seg', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    
    pub_block = rospy.Publisher('block_detection', detections, queue_size=3)
    # pub_depth = rospy.Publisher('patch_depth', Float64, queue_size=3)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, find_block)
    # depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, find_depth)
    while not rospy.is_shutdown():
        
        # hello_str = "hello world %s" % rospy.get_time()
        pub_block.publish(block_detect)
        # pub_depth.publish(dep)
        rate.sleep()



if __name__ == '__main__':
    model = pickle.load(open(os.path.join(rospack.get_path("patch_detection"), "src", "model.p"), 'rb'))
    main()


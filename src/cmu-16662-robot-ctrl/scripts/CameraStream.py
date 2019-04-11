import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image

class CameraStream():
    def __init__(self):
        self.cam_image_topic = "/camera/color/image_raw"
        self.image_sub = rospy.Subscriber(self.cam_image_topic,Image,self.get_image)
        self.img = 0

    def get_image(self,img_msg):
        bridge = CvBridge()
        self.img = bridge.imgmsg_to_cv2(img_msg, "bgr8")

if __name__ == "__main__":
    rospy.init_node('cam_stream_node', anonymous=True)
    cam = CameraStream()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        cv2.imshow("Camera Stream",cam.img)
        cv2.waitKey(3)
    # rospy.spin()
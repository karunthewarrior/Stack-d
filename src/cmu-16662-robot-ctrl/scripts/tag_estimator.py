import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64MultiArray,Float64
import rospy
import math
import kinematics as kin


class tag_estimator:
    def __init__(self):
        self.camera_pose_topic = "/ar_pose_marker"
        self.position_camera = np.array([0,0,0,1])
        self.world_point_topic = "/marker_pos"

        self.pan_state_topic = "/pan/state"
        self.tilt_state_topic = "/tilt/state"

        self.sub_pan = rospy.Subscriber(self.camera_pose_topic,Float64,self.get_pan)
        self.sub_tilt = rospy.Subscriber(self.camera_pose_topic,Float64,self.get_tilt)

        self.sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_point)
        self.pub = rospy.Publisher(self.world_point_topic,Float64MultiArray, queue_size=1)

    def get_pan(self,pan):
        self.pan = pan.data

    def get_tilt(self,tilt):
        self.tilt = tilt.data

    def get_point(self,tag):
        try:
            ar_position_obj = tag.markers[0].pose.pose.position
            # self.position_camera = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z,1])
            self.position_camera = np.array([ar_position_obj.z,-ar_position_obj.x,ar_position_obj.y,1])
        except:
            self.position_camera = np.array([0,0,0,1])

        self.set_point()

    def set_point(self):
        pub_data=Float64MultiArray()
        H_c2w = kin.cam_to_world(self.pan,self.tilt)
        point = np.dot(H_c2w,self.position_camera)
        pub_data.data = point
        rospy.loginfo(pub_data)
        self.pub.publish(pub_data)


if __name__ =="__main__":
    rospy.init_node('marker_publisher', anonymous=True)
    rospy.sleep(1)
    estimator = tag_estimator()
    rospy.sleep(1)
    rospy.spin()
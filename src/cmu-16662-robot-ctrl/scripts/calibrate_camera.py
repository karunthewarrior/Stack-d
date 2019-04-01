import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import 

camera_pose_topic = '/ar_pose_marker'

def get_camera_pose(camera_pose):
    time = joint_state.header.stamp
    camera_pose = np.array(joint_state.position)[0:5]
    rospy.loginfo(camera_pose)

if __name__ == "__main__":
	rospy.init_node('listener', anonymous=True)
	sub = rospy.Subscriber(camera_pose_topic,JointState,get_joint_state)
	rospy.spin()

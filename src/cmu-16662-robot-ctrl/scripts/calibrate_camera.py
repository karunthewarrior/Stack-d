import sys
import numpy as np
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import kinematics as kin
import ArmController as controller


class camera_calib:
    def __init__(self):
        self.camera_pose_topic = "/ar_pose_marker"
        self.ar_position = np.array([None,None,None])
        sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_camera_pose)

    def get_camera_pose(self,camera_pose):
        time = camera_pose.header.stamp
        try:
            ar_position_obj = camera_pose.markers[0].pose.pose.position
            self.ar_position = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z])
            rospy.loginfo(self.ar_position)
        except:
            self.ar_position = np.array([None,None,None])

if __name__ == "__main__":
    rospy.init_node("camera_calib", anonymous=True)
    cam = camera_calib()
    controller = controller.ArmController()
    rospy.sleep(2)
    target_joints = [[0,-10,20,-70,0]]
    for joint in target_joints:
        controller.set_joint_state(joint)
        while(not controller.has_converged()):
            pass
        cam_pos = cam.ar_position
        arm_pos = kin.forward_kinematics(joint)[0][3,:3]
        rospy.loginfo(cam_pos)
        rospy.loginfo(arm_pos)

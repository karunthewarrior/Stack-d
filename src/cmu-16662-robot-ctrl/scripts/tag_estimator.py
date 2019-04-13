import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64MultiArray,Float64
import rospy
import math
import kinematics as kin
import ArmController as ac

class tag_estimator:
    def __init__(self):
        self.camera_pose_topic = "/ar_pose_marker"
        self.position_camera = np.array([0,0,0,1])
        self.world_point_topic = "/marker_pos"

        self.pan_state_topic = "/pan/state"
        self.tilt_state_topic = "/tilt/state"

        self.sub_pan = rospy.Subscriber(self.pan_state_topic,Float64,self.get_pan)
        self.sub_tilt = rospy.Subscriber(self.tilt_state_topic,Float64,self.get_tilt)

        self.sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_point)
        self.pub = rospy.Publisher(self.world_point_topic,Float64MultiArray, queue_size=1)

    def get_pan(self,pan):
        self.pan = pan.data

    def get_tilt(self,tilt):
        self.tilt = tilt.data

    def get_point(self,tag):
        try:
            ar_position_obj = tag.markers[0].pose.pose.position
            rospy.loginfo(ar_position_obj)
            self.position_camera = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z,1])
        # self.position_camera = np.array([ar_position_obj.z,-ar_position_obj.x,ar_position_obj.y,1])
        except:
            # rospy.loginfo("No detection")
            pass
        # self.position_camera = np.array([0,0,0,0])
        # self.set_point()

    def set_point(self):
        pub_data=Float64MultiArray()
        H_c2w = kin.cam_to_world(self.pan,self.tilt)
        # print("H",H_c2w)
        point = np.dot(H_c2w,self.position_camera)
        pub_data.data = np.array([point[0],point[1],point[2]])
        rospy.loginfo(pub_data)
        self.pub.publish(pub_data)


if __name__ =="__main__":
    rospy.init_node('marker_publisher', anonymous=True)
    estimator = tag_estimator()
    arm_controller = ac.ArmController()

    tilt_controller = ac.CamController('/tilt/state','/tilt/command')
    pan_controller = ac.CamController('/pan/state','/pan/command')
    rospy.sleep(1)
    pan_controller.set_cam_state(np.deg2rad(0))
    while(not pan_controller.has_converged()):
        pass
    tilt_controller.set_cam_state(np.deg2rad(-40))
    while(not tilt_controller.has_converged()):
        pass
    rospy.sleep(3)

    target_joints = []
    H_c2w = kin.cam_to_world(estimator.pan,estimator.tilt)
    point = np.dot(H_c2w,estimator.position_camera)
    # rospy.loginfo(estimator.position_camera)
    pos_list = [point[:3]+np.array([0,0,0.05]),point[:3]]
    rospy.loginfo(pos_list)
    for pos in pos_list:
        q = kin.inverse_kinematics(pos)
        print(kin.forward_kinematics(q)[0]["joint_4"])
        if q!=None:
            target_joints.append(q)
        else:
            print("No solution")
            exit()

    arm_controller.home_arm()
    arm_controller.open()
    for joint in target_joints:
        arm_controller.set_joint_state(joint)
        while(not arm_controller.has_converged()):
            pass
    arm_controller.close()
    rospy.sleep(0.5)
    arm_controller.home_arm()
    # while(not rospy.is_shutdown()):
        # estimator.set_point()

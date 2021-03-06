import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64MultiArray,Float64
import rospy
import math
import kinematics as kin
import ArmController as ac
import transforms3d as tf

class tag_estimator:
    def __init__(self):
        self.camera_pose_topic = "/ar_pose_marker"
        self.camera_pose_topic = "block_detection"
        self.position_camera = np.array([0,0,0,1])
        self.world_point_topic = "/marker_pos"

        self.pan_state_topic = "/pan/state"
        self.tilt_state_topic = "/tilt/state"

        self.sub_pan = rospy.Subscriber(self.pan_state_topic,Float64,self.get_pan)
        self.sub_tilt = rospy.Subscriber(self.tilt_state_topic,Float64,self.get_tilt)

        # self.sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_point)
        self.sub_col = rospy.Subscriber(self.camera_pose_topic,Float64MultiArray,self.get_point)
        self.pub = rospy.Publisher(self.world_point_topic,Float64MultiArray, queue_size=1)

    def get_pan(self,pan):
        self.pan = pan.data

    def get_tilt(self,tilt):
        self.tilt = tilt.data

    # def get_point(self,tag):
    #     try:
    #         rospy.loginfo(len(tag.markers))
    #         if len(tag.markers) ==2:
    #             # rospy.loginfo("GOT IT")
    #             self.p = []
    #             for marker in tag.markers:
    #                 ar_position_obj = marker.pose.pose.position
    #                 p = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z,1])
    #                 self.p.append(p)

    #             # if marker.id == 2:
    #             #     self.p1 = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z,1])
    #             # elif marker.id == 4:
    #             #     self.p2 = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z,1])
    #         # rospy.loginfo(ar_position_obj)
    #         # self.position_camera = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z,1])
    #     # self.position_camera = np.array([ar_position_obj.z,-ar_position_obj.x,ar_position_obj.y,1])
    #     except:
    #         # rospy.loginfo("No detection")
    #         pass
    #     # self.position_camera = np.array([0,0,0,0])
    #     # self.set_point()

    def get_point(self,pos):
        p = [pos.data[0]/1000,pos.data[1]/1000,(pos.data[2]+5)/1000,1]
        q = np.array([0.500, -0.500, 0.500, 0.500])
        R = tf.quaternions.quat2mat(q)
        H = kin.rot_H(R)
        p_t = np.dot(H,p)
        
        p_end = [p_t[1],p_t[2],p_t[0],1]
        # print(p_end,"point")
        # print(np.dot(H,p),"HIASDHASD")
        self.p = [p_end]


    def set_point(self):
        pub_data=Float64MultiArray()
        H_c2w = kin.cam_to_world(self.pan,self.tilt)
        # print("H",H_c2w)
        point = np.dot(H_c2w,self.position_camera)
        pub_data.data = np.array([point[0],point[1],point[2]])
        rospy.loginfo(pub_data)
        self.pub.publish(pub_data)

def make_trajectory(s,d):
    p_list = [s[:3]+np.array([0,0,0.09]),
                s[:3],
                s[:3]+np.array([0,0,0.09]),
                d[:3]+np.array([0,0,0.09]),
                d[:3],
                d[:3]+np.array([0,0,0.09])]
    return p_list

def make_destination(center,levels):
    dist_x, dist_y = 0.06,0.06
    d_list = []
    for l in range(levels):
        d = np.array([[0,-dist_y/2,(l-1)*0.02],
                    [0,dist_y/2,(l-1)*0.02],
                    [-dist_x/2,0,l*0.02],
                    [dist_x/2,0,l*0.02]]) + center
        d_list.append(d)
    d = np.vstack(d_list)
    return d

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
    rospy.loginfo(len(estimator.p))
    if len(estimator.p) == 1:
        pw = [np.dot(H_c2w,p) for p in estimator.p]
        print(pw,"point1")
        destination = make_destination(np.array([0.30,0.13,0]),levels=2)
        pos_list = []
        rospy.loginfo(destination)
        for s,d in zip(pw,destination):
            # rospy.loginfo(make_trajectory(s,d))   
            pos_list.extend(make_trajectory(s,d))
        rospy.loginfo(pos_list)
    else:
        pos_list = []
    # rospy.loginfo(estimator.position_camera)
    # pos_list = [point1[:3]+np.array([0,0,0.05]),point1[:3],point1[:3]+np.array([0,0,0.05]),point2[:3]+np.array([0,0,0.05]),point2[:3]+np.array([0,0,0.02]),point2[:3]+np.array([0,0,0.05])]
    

    grip = [False,True,True,True,False,False] * len(pw)
    yaw = [0]*12 + [0]*3 + [np.pi/2]*3 + [0]*3 + [np.pi/2]*3   
    rospy.loginfo(pos_list)
    for pos,y in zip(pos_list,yaw):
        q = kin.inverse_kinematics(pos,y)
        print(kin.forward_kinematics(q)[0]["joint_4"])
        if q!=None:
            target_joints.append(q)
        else:
            print("No solution")
            exit()

    arm_controller.home_arm()
    arm_controller.open()
    for joint,g in zip(target_joints,grip):
        arm_controller.set_joint_state(joint)
        while(not arm_controller.has_converged()):
            pass
        if g == True:
            arm_controller.close()
        else:
            arm_controller.open()
    # arm_controller.close()
    rospy.sleep(0.5)
    arm_controller.home_arm()

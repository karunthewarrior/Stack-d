import numpy as np
from std_msgs.msg import Float64MultiArray,Float64
import rospy
import math
import kinematics as kin
import ArmController as ac
import transforms3d as tf
from patch_detection.msg import blocks_detected

"""
Class that interfaces the block position estimation.
1. Subscribes to the block detection topic "block_detection"
2. Converts the position from the camera frame to the world frame.
3. Publishes the position in the world frame to the topic "block_pos"
"""
class block_estimator:
    def __init__(self):
        self.camera_pose_topic = "block_detection"
        self.position_camera = np.array([0,0,0,1])
        self.world_point_topic = "/block_pos"

        self.pan_state_topic = "/pan/state"
        self.tilt_state_topic = "/tilt/state"

        self.sub_pan = rospy.Subscriber(self.pan_state_topic,Float64,self.get_pan)
        self.sub_tilt = rospy.Subscriber(self.tilt_state_topic,Float64,self.get_tilt)

        self.sub_col = rospy.Subscriber(self.camera_pose_topic,blocks_detected,self.get_point)
        self.pub = rospy.Publisher(self.world_point_topic,Float64MultiArray, queue_size=1)

#Reads the pan value.
    def get_pan(self,pan):
        self.pan = pan.data

#Reads the tilt value.
    def get_tilt(self,tilt):
        self.tilt = tilt.data

#Subscriber callback that updates the block position and colour and creates a list of blocks in class variable p.
    def get_point(self,blocks):
        self.p = []
        for x,y,z,c,theta in zip(blocks.x,blocks.y,blocks.z,blocks.color,blocks.angle):
            p = [x/1000,y/1000,(z+5)/1000,1] 
            q = np.array([0.500, -0.500, 0.500, 0.500])
            R = tf.quaternions.quat2mat(q)
            H = kin.rot_H(R)
            p_t = np.dot(H,p)
            p_end = [p_t[1],p_t[2],p_t[0],1]
            self.p.append((p_end,c))

def make_trajectory(s,d,yaw=False):
    p_list = [s[:3]+np.array([0.02,-0.05,0.09]),
                s[:3],
                s[:3]+np.array([0,0,0.09]),
                d[:3]+np.array([0,0,0.09]),
                d[:3],
                d[:3]+np.array([0,0,0.09])]
    grip_list = [False,True,True,True,False,False]
    if yaw:
        yaw_list = [0,0,0,np.pi/2,np.pi/2,0]
    else:
        yaw_list = [0,0,0,0,0,0]
    return p_list,grip_list,yaw_list

def make_trajectory_yaw(s,d,yaw=0):
    p_list = [s[:3]+np.array([0.02,-0.05,0.09]),
                s[:3],
                s[:3]+np.array([0,0,0.09]),
                d[:3]+np.array([0,0,0.09]),
                d[:3],
                d[:3]+np.array([0,0,0.09])]
    grip_list = [False,True,True,True,False,False]
    yaw_list = [0,0,0,yaw,yaw,0]
    return p_list,grip_list,yaw_list

def make_destination(center,levels):
    dist_x, dist_y = 0.06,0.06
    d_list = []
    for l in range(levels):
        d = np.array([[0,-dist_y/2,(l-1)*0.04],
                    [0,dist_y/2,(l-1)*0.04],
                    [-dist_x/2,0,l*0.04],
                    [dist_x/2,0,l*0.04]]) + center
        d_list.append(d)
    d = np.vstack(d_list)
    return d

def move_structure(center,block_points,height = -0.08):
    x_center = 0
    y_center = 0
    for i in block_points:
        x_center = x_center + i[0]
        y_center = y_center + i[1]
    x_center = x_center/len(block_points)
    y_center = y_center/len(block_points)

    new_block_points = [((x - x_center + center[0],y - y_center+center[1],height),theta) for x,y,theta in block_points]
    return new_block_points


#Hacky exampe that was used to test an old version of the code. 
#TODO CLEANUP
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
        destination = make_destination(np.array([0.30,0.17,0]),levels=2)
        pos_list = []
        rospy.loginfo(destination)
        for s,d in zip(pw,destination):
            pos_list.extend(make_trajectory(s,d))
        rospy.loginfo(pos_list)
    else:
        pos_list = []
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
    rospy.sleep(0.5)
    arm_controller.home_arm()

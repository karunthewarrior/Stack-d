import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState
import kinematics as kin
from std_msgs.msg import Empty,Float64

class ArmController():
    def __init__(self):
        # Topics to control arm joints
        self.goal_topic = '/goal_dynamixel_position'
        self.joint_state_topic = '/joint_states'
        self.gripper_open_topic = '/gripper/open'
        self.gripper_close_topic = '/gripper/close'

        self.joint_target = np.zeros(5)
        self.pub = rospy.Publisher(self.goal_topic,JointState, queue_size=1)
        self.open_pub = rospy.Publisher(self.gripper_open_topic,Empty, queue_size=1)
        self.close_pub = rospy.Publisher(self.gripper_close_topic,Empty, queue_size=1)
        self.sub = rospy.Subscriber(self.joint_state_topic,JointState,self.get_joint_state)
        self.history = []

        #Pausing so that the subscriber callback initializes the joint_state variable
        rospy.sleep(0.5)

#Publisher to set the target joint angles
    def set_joint_state(self,joint_target):
        self.joint_target = joint_target
        joint_state = JointState()
        joint_state.position = tuple(joint_target)
        self.pub.publish(joint_state)

#Moves the arm to the home configuration
    def home_arm(self):
        rospy.loginfo('Going to arm home pose')
        self.set_joint_state(np.zeros(5))
        # rospy.sleep(5)
        while(not self.has_converged()):
            pass

#Opens the gripper when called
    def open(self):
        empty_msg = Empty()
        self.open_pub.publish(empty_msg)
        rospy.sleep(0.5)

#Closes the gripper when called
    def close(self):
        empty_msg = Empty()
        self.close_pub.publish(empty_msg)   
        rospy.sleep(0.5)

#Subscriber callback to get current joint states
    def get_joint_state(self,joint_state):
        self.time = joint_state.header.stamp
        self.joint_state = np.array(joint_state.position)[0:5]

    def has_converged(self):
        converged = False
        # print(abs(self.joint_state-self.joint_target) < 0.0174533)
        if(np.all(abs(self.joint_state-self.joint_target) < 0.0174533)):
            self.history.append(self.time)
            # rospy.loginfo((self.time - self.history[0]).to_sec())
            if (self.time - self.history[0]).to_sec() > 1:
                converged = True
        else:
            self.history = []   
        return converged

    def has_converged_relaxed(self):
        converged = False
        self.converged_joints = np.array((abs(self.joint_state[:3].reshape(-1,1)-self.joint_target[:3].reshape(-1,1)) < 3*0.0174533,abs(self.joint_state[3:5].reshape(-1,1)-self.joint_target[3:5].reshape(-1,1))<0.0174533))
        if(np.all(abs(self.joint_state[:3].reshape(-1,1)-self.joint_target[:3].reshape(-1,1)) < 3*0.0174533) and np.all(abs(self.joint_state[3:5].reshape(-1,1)-self.joint_target[3:5].reshape(-1,1))<0.0174533)):
            self.history.append(self.time)
            if (self.time - self.history[0]).to_sec() > 1:
                converged = True
                print("converged")
        else:
            self.history = []   
        return converged

class CamController():
    def __init__(self,cam_state_topic,cam_goal_topic):
        #Topics to control cam joint
        self.cam_goal_topic = cam_goal_topic
        self.cam_state_topic = cam_state_topic
        self.cam_target = 0
        self.pub = rospy.Publisher(self.cam_goal_topic,Float64, queue_size=1)
        self.sub = rospy.Subscriber(self.cam_state_topic,Float64,self.get_cam_state)
        self.history = []
        rospy.sleep(0.5)

#Publisher to set the target angle
    def set_cam_state(self,cam_target):
        self.cam_target = cam_target
        cam_state = Float64()
        cam_state.data = cam_target
        self.pub.publish(cam_state)

#Sets the cam angle to zero
    def home_arm(self):
        self.set_joint_state(0)
        # rospy.sleep(5)
        while(not self.has_converged()):
            pass

#Subscriber callback to get the current cam angle
    def get_cam_state(self,cam_state):
        self.time = rospy.get_rostime()
        self.cam_state = cam_state.data

#Checks if the angle has converged to the target
    def has_converged(self):
        converged = False
        if(abs(self.cam_state-self.cam_target) < 0.1):
            self.history.append(self.time)
            # rospy.loginfo((self.time - self.history[0]).to_sec())
            if (self.time - self.history[0]).to_sec() > 0.5:
                converged = True
        else:
            self.history = []
        return converged

if __name__ == "__main__":
    #Initializing rospy and initializing the arm and camera objects
    rospy.init_node('controller_test', anonymous=True)
    arm_controller = ArmController()
    tilt_controller = CamController('/tilt/state','/tilt/command')
    pan_controllelr = CamController('/pan/state','/pan/command')
    target_pan = np.deg2rad([0,20,-20,0])
    target_tilt = np.deg2rad([0,20,-20,0])
    # rospy.sleep(2)
    # target_joints = [[0,0,0,0,0],[0,10,20,0,0]]
    target_joints = []
    pos_list = [[ 0.3, 0.1, 0]]
    # pos_list = [[]] 
    # for tilt in target_tilt:
    #     tilt_controller.set_cam_state(tilt)
    #     while(not tilt_controller.has_converged()):
    #         pass
    # for pan in target_pan:
    #     pan_controller.set_cam_state(pan)
    #     while(not pan_controller.has_converged()):
    #         pass

    for pos in pos_list:
        q = kin.inverse_kinematics(pos,np.deg2rad(45))
        if q!=None:
            target_joints.append(q)
        else:
            print("No solution")
            exit()
    rospy.sleep(2)

    # target_joints = [[np.deg2rad(20),0,0,0,0]]
    # print(target_joints)
    arm_controller.home_arm()
    arm_controller.open()
    for joint in target_joints:
        arm_controller.set_joint_state(joint)
        while(not arm_controller.has_converged()):
            pass
    arm_controller.close()
    # controller.home_arm()

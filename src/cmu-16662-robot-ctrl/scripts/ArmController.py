import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState
import kinematics as kin
from std_msgs.msg import Float64MultiArray

class ArmController():
    def __init__(self):
        self.goal_topic = '/goal_dynamixel_position'
        self.joint_state_topic = '/joint_states'
        self.joint_target = np.zeros(5)
        self.pub = rospy.Publisher(self.goal_topic,JointState, queue_size=1)
        self.sub = rospy.Subscriber(self.joint_state_topic,JointState,self.get_joint_state)
        self.history = []

    def set_joint_state(self,joint_target):
        self.joint_target = joint_target
        joint_state = JointState()
        joint_state.position = tuple(joint_target)
        self.pub.publish(joint_state)

    def home_arm(self):
        rospy.loginfo('Going to arm home pose')
        self.set_joint_state(np.zeros(6))
        # rospy.sleep(5)
        while(not controller.has_converged()):
            pass

    def get_joint_state(self,joint_state):
        self.time = joint_state.header.stamp
        self.joint_state = np.array(joint_state.position)[0:6]

    def has_converged(self):
        converged = False
        # rospy.loginfo(np.linalg.norm(self.joint_state[5]-self.joint_target[5]))
        if(np.linalg.norm(self.joint_state[:5]-self.joint_target[:5]) < 0.1 and np.linalg.norm(self.joint_state[5]-self.joint_target[5]) < 0.1):
            self.history.append(self.time)
            # rospy.loginfo((self.time - self.history[0]).to_sec())
            if (self.time - self.history[0]).to_sec() > 0.5:
                converged = True
        else:
            self.history = []   
        return converged


class tag_pos():
    def __init__(self):
        self.marker_topic = '/marker_pos'
        self.sub = rospy.Subscriber(self.marker_topic,Float64MultiArray,self.get_position)

    def get_position(self,tag):
        self.tag_pos = np.array(tag.data)
        rospy.loginfo(self.tag_pos)


if __name__ == "__main__":  
    rospy.init_node('controller_test', anonymous=True)
    controller = ArmController()
    tag_position = tag_pos()
    rospy.sleep(2)
    block_pos = np.array(tag_position.tag_pos) + np.array([0.05,0,0.15])
    # target_joints = np.deg2rad([[0,0,0,0,0,-50],[0,10,20,0,0,0]])
    # target_joints = [[0,0,0,0,-0.87],[0,0,0,0,-0.3]]
    target_joints = []
    # pos_list = [[0.3,0,0.1],[0.3,0,-0.055],[0.3,0,-0.055],[0.3,0,0.1],[0.25,0.25,0.1],[0.25,0.25,0],[0.25,0.25,-0.03]]
    pos_list = [block_pos,block_pos-np.array([0,0,0.1]),block_pos-np.array([0,0,0.1]),block_pos]
    gripper_list = [True,True,False,False]
    # pos_list = [[0.25,0.2,0.1]]
    # gripper_list = [True,True,False,False,False,False,True]
    for pos,grip in zip(pos_list,gripper_list):
        q = kin.inverse_kinematics(pos,grip)
        # print(kin.forward_kinematics(q)[0]["joint_4"])
        if q!=None:
            target_joints.append(q)
        else:
            rospy.loginfo("No solution")
    # rospy.sleep(2)
    # print(target_joints)
    # controller.home_arm()
    rospy.loginfo(target_joints)
    for joint in target_joints:
        controller.set_joint_state(joint)
        while(not controller.has_converged()):
            # rospy.loginfo("not")
            pass
        # rospy.sleep(4)
# controller.home_arm()
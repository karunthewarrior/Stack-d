import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState
import kinematics as kin

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
        self.set_joint_state(np.zeros(5))
        # rospy.sleep(5)
        while(not controller.has_converged()):
            pass

    def get_joint_state(self,joint_state):
        self.time = joint_state.header.stamp
        self.joint_state = np.array(joint_state.position)[0:5]

    def has_converged(self):
        converged = False
        if(np.linalg.norm(self.joint_state-self.joint_target) < 0.1):
            self.history.append(self.time)
            # rospy.loginfo((self.time - self.history[0]).to_sec())
            if (self.time - self.history[0]).to_sec() > 0.5:
                converged = True
        else:
            self.history = []   
        return converged

if __name__ == "__main__":  
    rospy.init_node('controller_test', anonymous=True)
    controller = ArmController()
    # target_joints = [[0,0,0,90,0]]
    # target_joints = [[0,0,0,0,0],[0,10,0,0,0],[0,20,30,0,0]]
    target_joints = np.deg2rad([[0,-10,20,-70,0],[0,-10,40,-70,0],[10,-20,30,-80,0]])
    # pos_list = [[0.3,0,0.1],[0.3,0.1,0.15],[0.25,0.1,0.25]]
    # pos_list = []
    # target_joints = [[0.48056895, 0.76073892, 0.02609777, 1.5390177,  0.79548688]]
    # for pos in pos_list:
    #     q = kin.inverse_kinematics(pos)
    #     if q!=None:
    #         target_joints.append(q)
    #     else:
    #         print("No solution")
    #         exit()
    rospy.sleep(2)
    # controller.home_arm()
    for joint in target_joints:
        controller.set_joint_state(joint)
        while(not controller.has_converged()):
            # rospy.loginfo("not")
            pass

    # controller.home_arm()

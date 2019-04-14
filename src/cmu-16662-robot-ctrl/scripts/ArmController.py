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
        # Used to plot response of the controller
        self.goal = []
        self.state = []
        #Pausing so that the subscriber callback initializes the joint_state variable
        rospy.sleep(1)

#Publisher to set the target joint angles
    def set_joint_state(self,joint_target):
        self.joint_target = joint_target
        joint_state = JointState()
        joint_state.position = tuple(joint_target)
        self.pub.publish(joint_state)
        while(not self.has_converged()):
            if rospy.is_shutdown():
                break

#Moves the arm to the home configuration
    def home_arm(self):
        rospy.loginfo('Going to arm home pose')
        self.set_joint_state(np.zeros(5))
        while(not self.has_converged()):
            if rospy.is_shutdown():
                break

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
        #Bookkeeping to plot the controller response
        self.goal.append(self.joint_target)
        self.state.append(self.joint_state)


"""
Checks if the joint angles have converged to the target
Output: Boolean - True if converged
"""
    def has_converged(self):
        converged = False
        if(np.linalg.norm(self.joint_state-self.joint_target) < 0.1):
            self.history.append(self.time)
            if (self.time - self.history[0]).to_sec() > 0.5:
                converged = True
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

        rospy.sleep(0.5)

#Publisher to set the target angle
    def set_cam_state(self,cam_target):
        self.cam_target = cam_target
        cam_state = Float64()
        cam_state.data = cam_target
        self.pub.publish(cam_state)

#Sets the cam angle to zero
    def home_cam(self):
        self.set_joint_state(0)
        while(not self.has_converged()):
            if rospy.is_shutdown():
                break

#Subscriber callback to get the current cam angle
    def get_cam_state(self,cam_state):
        self.time = rospy.get_rostime()
        self.cam_state = cam_state.data

#Checks if the angle has converged to the target.
    def has_converged(self):
        converged = False
        if(abs(self.cam_state-self.cam_target) < 0.1):
            self.history.append(self.time)
            if (self.time - self.history[0]).to_sec() > 0.1:
                converged = True
        else:
            self.history = []
        return converged

if __name__ == "__main__":  
    #Initializing rospy and initializing the arm and camera objects
    rospy.init_node('controller_test', anonymous=True)
    arm_controller = ArmController()
    tilt_controller = CamController('/tilt/state','/tilt/command')
    pan_controller = CamController('/pan/state','/pan/command')

    target_joints = []
    # pos_list = [[0.3,0.1,0],[0.3,0.15,0],[0.3,-0.1,0]]
    pos_list = [[0.3,0,0.1],[0.3,0,-0.055],[0.3,0,-0.055],[0.3,0,0.1],[0.25,0.25,0.1],[0.25,0.25,0],[0.25,0.25,-0.03],[0.25,0.25,-0.03]]
    
    for pos in pos_list:
        q = kin.inverse_kinematics(pos)
        print(kin.forward_kinematics(q)[0]["joint_4"])
        if q!=None:
            target_joints.append(q)
        else:
            print("No solution")
            exit()
    rospy.sleep(2)

    for joint in target_joints:
        arm_controller.set_joint_state(joint)
    arm_controller.home_arm()
    # pickle.dump(arm_controller.goal,open('goal_states.p','wb'))
    # pickle.dump(arm_controller.state,open('joint_states.p','wb'))


import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pprint import pprint
import ArmController as ac
import kinematics as kin
from std_msgs.msg import Float64MultiArray

class Point_detection():
    def __init__(self):
        rospy.init_node("Point_detection", anonymous=True)
        rospy.Subscriber("pixel_error",Float64MultiArray, self.get_error)
        rospy.sleep(1)
    
    def get_error(self, error_pixel):
        self.error_pixel = np.array(error_pixel.data).reshape(-1,1)

def compute_joint_angles(error_pixel,angles):
    q = np.array(angles).reshape(-1,1)
    alpha = 1e-3
    H = kin.webcam_to_world(angles)
    error_world  = np.dot(H[:3,:3],error_pixel[:3])
    final_pos,fk_list = kin.forward_kinematics(angles)
    J = kin.jacobian(fk_list)
    delta_q = alpha * np.dot(J.T,error_world)
    print(final_pos,"before")
    print(q,"before")
    q[:3] = q[:3] + delta_q
    print(q,"after")
    final_pos,_ = kin.forward_kinematics(q)
    # q[3] = np.pi/2 - final_pos["joint_4"][4] 
    # q[4] = -q[0]
    print(final_pos,"before")
    return q

if __name__ == '__main__':
    try:
        servoing = Point_detection()
        arm_controller = ac.ArmController()

        pos = [0.2,0,-0.06]
        q = kin.inverse_kinematics(pos,np.deg2rad(0))
        # arm_controller.home_arm()
        arm_controller.set_joint_state(q)
        while(not arm_controller.has_converged()):
            pass    
        # print(compute_joint_angles(np.array([0,30,100,1]).reshape(-1,1),[0,np.deg2rad(45),np.deg2rad(20),np.deg2rad(30),0]))
        while not rospy.is_shutdown():
            angles = arm_controller.joint_state
            q = compute_joint_angles(servoing.error_pixel,angles)
            rospy.loginfo(q)
            arm_controller.set_joint_state(q)
            # while(not arm_controller.has_converged()):
            #     print(q)
            #     pass
            if (np.all(np.abs(servoing.error_pixel<10))):
                break 

    except rospy.ROSInterruptException:
        pass
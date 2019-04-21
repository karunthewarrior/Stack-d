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
    print(final_pos["joint_4"],"before")
    J = kin.jacobian(fk_list)
    delta_q = alpha * np.dot(J.T,error_world)
    print(delta_q.shape)
    q[:3] = q[:3] + delta_q
    final_pos,_ = kin.forward_kinematics(q)
    print(final_pos["joint_4"],"after")
    q[4] = -q[0]
    return q

if __name__ == '__main__':
    try:
        servoing = Point_detection()
        arm_controller = ac.ArmController()

        pos = [0.25,-0.1,-0.06]
        q = kin.inverse_kinematics(pos,np.deg2rad(0))
        # arm_controller.home_arm()
        arm_controller.set_joint_state(q)
        while(not arm_controller.has_converged()):
            pass    
        while not rospy.is_shutdown():
            angles = arm_controller.joint_state

            q = compute_joint_angles(servoing.error_pixel,angles)
            final_pos,_ = kin.forward_kinematics(q)
            # q[3] = np.pi/2 - final_pos["joint_4"][4] 
            rospy.loginfo(q)
            arm_controller.set_joint_state(q)
            # while(not arm_controller.has_converged()):
            #     pass
            # rospy.sleep(5)  

            if (np.all(np.abs(servoing.error_pixel<10))):
                break 

    except rospy.ROSInterruptException:
        pass
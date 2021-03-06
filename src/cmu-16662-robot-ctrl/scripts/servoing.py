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
        rospy.Subscriber("pixel_error",Float64MultiArray, self.get_error)
        rospy.sleep(1)
    
    def get_error(self, error_pixel):
        self.error_pixel = np.array(error_pixel.data).reshape(-1,1)

def compute_joint_angles(error,angles,alpha=1e-3,pixel=True):
    q = np.array(angles).reshape(-1,1)
    H = kin.webcam_to_world(angles)
    if pixel:
        error_world  = np.dot(H[:3,:3],error[:3])
    else:
        error_world = error
    final_pos,fk_list = kin.forward_kinematics(angles)
    J = kin.jacobian(fk_list)
    delta_q = alpha * np.dot(J.T,error_world)
    q[:3] = q[:3] + delta_q
    final_pos,_ = kin.forward_kinematics(q)
    q[4] = -q[0]
    q[3] = np.pi/2 - final_pos["joint_3"][4]
    return q

def servo_xy(arm_controller,servo,error_thresh = 30):
    if not np.all(servo.error_pixel[:3] == 0):
        while np.any(np.abs(servo.error_pixel) > error_thresh):
            angles = arm_controller.joint_state
            q = compute_joint_angles(servo.error_pixel,angles)
            arm_controller.set_joint_state(q)
        pose,fk_list = kin.forward_kinematics(q)
        x,y = pose["joint_4"][:2]
        return (x,y)
    return None

def servo_z(arm_controller,servo,mode='down'):
    angles = arm_controller.joint_state
    pose,fk_list = kin.forward_kinematics(angles)
    current_pos = pose["joint_4"][0:3]
    if mode == 'down':
        end_pos = current_pos + np.array([0,0,-0.1])
    else:
        end_pos = current_pos - np.array([0,0,-0.1])
    error_list = [0.1]
    while(1):
        angles = arm_controller.joint_state
        pose,fk_list = kin.forward_kinematics(angles)
        current_pos = pose["joint_4"][0:3]
        error = (end_pos - current_pos).reshape(-1,1)
        if abs(error[-1] - error_list[-1])>1e-4:
            error_list.append(error[-1])
            print(error[-1],"ERROR")
        q = compute_joint_angles(error,angles,alpha=1.5,pixel=False)
        arm_controller.set_joint_state(q)
        if mode == 'down':
            if len(error_list) > 10 :
                if np.all(abs(np.array(error_list[-1])-np.array(error_list[-4:-2])) < 0.002):
                    print("Converged")
                    break
        elif mode == 'up':
            if len(error_list) > 5:
                break


if __name__ == '__main__':
    try:
        rospy.init_node("Servoing", anonymous=True)
        servo = Point_detection()
        arm_controller = ac.ArmController()
        pos = [0.25,-0.1,-0.06]
        q = kin.inverse_kinematics(pos,np.deg2rad(0))
        arm_controller.set_joint_state(q)
        while(not arm_controller.has_converged()):
            pass
        arm_controller.open()    
        print("seroving now")
        print(servo_xy(arm_controller,servo))
        servo_z(arm_controller,servo)
        arm_controller.close()
        rospy.sleep(1)
        servo_z(arm_controller,servo,'up')
    except rospy.ROSInterruptException:
        pass
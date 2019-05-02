import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pprint import pprint
import ArmController as ac
import kinematics as kin
from std_msgs.msg import Float64MultiArray,Float32


class Point_detection():
    def __init__(self):
        rospy.Subscriber("pixel_error",Float64MultiArray, self.get_error)
        rospy.Subscriber("block_yaw",Float32,self.get_yaw)
        rospy.sleep(1)
    
    def get_error(self, error_pixel):
        self.error_pixel = np.array(error_pixel.data).reshape(-1,1)

    def get_yaw(self,yaw):
        self.yaw = yaw.data

def compute_joint_angles(error,angles,alpha=5e-3,pixel=True,yaw=0):
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
    q[4] = -q[0] + yaw
    q[3] = np.pi/2 - final_pos["joint_3"][4]
    return q

# def servo_xy(arm_controller,servo,error_thresh = 10):
#     if not np.all(servo.error_pixel[:3] == 0):
#         q = arm_controller.joint_state
#         while np.any(np.abs(servo.error_pixel) > error_thresh):
#             print(servo.error_pixel,"errorpix")
#             angles = arm_controller.joint_state
#             q = compute_joint_angles(servo.error_pixel,angles)
#             # print(q,"Qs")
#             arm_controller.set_joint_state(q)
#             rospy.sleep(0.5)
#         pose,fk_list = kin.forward_kinematics(q)
#         x,y = pose["joint_4"][:2]
#         return (x,y)
#     return None

def servo_xy(arm_controller,servo,error_thresh = 8,alpha=5e-3):

    if not np.all(servo.error_pixel[:3] == 0):
        q = arm_controller.joint_state
        timer = rospy.get_rostime().to_sec()
        timer_current = timer
        while np.any(np.abs(servo.error_pixel) > error_thresh) or (timer_current - timer < 1):
            # print(np.any(np.abs(servo.error_pixel) > error_thresh))
            # print(servo.error_pixel,"errorpix")
            if np.all(np.abs(servo.error_pixel) < error_thresh):
                timer_current = rospy.get_rostime().to_sec()
            else:
                angles = arm_controller.joint_state
                q = compute_joint_angles(servo.error_pixel,angles,alpha=alpha)
                arm_controller.set_joint_state(q)
                rospy.sleep(0.5)
        print(servo.error_pixel,"FINAL ERROR")
        pose,fk_list = kin.forward_kinematics(q)
        x,y = pose["joint_4"][:2]
        theta = servo.yaw
        return (x,y,theta)
    return None

def servo_z(arm_controller,servo,mode='down',yaw=0):
    angles = arm_controller.joint_state
    pose,fk_list = kin.forward_kinematics(angles)
    current_pos = pose["joint_4"][0:3]
    if mode == 'down':
        end_pos = current_pos + np.array([0,0,-0.1])
    else:
        end_pos = current_pos - np.array([0,0,-0.1])
    angles = arm_controller.joint_state
    pose,fk_list = kin.forward_kinematics(angles)
    current_pos = pose["joint_4"][0:3]
    error = (end_pos - current_pos).reshape(-1,1)
    q = compute_joint_angles(error,angles,alpha=1,pixel=False,yaw=yaw)
    arm_controller.set_joint_state(q)
    rospy.sleep(1)


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
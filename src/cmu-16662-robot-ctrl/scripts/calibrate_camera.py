import sys
import numpy as np
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import kinematics as kin
import ArmController as controller
import pickle
import cv2
from transformations import euler_matrix, translation_matrix, quaternion_about_axis, quaternion_matrix,euler_from_matrix

class camera_calib:
    def __init__(self):
        self.camera_pose_topic = "/ar_pose_marker"
        self.ar_position = np.array([None,None,None])
        sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_camera_pose)

    def get_camera_pose(self,camera_pose):
        self.time = camera_pose.header.stamp
        try:
            ar_position_obj = camera_pose.markers[0].pose.pose.position
            ar_orient_obj = camera_pose.markers[0].pose.pose.orientation
            self.ar_position = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z])
            self.ar_orient = np.array([ar_orient_obj.x,ar_orient_obj.y,ar_orient_obj.z,ar_orient_obj.w])
        except:
            self.ar_position = np.array([None,None,None])

def invRodrigues(R):
    epsilon = 1e-16
    theta = np.arccos((np.trace(R)-1)/2.0)
    r =  np.zeros((3, 1))
    if abs(theta) > epsilon:
        rx = theta/(2*np.sin(theta))*(R - R.T)
        r = [rx[2,1],rx[0,2],rx[1,0]]
    return np.array(r)

def compute_transformation(arm_H_list,cam_H_list):
    points = np.array([[0.014,0.014,0,1],[-0.014,0.014,0,1],[0.014,-0.014,0,1],[-0.014,-0.014,0,1],[0,0,0,1]]).T
    # points = np.array([0,0,0,1]).T
    arm_pts = [np.dot(H,points).T for H in arm_H_list]
    cam_pts = [np.dot(H,points).T for H in cam_H_list]
    # print(arm_pts)
    # for k in cam_pts:
    #     print(k)
    arm_pts = np.vstack((arm_pts))
    cam_pts = np.vstack((cam_pts))
    K = np.array([[619.1472778320312, 0.0, 313.42169189453125,0],[ 0.0, 619.0415649414062, 242.69955444335938,0],[ 0.0, 0.0, 1.0,0]])
    pixel_location = np.dot(K,cam_pts.T)
    pixel_location = pixel_location/pixel_location[-1,:]
    pixel_location = pixel_location[0:2,:]
    pixel_location = pixel_location.T
    arm_pts = arm_pts[:,0:3]
    distcoeff = None
    H_1_guess = np.dot(arm_H_list[0],np.linalg.inv(cam_H_list[0]))
    rot_guess_rod = invRodrigues(H_1_guess[0:3,0:3])
    trans_guess = H_1_guess[0:3,3]
    print(trans_guess)
    success,rvec,tvec,inliers = cv2.solvePnPRansac(arm_pts,pixel_location,K[:3,:3],distcoeff,iterationsCount=10000,useExtrinsicGuess=True,rvec=rot_guess_rod,tvec=np.array(trans_guess))
    print(tvec)
    rot = cv2.Rodrigues(rvec)[0]
    H_rot = np.vstack((rot,np.array([[0,0,0]])))
    H_rot = np.hstack((H_rot,np.array([[0,0,0,1]]).T))
    H_trans = translation_matrix((tvec[0][0],tvec[1][0],tvec[2][0]))
    result = np.linalg.inv(np.dot(H_trans,H_rot))
    # print(result)

if __name__ == "__main__":
    rospy.init_node("camera_calib", anonymous=True)
    cam = camera_calib()
    controller = controller.ArmController()
    rospy.sleep(2)
    target_joints = np.deg2rad([[0,-20,20,-70,0],[0,-20,20,-80,0],[10,-20,30,-80,0],[-20,-20,30,-80,0],[-20,20,30,-80,0],[20,20,20,-80,0],[20,20,0,-70,0],[20,15,0,-70,0],[20,20,0,-80,0],[30,10,0,-50,0],[30,10,-20,-50,0],[0,10,-20,-50,0],[0,10,-10,-50,0],[-20,20,30,-80,0]])
    arm_H_list = []
    cam_H_list = []
    for joint in target_joints:
        controller.set_joint_state(joint)
        while(not controller.has_converged()):
            pass
        rospy.sleep(0.5)
        cam_pos = cam.ar_position
        cam_orient = cam.ar_orient
        print(euler_from_matrix(quaternion_matrix(cam_orient)),'rzyx')
        H = np.dot(translation_matrix(cam_pos),quaternion_matrix(cam_orient))
        arm_H = np.dot(kin.forward_kinematics(joint)[1][-1],translation_matrix((0.029,-0.001,0.043)))
        # rospy.loginfo(rospy.get_time())
        rospy.loginfo(cam.time)
        arm_H_list.append(arm_H)
        cam_H_list.append(H)
        rospy.sleep(0.5)
    pickle.dump(arm_H_list, open("arm_H_pos.p","wb"))
    pickle.dump(cam_H_list, open("cam_H_pos.p","wb"))
    arm_H_list = pickle.load(open("arm_H_pos.p","rb"))
    cam_H_list = pickle.load(open("cam_H_pos.p","rb"))
    H = compute_transformation(arm_H_list,cam_H_list)




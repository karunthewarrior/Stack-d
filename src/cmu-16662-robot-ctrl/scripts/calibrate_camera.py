import sys
import numpy as np
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import kinematics as kin
import ArmController as controller
import pickle
import cv2
from transformations import euler_matrix, translation_matrix, quaternion_about_axis, quaternion_matrix,euler_from_matrix
import math
import tf

#create class for camera calibration
class camera_calib:
    def __init__(self):

        self.camera_pose_topic = "/ar_pose_marker"
        self.ar_position = np.array([None,None,None])
        self.ar_orient = np.array([None,None,None,None])
        sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_camera_pose)
        self.listener = tf.TransformListener()

    def get_camera_pose(self,camera_pose):
        #get ar tag pose
        self.time = camera_pose.header.stamp
        try:
            (self.ar_position,self.ar_orient) = self.listener.lookupTransform('/camera_color_optical_frame','/ar_marker_1', rospy.Time(0))
        except:
            self.ar_position = np.array([None,None,None])
        rospy.loginfo(self.ar_position)
        rospy.loginfo(self.ar_orient)


def eulerAnglesToRotationMatrix(theta) :
    #get rotation matrix from euler angles with "rzyx" convention
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                                          
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

def invRodrigues(R):
    #calculate rodrigues rotation from rotation matrix 
    epsilon = 1e-16
    theta = np.arccos((np.trace(R)-1)/2.0)
    r =  np.zeros((3, 1))
    if abs(theta) > epsilon:
        rx = theta/(2*np.sin(theta))*(R - R.T)
        r = [rx[2,1],rx[0,2],rx[1,0]]
    return np.array(r)

def compute_transformation(arm_H_list,cam_H_list):
    #define the 4 corners and center of ar tag in ar tag frame
    points = np.array([[0.014,0.014,0,1],[-0.014,0.014,0,1],[0.014,-0.014,0,1],[-0.014,-0.014,0,1],[0,0,0,1]]).T
    # points = np.array([[0,0,0,1]]).T
    
    #transform corner points in world and camera fram 
    arm_pts = [np.dot(H,points).T for H in arm_H_list]
    cam_pts = [np.dot(H,points).T for H in cam_H_list]
    arm_pts = np.vstack((arm_pts))
    cam_pts = np.vstack((cam_pts))

    #transform points in camera frame to pixel locations
    K = np.array([[619.1472778320312, 0.0, 313.42169189453125,0],[ 0.0, 619.0415649414062, 242.69955444335938,0],[ 0.0, 0.0, 1.0,0]])
    pixel_location = np.dot(K,cam_pts.T)
    pixel_location = pixel_location/pixel_location[-1,:]
    pixel_location = pixel_location[0:2,:]
    pixel_location = pixel_location.T
    arm_pts = arm_pts[:,0:3]

    #create initial guess based on FK and AR tag in first configuration
    H_1_guess = np.linalg.inv(np.dot(arm_H_list[0],np.linalg.inv(cam_H_list[0])))
    rot_guess_rod = invRodrigues(H_1_guess[0:3,0:3])
    trans_guess = H_1_guess[0:3,3]

    #find transform of world in camera frame frame
    distcoeff = None
    success,rvec,tvec,inliers = cv2.solvePnPRansac(arm_pts,pixel_location,K[:3,:3],distcoeff,iterationsCount=100000,useExtrinsicGuess=True,rvec=rot_guess_rod,tvec=np.array(trans_guess))
    
    #find transform of camera in world frame 
    rot = cv2.Rodrigues(rvec)[0]
    H = np.eye(4)
    H[:3,:3] = rot
    H[:3,3] = tvec
    H_world_to_camera = np.linalg.inv(H)  

    #print results
    print("camera frame with in world frame:",H_world_to_camera)

if __name__ == "__main__":
    #set up node and classes for calibration
    rospy.init_node("camera_calib", anonymous=True)
    cam = camera_calib()
    arm_controller = controller.ArmController()
    tilt_controller = controller.CamController('/tilt/state','/tilt/command')
    pan_controller = controller.CamController('/pan/state','/pan/command')
    rospy.sleep(2)

    tilt_controller.set_cam_state(np.deg2rad(-20))
    while(not tilt_controller.has_converged()):
        pass

    pan_controller.set_cam_state(np.deg2rad(0))
    while(not pan_controller.has_converged()):
        pass

    #create list of target joints for calibration and loop through them
    target_joints = np.deg2rad([[0,-20,20,-70,0],[0,-20,20,-80,0],[10,-20,30,-80,0],[-20,-20,30,-80,0],[-20,20,30,-80,0],[20,20,20,-80,0],[20,20,0,-70,0],[20,15,0,-70,0],[20,20,0,-80,0],[30,10,0,-50,0],[30,10,-20,-50,0],[0,10,-20,-50,0],[0,10,-10,-50,0],[-20,20,30,-80,0]])
    # target_joints = np.hstack((target_joints,np.zeros((target_joints.shape[0],1))))
    arm_H_list = []
    cam_H_list = []
    for joint in target_joints:
        arm_controller.set_joint_state(joint)
        while(not arm_controller.has_converged()):
            pass
        rospy.sleep(0.5)

        #get position and orientation from AR tag in camera frame
        cam_pos = cam.ar_position
        cam_orient = cam.ar_orient

        #create Homogenious matrix of AR tag in camera frame and append in list
        cam_orient_rot= eulerAnglesToRotationMatrix(tf.transformations.euler_from_quaternion(cam_orient))
        cam_H = np.eye(4)
        cam_H[:3,:3] = cam_orient_rot
        cam_H[:3,3] = cam_pos
        cam_H_list.append(cam_H)

        #compute Homogenious matrix of ar tag in world frame and append in list
        arm_H = np.dot(kin.forward_kinematics(joint)[1][-1],translation_matrix((0.029,-0.001,0.043)))
        arm_H_list.append(arm_H)
        rospy.sleep(0.5)

    #save homogenious matrices in pickle file
    pickle.dump(arm_H_list, open("arm_H_pos.p","wb"))
    pickle.dump(cam_H_list, open("cam_H_pos.p","wb"))

    #load pickle files and compute transform
    arm_H_list = pickle.load(open("arm_H_pos.p","rb"))
    cam_H_list = pickle.load(open("cam_H_pos.p","rb"))
    H = compute_transformation(arm_H_list,cam_H_list)
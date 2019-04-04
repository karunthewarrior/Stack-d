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

class camera_calib:
    def __init__(self):
        self.camera_pose_topic = "/ar_pose_marker"
        self.ar_position = np.array([None,None,None])
        sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_camera_pose)
        self.listener = tf.TransformListener()

    def get_camera_pose(self,camera_pose):
        self.time = camera_pose.header.stamp
        try:
            (self.ar_position,self.ar_orient) = self.listener.lookupTransform('/camera_color_optical_frame','/ar_marker_6', rospy.Time(0))
        except:
            self.ar_position = np.array([None,None,None])

def eulerAnglesToRotationMatrix(theta) :
     
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


def compute_transformation(angle_head_pan,angle_head_tilt):
    tilt_offset = -1.6
    tilt_downward = -0.73
    angle_head_tilt = -(tilt_offset -tilt_downward)
    trans_world_pan = np.array([-0.098, 0.000, 0.420])
    rot_world_pan = eulerAnglesToRotationMatrix((0,0,angle_head_pan))
    H_world_to_pan = np.eye(4)
    H_world_to_pan[:3,:3] = rot_world_pan
    H_world_to_pan[0:3,-1] = trans_world_pan
    
    trans_pan_tilt = np.array([[0.000, 0.000, 0.050]])
    rot_pan_tilt = eulerAnglesToRotationMatrix((0,angle_head_tilt,0))
    H_pan_to_tilt = np.eye(4)
    H_pan_to_tilt[:3,:3] = rot_pan_tilt
    H_pan_to_tilt[0:3,-1] = trans_pan_tilt

    trans_tilt_camera = np.array([0.067, 0.000, -0.004])
    H_tilt_to_camera = np.eye(4)
    H_tilt_to_camera[0:3,-1] = trans_tilt_camera

    H_world_to_camera_link = np.dot(np.dot(H_world_to_pan,H_pan_to_tilt),H_tilt_to_camera)

    trans_camera_link_to_color = np.array([-0.000, 0.015, 0.000])
    H_camera_link_to_color = np.eye(4)
    H_camera_link_to_color[0:3,-1] = trans_camera_link_to_color

    H_world_to_color = np.dot(H_world_to_camera_link,H_camera_link_to_color)

    rot_color_to_optical = eulerAnglesToRotationMatrix((-3.14/2,0,-3.14/2))
    H_color_to_optical = np.eye(4)
    H_color_to_optical[0:3,0:3] = rot_color_to_optical

    H_world_to_optical = np.dot(H_world_to_color,H_color_to_optical)
    
    return H_world_to_optical
if __name__ == "__main__":
    H = compute_transformation(0,0)



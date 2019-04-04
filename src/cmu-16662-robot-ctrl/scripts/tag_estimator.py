import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64MultiArray
import rospy
import math


class tag_estimator:
    def __init__(self):
        self.camera_pose_topic = "/ar_pose_marker"
        self.position_camera = np.array([0,0,0,1])
        self.transform = get_transform()
        self.world_point_topic = "/marker_pos"
        self.sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_point)
        self.pub = rospy.Publisher(self.world_point_topic,Float64MultiArray, queue_size=1)

    def get_point(self,tag):
        try:
            ar_position_obj = tag.markers[0].pose.pose.position
            self.position_camera = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z,1])
        except:
            self.position_camera = np.array([0,0,0,1])

        self.set_point()

    def set_point(self):
        pub_data=Float64MultiArray()
        point = np.dot(self.transform,self.position_camera)
        pub_data.data = point
        rospy.loginfo(pub_data)
        self.pub.publish(pub_data)

def get_transform():
    tilt_offset = -1.6
    tilt_downward = -0.73
    angle_head_tilt = -(tilt_offset -tilt_downward)
    angle_head_pan = 0
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

if __name__ =="__main__":
    rospy.init_node('marker_publisher', anonymous=True)
    estimator= tag_estimator()
    rospy.sleep(1)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     estimator.set_point()

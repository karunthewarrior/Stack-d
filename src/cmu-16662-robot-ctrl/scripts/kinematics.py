import numpy as np
import transforms3d as tf

"""
Computes the homogenous transformation
Input: orig - Relative translation from current frame to target frame
       axis - Axis about which to rotate
       angle - Angle of rotation about axis
Output: H - Homogenous transformation from current frame to target frame 
"""
def get_H(orig,axis,angle):
    orig = np.vstack([np.hstack([np.eye(3),np.array(orig).reshape(-1,1)]),[0,0,0,1]])
    axis = np.vstack([np.hstack([tf.axangles.axangle2mat(axis, angle),np.array([0, 0, 0]).reshape(-1,1)]),[0,0,0,1]])
    H = np.dot(orig,axis)
    return H 

"""
Computes the homogenous transformation for pure translation
Input: trans - Relative translation from current frame to target frame
Output: Homogenous transformation from current frame to target frame 
"""
def trans_H(trans):
    return np.vstack([np.hstack([np.eye(3),np.array(trans).reshape(-1,1)]),[0,0,0,1]])

"""
Computes the homogenous transformation for pure rotation
Input: R - Rotation matrix from current frame to target
Output: Homogenous transformation from current frame to target frame 
"""
def rot_H(R):
    return np.vstack([np.hstack([R,np.zeros(3).reshape(-1,1)]),[0,0,0,1]])

"""
Computes the homogenous transformation from the camera frame to the world frame (arm_base_link_joint)
Input: pan - pan motor angle
       tilt - tilt motor angle
Output: H_cam_to_world - Homogenous transformation from camera frame to world frame 
"""
def cam_to_world(pan,tilt):
    #Hardcoding the translations between the joints
    orig_list = np.array([[-0.0154999999999999, 0, 0.4112625],[0, 0, 0.05],[0.06705, 0.02, -0.00425]])
    #Hardcoding the axis of rotations of the joints
    axis_list = [[0,0,1],[0,-1,0],[0,0,1]] #Second axis is set to -1 since our setup has negative joint orientation to turn along the y-axis

    #Last angle (0) corresponds to a fixed joint
    angles = [pan,tilt,0]

    #Finding the relative transformation between each frame
    all_H = [get_H(origin,axis,angle) for origin,axis,angle in zip(orig_list,axis_list,angles)]


    cam_H = np.linalg.multi_dot(all_H)
    H_cam_to_world = np.linalg.inv(cam_H)
    return cam_H

"""
Computes forward kinematics of the arm
Input: angles - list of 
       tilt - tilt motor angle
Output: H_cam_to_world - Homogenous transformation from camera frame to world frame 
"""
def forward_kinematics(angles):
    orig_list = np.array([[0,0,0.072],[0, 0, 0.04125],[0.05, 0, 0.2],[0.2002, 0, 0],[0.193, 0, 0]])
    axis_list = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0]]
 
    all_H = [get_H(origin,axis,angle) for origin,axis,angle in zip(orig_list,axis_list,angles)]

    fk_list = [all_H[0]]
    for i in range(2,6):
        fk_list.append(np.linalg.multi_dot(all_H[0:i]))
    
    centroids = np.vstack([H[:3,-1] for H in fk_list]) 
    orientation = np.vstack([tf.euler.mat2euler(x[:3,:3],'rxyz') for x in fk_list])

    pose_mat = np.hstack([centroids,orientation])
    pose = {"joint_1":pose_mat[0],"joint_2":pose_mat[1],"joint_3":pose_mat[2],"joint_4":pose_mat[3],"gripper":pose_mat[4]}
    return pose,fk_list

def jacobian(fk_list):
    axis_list = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0]])
    jac = []
    for H,axis in zip(fk_list[:3],axis_list[:3]):  #only first three joints
        a = np.dot(H[:3,:3],axis.reshape(-1,1)).reshape(1,-1)
        p = (fk_list[-1][:3,-1] - H[:3,-1]).reshape(1,-1)
        jac_column = np.cross(a,p).reshape(-1,1)
        jac.append(jac_column)
    return np.hstack(jac)

def inverse_kinematics(target_pose,yaw,open_grip=True,max_iter=1000,offset=True):
    q = np.ones((5,1)) * np.pi/4
    gripper_offset = np.array([0, 0.015, 0.238])
    if offset:
        target_pose = target_pose + gripper_offset
    q[0] = -np.pi/4
    q[2] = np.pi/5
    q[3] = 0
    q[4] = 0
    x = np.zeros(3)
    dx = target_pose - x
    i = 0
    while(np.any(np.absolute(dx) > 1e-4)):
        if i == max_iter:
            print("timeout")
            return None
        i +=1
        _,fk_list = forward_kinematics(q)
        jac = jacobian(fk_list)
        dq = np.dot(np.linalg.pinv(jac),dx).reshape(-1,1)
        q[:3] = q[:3] + dq 
        final_pos,_ = forward_kinematics(q)
        dx = target_pose - final_pos["joint_4"][:3]
    q[3] = np.pi/2 - final_pos["joint_4"][4] 
    q[4] = -q[0] + yaw
    q = np.array([map_angle(a) for a in q])
    print(q,"Q",np.rad2deg(q))
    if (np.all(abs(q[:3]) >= 0) and np.all(abs(q[:3]) <= np.pi/2)):
        return list(q.reshape(-1,)) 
    else:
        return None
    
def map_angle(a):
    if a >=0:
        a = a % (2*np.pi)
    else:
        a = a % (-2*np.pi)
    if (a >= np.deg2rad(270) and a <= 2*np.pi):
        return float(a - 2*np.pi)
    elif (a <= -np.deg2rad(270) and a >= -2*np.pi):
        return float(-2*np.pi -a)
    else:
        return float(a)

if __name__ == "__main__":
    pos_list = [[0.3,0.02,0.4]]
    for pos in pos_list:
        q = inverse_kinematics(pos,0)
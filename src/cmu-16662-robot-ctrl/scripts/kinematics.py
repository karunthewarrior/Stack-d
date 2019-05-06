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
Output: cam_H - Homogenous transformation from camera frame to world frame 
"""
def cam_to_world(pan,tilt):
    orig_list = np.array([[-0.1154999999999999, 0, 0.4112625],[0, 0, 0.05],[0.06705, 0.033, 0]])
    axis_list = [[0,0,1],[0,-1,0],[0,0,1]]
    angles = [pan,tilt,0]
    all_H = [get_H(origin,axis,angle) for origin,axis,angle in zip(orig_list,axis_list,angles)]
    cam_H = np.linalg.multi_dot(all_H)
    H_cam_to_world = np.linalg.inv(cam_H)
    return cam_H

"""
Computes the homogenous transformation from the webcam frame to the world frame (arm_base_link_joint)
Input: angles - list of the 5 joint angles of the arm 
Output: webcam_H - Homogenous transformation from webcam frame to world frame 
"""
def webcam_to_world(angles):
    #Hardcoding the translations between the joints
    orig_list = np.array([[0,0,0.072],[0, 0, 0.04125],[0.05, 0, 0.2],[0.2002, 0, 0],[0.063, 0, 0],[0.027, 0, 0.043],[0.054,0,-0.0216]])
    #Hardcoding the axis of rotations of the joints
    axis_list = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0],[1,0,0],[1,0,0]]
    #Hardcoding the fixed joint angles
    angles = np.hstack([angles,np.array([0,0])])
    #Finding the relative transformation between each frame
    all_H = [get_H(origin,axis,angle) for origin,axis,angle in zip(orig_list,axis_list,angles)]
    webcam_H = np.linalg.multi_dot(all_H)
    return webcam_H

"""
Computes forward kinematics of the arm
Input: angles - list of the 5 joint angles of the arm 
Output: pose - Dictionary containing the 6DOF pose of each joint
        fk_list - List of homogenous transformation matrices that take you from the world frame to the frame at each joint in sequence.
"""
def forward_kinematics(angles):
    #Hardcoding the translations between the joints
    orig_list = np.array([[0,0,0.072],[0, 0, 0.04125],[0.05, 0, 0.2],[0.2002, 0, 0],[0.063, 0, 0]])
    #Hardcoding the axis of rotations of the joints
    axis_list = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0]]
    #Finding the relative transformation between each frame
    all_H = [get_H(origin,axis,angle) for origin,axis,angle in zip(orig_list,axis_list,angles)]

    #Multiplying the relative transformation to get the transformation until the correnponding joint angle
    fk_list = [all_H[0]]
    for i in range(2,6):
        fk_list.append(np.linalg.multi_dot(all_H[0:i]))
    
    """
    Extracting Pose from homogenous matrix and storing them in a dictionary. 
    Each value is accesed using the name of the joint as the Key.
    """
    centroids = np.vstack([H[:3,-1] for H in fk_list]) 
    orientation = np.vstack([tf.euler.mat2euler(x[:3,:3],'rxyz') for x in fk_list])

    pose_mat = np.hstack([centroids,orientation])
    pose = {"joint_1":pose_mat[0],"joint_2":pose_mat[1],"joint_3":pose_mat[2],"joint_4":pose_mat[3],"gripper":pose_mat[4]}
    return pose,fk_list

"""
Computes the jacobian corresponding to the 3D position of a point in task space (in our case position of joint_1 to joint_3).
Since we are only concerned with the position of the third joint, our jacobian is of shape 3x3.
Input: fk_list - List of homogenous transformations for each joint computed by the forward kinematics
Output: jacobian - 3x3 matrix relating the change in joint angle to change in 3D postion of each joint.
"""
def jacobian(fk_list,full=False):
    axis_list = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0]])
    jac = []
    if full:
        ind = 5
    else:
        ind = 3
    for H,axis in zip(fk_list[:ind],axis_list[:ind]):  #only first three joints
        a = np.dot(H[:3,:3],axis.reshape(-1,1)).reshape(1,-1)
        p = (fk_list[-1][:3,-1] - H[:3,-1]).reshape(1,-1)
        jac_column = np.cross(a,p).reshape(-1,1)
        jac.append(jac_column)
    return np.hstack(jac)

"""
Solves the inverse kinematics problem to find the joint angles that result in positioning the end effector at the target location.
Our method positions joint 3 at a location that is calculated using the offset between the gripper and joint 3.
The angles for joint 4 and joint 5 are computed such that the gripper is parallel to the ground and achives the required yaw angle.
Input: target_pose - numpy array containing the target position
       yaw - float value containing the desired yaw in radians
       max_iter - max number of iterations before the loop times out. If a solution is not found within the error threshold,
                  the function times out.
Output: q - joint angles that are the solution to the inverse kinematics
"""
def inverse_kinematics(target_pose,yaw=0,max_iter=1000):
    #Computing gripper offset to account for error in end effector position
    gripper_offset = np.array([0, 0, 0.23])
    target_pose = target_pose + gripper_offset
    #Initializing joint angles
    q = np.zeros((5,1))
    q[0] = -np.pi/4
    q[1] = np.pi/4
    q[2] = np.pi/5

    x = np.zeros(3)    
    dx = target_pose - x

    i = 0
    while(np.any(np.absolute(dx) > 1e-4)):
        if i == max_iter:
            print("Timeout")
            return None
        i +=1
        #Computing the pose of the joints and the jacobian
        _,fk_list = forward_kinematics(q)
        jac = jacobian(fk_list)
        #Using the pseudoinverse to find dq
        dq = np.dot(np.linalg.pinv(jac),dx).reshape(-1,1)
        #Updating the joint angles
        q[:3] = q[:3] + dq 
        #Recomputing error in task space position of the 3D point (joint_4)
        final_pos,_ = forward_kinematics(q)
        dx = target_pose - final_pos["joint_4"][:3]
    
    #Adding the offset to align the gripper parallel to the ground
    q[3] = np.pi/2 - final_pos["joint_3"][4] 
    #Adding the offset to account for the input yaw
    q[4] = -q[0] + yaw
    #Mapping the angle to lie within -pi/2 to pi/2 
    q = np.array([map_angle(a) for a in q])
    #Return the solution if it is within the joint limits
    if (np.all(abs(q[:3]) >= 0) and np.all(abs(q[:3]) <= np.pi/2)):
        return list(q.reshape(-1,)) 
    else:
        print("Solution not within joint limits")
        return None
  
#Helper function to map an angle in the fourth quadrant to a negative value    
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

#Usage Example:
if __name__ == "__main__":
    output_angles = inverse_kinematics(np.array([ 0.23574115, -0.11669366,  0.15680248]))
    print("Output joint angles: ",output_angles)
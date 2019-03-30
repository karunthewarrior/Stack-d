import numpy as np
import transforms3d as tf

def get_H(orig,axis,angle):
    orig = np.vstack([np.hstack([np.eye(3),np.array(orig).reshape(-1,1)]),[0,0,0,1]])
    axis = np.vstack([np.hstack([tf.axangles.axangle2mat(axis, angle),np.array([0, 0, 0]).reshape(-1,1)]),[0,0,0,1]])
    H = np.dot(orig,axis)
    return H 

def trans_H(trans):
    return np.vstack([np.hstack([np.eye(3),np.array(trans).reshape(-1,1)]),[0,0,0,1]])

def rot_H(R):
    return np.vstack([np.hstack([R,np.zeros(3).reshape(-1,1)]),[0,0,0,1]])

def forward_kinematics(angles):
    orig_list = np.array([[0,0,0.072],[0, 0, 0.04125],[0.05, 0, 0.2],[0.2002, 0, 0],[0.063, 0.0001, 0]])
    axis_list = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0]]
 
    all_H = [get_H(origin,axis,np.deg2rad(angle)) for origin,axis,angle in zip(orig_list,axis_list,angles)]

    fk_list = [all_H[0]]
    for i in range(2,6):
        fk_list.append(np.linalg.multi_dot(all_H[0:i]))
    
    centroids = np.vstack([H[:3,-1] for H in fk_list]) 
    orientation = np.vstack([tf.euler.mat2euler(x[:3,:3],'rxyz') for x in fk_list])

    pose = np.hstack([centroids,orientation])
    fk_mat = fk_list[-1]
    return pose,fk_list

def jacobian(fk_list):
    axis_list = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0]])
    jac = []
    for H,axis in zip(fk_list,axis_list):
        a = np.dot(H[:3,:3],axis.reshape(-1,1)).reshape(1,-1)
        p = (fk_list[-1][:3,-1] - H[:3,-1]).reshape(1,-1)
        jac_column = np.vstack([np.cross(a,p).reshape(-1,1),a.reshape(-1,1)])
        jac.append(jac_column)
    return np.hstack(jac)


if __name__ == "__main__":
    angles= np.rad2deg([-0.9501,0.8786,0.5130,-1.4157,-0.1997])
    link_pose,fk_list = forward_kinematics(angles)
    jac = jacobian(fk_list)

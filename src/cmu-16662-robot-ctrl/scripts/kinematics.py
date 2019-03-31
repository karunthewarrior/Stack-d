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
 
    all_H = [get_H(origin,axis,angle) for origin,axis,angle in zip(orig_list,axis_list,angles)]

    fk_list = [all_H[0]]
    for i in range(2,6):
        fk_list.append(np.linalg.multi_dot(all_H[0:i]))
    
    centroids = np.vstack([H[:3,-1] for H in fk_list]) 
    orientation = np.vstack([tf.euler.mat2euler(x[:3,:3],'rxyz') for x in fk_list])

    pose = np.hstack([centroids,orientation])
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

def inverse_kinematics(target_pose,max_iter=100):
    q = np.zeros((5,1))
    joint_pose,_ = forward_kinematics(q)
    x = joint_pose[-1]
    dx = target_pose - x
    i = 0
    while(np.all(np.absolute(dx) > 1e-12)):
        if i == max_iter:
            return None
        i +=1
        _,fk_list = forward_kinematics(q)
        jac = jacobian(fk_list)
        dq = np.dot(np.linalg.pinv(jac),dx).reshape(-1,1)
        q = q + dq 
        # print(q)
        final_pos,_ = forward_kinematics(q)
        dx = target_pose - final_pos[-1]
    q = [map_angle(a) for a in q%(2*np.pi)]
    return q

def map_angle(a):
    if a > np.pi:
        return float(a - 2*np.pi)
    else:
        return float(a)

if __name__ == "__main__":
    angles= [-0.9501,0.8786,0.5130,-1.4157,-0.1997]
    print(angles)
    link_pose,fk_list = forward_kinematics(angles)
    print(fk_list[-1])
    jac = jacobian(fk_list)
    q = inverse_kinematics([0.16557369, -0.23141237,  0.00692751,  0.09782728, -0.17601728, -0.93904669])



import numpy as np
# import transforms3d as tf
import transformations as tf

def get_H(orig,axis,angle):
    orig = np.vstack([np.hstack([np.eye(3),np.array(orig).reshape(-1,1)]),[0,0,0,1]])
    axis = tf.quaternion_matrix(tf.quaternion_about_axis(angle,axis))
    H = np.dot(orig,axis)
    return H 

def trans_H(trans):
    return np.vstack([np.hstack([np.eye(3),np.array(trans).reshape(-1,1)]),[0,0,0,1]])

def rot_H(R):
    return np.vstack([np.hstack([R,np.zeros(3).reshape(-1,1)]),[0,0,0,1]])

def forward_kinematics(angles):
    orig_list = np.array([[0,0,0.072],[0, 0, 0.04125],[0.05, 0, 0.2],[0.2002, 0, 0],[0.193, 0, 0]])
    axis_list = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0]]
 
    all_H = [get_H(origin,axis,angle) for origin,axis,angle in zip(orig_list,axis_list,angles)]

    fk_list = [all_H[0]]
    for i in range(2,6):
        fk_list.append(np.linalg.multi_dot(all_H[0:i]))
    
    centroids = np.vstack([H[:3,-1] for H in fk_list]) 
    orientation = np.vstack([tf.euler_from_matrix(x,'rxyz') for x in fk_list])

    pose_mat = np.hstack([centroids,orientation])
    pose = {"joint_1":pose_mat[0],"joint_2":pose_mat[1],"joint_3":pose_mat[2],"joint_4":pose_mat[3],"gripper":pose_mat[4]}
    return pose,fk_list

def jacobian(fk_list):
    axis_list = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0]])
    jac = []
    for H,axis in zip(fk_list[:3],axis_list[:3]):  #only first three joints
        a = np.dot(H[:3,:3],axis.reshape(-1,1)).reshape(1,-1)
        p = (fk_list[-1][:3,-1] - H[:3,-1]).reshape(1,-1)
        jac_column = np.vstack([np.cross(a,p).reshape(-1,1),a.reshape(-1,1)])
        # jac_column = np.cross(a,p).reshape(-1,1)
        jac.append(jac_column)
    return np.hstack(jac)

def inverse_kinematics(target_pose,open_grip=True,max_iter=1000,offset=True):
    q = np.ones((6,1)) * np.pi/4
    gripper_offset = np.array([0, 0, 0.193])
    # print(target_pose,"PRE KARUN")
    if offset:
        target_pose = target_pose + gripper_offset
    # print(target_pose,"POSE KaruN")
    q[0] = -np.pi/4
    # q[1] = np.pi/2
    q[2] = np.pi/5
    q[3] = 0
    q[4] = 0
    if open_grip:
        q[5] = -0.87
    else:
        q[5] = -0.33
    x = np.zeros(3)
    dx = target_pose - x
    i = 0
    while(np.any(np.absolute(dx) > 1e-1)):
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
    q[4] = -q[0]
    q = np.array([map_angle(a) for a in q])
    print(q,"Q",np.rad2deg(q))
    if (np.all(abs(q[:3]) >= 0) and np.all(abs(q[:3]) <= np.pi/2)):
        return list(q.reshape(-1,)) 
    else:
        # print(q)
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
    # angles= [-0.9501,0.8786,0.5130,-1.4157,-0.1997]
    # link_pose,fk_list = forward_kinematics([0,0,0,np.pi,0])
    # print(link_pose[-1])
    # jac = jacobian(fk_list)
    # q = inverse_kinematics([0.16557369, -0.23141237,  0.00692751])
    pos_list = [[0.3,0,0.1]]
    for pos in pos_list:
        q = inverse_kinematics(pos)
        print(q,"WQEQWE")
        print("FK",forward_kinematics(q)[0]["joint_4"])
        if q !=None:
            print(np.rad2deg(q))
# print(forward_kinematics(q)[0][-1,0:3])
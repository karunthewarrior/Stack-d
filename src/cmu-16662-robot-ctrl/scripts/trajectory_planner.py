import kinematics as kin
import numpy as np

def straight_trajectory(start_pose,target_pose,yaw=0,max_iter=1000):
    alpha = 1

    gripper_offset = np.array([0, 0.015, 0.185])
    target_pose = target_pose
    #Initializing joint angles
    q = np.array(kin.inverse_kinematics(start_pose,yaw)).reshape(-1,1)
    print(q.shape)
    #TODO initialze x to the position of joint_4 given the initial joint configuration. Currently zero works so not changing. 
    #The motivation to change it would be so that we can get straight line paths from initial to target position.
    x = start_pose
    dx = target_pose - x

    i = 0
    q_list = []
    x_list = []
    print(x)
    while(np.any(np.absolute(dx) > 1e-2)):
        # print(dx,"dx")
        if i == max_iter:
            print("timeout")
            return None
        i +=1
        #Computing the pose of the joints and the jacobian
        _,fk_list = kin.forward_kinematics(q)
        jac = kin.jacobian(fk_list)
        #Using the pseudoinverse to find dq
        dq = np.dot(jac.T,dx).reshape(-1,1)
        #Updating the joint angles
        q[:3] = q[:3] + alpha*dq 
        #Recomputing error in task space position of the 3D point (joint_4)
        final_pos,_ = kin.forward_kinematics(q)
        #Adding the offset to align the gripper parallel to the ground
        q[3] = np.pi/2 - final_pos["joint_4"][4] 
        #Adding the offset to account for the input yaw
        q[4] = -q[0] + yaw
        dx = target_pose - final_pos["joint_4"][:3] 
    
        #Mapping the angle to lie within -pi/2 to pi/2 
        q = np.array([kin.map_angle(a) for a in q]).reshape(-1,1)
        if not (np.all(abs(q[:3]) >= 0) and np.all(abs(q[:3]) <= np.pi/2)):
            return None
        q_list.append(q)
        x_list.append(final_pos["joint_4"][:3])
        print(x_list[-1],"x")
    return x_list,q_list

if __name__=="__main__":
    p1 = np.array([0.2,0.1,0.2])
    p2 = np.array([0.2,0.1,0.1])
    # q = kin.inverse_kinematics(p2,0)
    # print(q,"q")
    x,q = straight_trajectory(p1,p2)
    print(x)
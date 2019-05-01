import kinematics as kin
import ArmController as ac
import servoing as serv
import tag_estimator as tag
import rospy
import numpy as np

if __name__ =="__main__":
    rospy.init_node('block_pickup', anonymous=True)
    estimator = tag.tag_estimator()
    servo = serv.Point_detection()
    arm_controller = ac.ArmController()



    tilt_controller = ac.CamController('/tilt/state','/tilt/command')
    pan_controller = ac.CamController('/pan/state','/pan/command')
    rospy.sleep(0.5)
    pan_controller.set_cam_state(np.deg2rad(0))
    while(not pan_controller.has_converged()):
        pass
    tilt_controller.set_cam_state(np.deg2rad(-40))
    while(not tilt_controller.has_converged()):
        pass
    rospy.sleep(0.5)
    print("DONE")
    H_c2w = kin.cam_to_world(estimator.pan,estimator.tilt)
    
    servo_height = 0.04

    if len(estimator.p) == 4:
        pw = [np.hstack([np.dot(H_c2w,p)[:2],servo_height]) for p in estimator.p]
    else:
        print("no block detected")
    # rospy.loginfo(estimator.position_camera)
    # pos_list = [point1[:3]+np.array([0,0,0.05]),point1[:3],point1[:3]+np.array([0,0,0.05]),point2[:3]+np.array([0,0,0.05]),point2[:3]+np.array([0,0,0.02]),point2[:3]+np.array([0,0,0.05])]
    # arm_controller.home_arm()
    destination = tag.make_destination(np.array([0.25,-0.13,-0.11]),levels=1)
    print(pw)
    for i,(pos,dest) in enumerate(zip(pw,destination)):
        q = kin.inverse_kinematics(pos-np.array([0.02,0,0]),0)  #ADD DESIRED YAW 
        if q!=None:
            arm_controller.set_joint_state(q)
            while(not arm_controller.has_converged()):
                pass 
            arm_controller.open()
            print("reached set point")
            rospy.sleep(1)
            print("servoing in xy now")
            (x,y) = serv.servo_xy(arm_controller,servo)
            print("Finished servoing xy")
            s = np.array([x+0.037,y,-0.11])
            if i > 1:
                yaw_flag = True
            else:
                yaw_flag = False
            traj,grip,yaw_list = tag.make_trajectory(s,dest,yaw_flag)
            for pt,g,yaw in zip(traj,grip,yaw_list):
                q = kin.inverse_kinematics(pt,0)
                arm_controller.set_joint_state(q)
                while(not arm_controller.has_converged()):
                    pass
                if g:
                    arm_controller.close()
                else:
                    arm_controller.open()
            # print(q,"angls")
            # if q!=None:
            #     arm_controller.set_joint_state(q)
            #     while(not arm_controller.has_converged()):
            #         pass
            # q = kin.inverse_kinematics(np.array([x+0.037,y,-0.125]),0)
            # print(q,"angls")
            # if q!=None:
            #     arm_controller.set_joint_state(q)
            #     while(not arm_controller.has_converged()):
            #         pass
            # print("NO SOLUTION!!")
            # print("servoing in z now")
            # rospy.sleep(1)
            # serv.servo_z(arm_controller,servo)
            # arm_controller.close()

        else:
            print("No solution")
            exit()
import kinematics as kin
import ArmController as ac
import servoing as serv
import tag_estimator as tag
import rospy
import numpy as np
from std_msgs.msg import Int32

if __name__ =="__main__":
    rospy.init_node('block_pickup', anonymous=True)
    estimator = tag.tag_estimator()
    servo = serv.Point_detection()
    arm_controller = ac.ArmController()
    color_pub = rospy.Publisher('/block_color',Int32,queue_size = 1)

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
    
    servo_height = 0.03
    rospy.sleep(5)
    print(len(estimator.p),"YESAEAWSE")
    if len(estimator.p) == 4:
        pw = [np.hstack([np.dot(H_c2w,p[0])[:2],servo_height,p[1]]) for p in estimator.p] #p[1] is color 
        print(pw)
    else:
        print("no block detected")
    # rospy.loginfo(estimator.position_camera)
    # pos_list = [point1[:3]+np.array([0,0,0.05]),point1[:3],point1[:3]+np.array([0,0,0.05]),point2[:3]+np.array([0,0,0.05]),point2[:3]+np.array([0,0,0.02]),point2[:3]+np.array([0,0,0.05])]
    # arm_controller.home_arm()
    destination = tag.make_destination(np.array([0.3,-0.13,-0.07]),levels=1)
    print(pw)
    for i,(pos,dest) in enumerate(zip(pw,destination)):
        color_pub.publish(pos[3])
        q = kin.inverse_kinematics(pos[:3]+np.array([-0.037,0,servo_height]),0)  #ADD DESIRED YAW 
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
            s = np.array([x+0.037,y,-0.08])
            if i > 1:
                yaw_flag = True
            else:
                yaw_flag = False
            traj,grip,yaw_list = tag.make_trajectory(s,dest,yaw_flag)
            for ind,(pt,g,yaw) in enumerate(zip(traj,grip,yaw_list)):
                q = kin.inverse_kinematics(pt,yaw)
                arm_controller.set_joint_state(q)

                while(not arm_controller.has_converged()):
                    pass
                if ind == 1 or ind ==4:
                    serv.servo_z(arm_controller,servo,'down',yaw=yaw)
                    serv.servo_z(arm_controller,servo,'down',yaw=yaw)
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
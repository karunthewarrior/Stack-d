import kinematics as kin
import ArmController as ac
import servoing as serv
import tag_estimator as tag
import rospy
import numpy as np
from std_msgs.msg import Int32
import math

if __name__ =="__main__":
    rospy.init_node('scene_analysis', anonymous=True)
    estimator = tag.tag_estimator()
    servo = serv.Point_detection()
    arm_controller = ac.ArmController()
    color_pub = rospy.Publisher('/block_color',Int32,queue_size = 1)

    tilt_controller = ac.CamController('/tilt/state','/tilt/command')
    pan_controller = ac.CamController('/pan/state','/pan/command')
    rospy.sleep(0.5)
###################################################################
    pan_controller.set_cam_state(np.deg2rad(-15))
    while(not pan_controller.has_converged()):
        pass
    tilt_controller.set_cam_state(np.deg2rad(-42))
    while(not tilt_controller.has_converged()):
        pass
    rospy.sleep(0.5)
    # print("DONE")
    H_c2w = kin.cam_to_world(estimator.pan,estimator.tilt)
    
    servo_height = 0.04
    rospy.sleep(1)
    print(len(estimator.p),"YESAEAWSE")
    if len(estimator.p) == 3:
        pw = [np.hstack([np.dot(H_c2w,p[0])[:2],servo_height,p[1]]) for p in estimator.p] #p[1] is color 
        print(pw)
    else:
        print("no block detected")
    destination_list = []
    for pos in pw:
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
            (x,y,theta) = serv.servo_xy(arm_controller,servo,alpha=4e-3,error_thresh=30)
            print("Finished servoing xy")
            destination_list.append((x+(math.cos(theta)*0.037),y+(math.sin(theta)*0.037),theta))
        else:
            print("No solution")

            exit()


 ##############################################################################   
    # destination_list = [(0.22331957748384187, -0.12810423965076156, -0.4035348892211914), (0.22159798903887126, -0.24227360593643243, 0.38050639629364014), (0.17225704439775558, -0.21728558998308453, 1.5083775520324707), (0.3067707448908044, -0.17565317508125985, -1.5204188823699951)]
    # destination_list = [(x+(math.cos(theta)*0.037),y+(math.sin(theta)*0.037),theta)  for x,y,theta in destination_list] 
    
    drop_center = (0.26,0.18)
    destination = tag.move_structure(drop_center,destination_list)
    print(destination,"DEST")
    # rospy.sleep(3)    
    print("FINISHED SCENE ANALYSIS- BUILDING STRUCTURE")
    rospy.sleep(2)
    pan_controller.set_cam_state(0.432582587004)


    q = [0, 0.1135, 0.9603, -0.9235, -0.0015]
    arm_controller.set_joint_state(q)
    while(not arm_controller.has_converged()):
        pass 
    q = [0, -0.4080, 1.5079, 0.1212, 0.00767]
    arm_controller.set_joint_state(q)
    while(not arm_controller.has_converged()):
        pass 

    while(not pan_controller.has_converged()):
        pass
    tilt_controller.set_cam_state(-0.696427)
    while(not tilt_controller.has_converged()):
        pass
    rospy.sleep(0.5)
    print("DONE")
    H_c2w = kin.cam_to_world(estimator.pan,estimator.tilt)
    
    servo_height = 0.035
    rospy.sleep(1)
    print(len(estimator.p),"YESAEAWSE")
    if len(estimator.p) == 3:
        pw = [np.hstack([np.dot(H_c2w,p[0])[:2],servo_height,p[1]]) for p in estimator.p] #p[1] is color 
        print(pw)
    else:
        print("no block detected")

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
            (x,y,theta) = serv.servo_xy(arm_controller,servo)
            print("Finished servoing xy")
            s = np.array([x+0.037,y,-0.08])  #ADD DIRECTION COSINES
            # print(dest[0],dest[1],"CHEKTHIS")
            traj,grip,yaw_list = tag.make_trajectory_yaw(s,dest[0],dest[1])
            for ind,(pt,g,yaw) in enumerate(zip(traj,grip,yaw_list)):
                print(pt,"THIS IS THE POINT",ind)
                q = kin.inverse_kinematics(pt,yaw)  #ADD THETA LATER
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
        else:
            print("No solution")
            exit()

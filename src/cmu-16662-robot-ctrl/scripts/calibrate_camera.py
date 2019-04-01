import sys
import numpy as np
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import kinematics as kin
import ArmController as controller
import pickle


class camera_calib:
    def __init__(self):
        self.camera_pose_topic = "/ar_pose_marker"
        self.ar_position = np.array([None,None,None])
        sub_tag = rospy.Subscriber(self.camera_pose_topic,AlvarMarkers,self.get_camera_pose)

    def get_camera_pose(self,camera_pose):
        time = camera_pose.header.stamp
        try:
            ar_position_obj = camera_pose.markers[0].pose.pose.position
            self.ar_position = np.array([ar_position_obj.x,ar_position_obj.y,ar_position_obj.z])
            # rospy.loginfo(self.ar_position)
        except:
            self.ar_position = np.array([None,None,None])

def get_block(arm_pt):
    x,y,z = arm_pt
    a1 = np.array([x,y,z,1,0,0,0,0,0,0,0,0]).reshape(1,-1)
    a2 = np.array([0,0,0,0,x,y,z,1,0,0,0,0]).reshape(1,-1)
    a3 = np.array([0,0,0,0,0,0,0,0,x,y,z,1]).reshape(1,-1)
    A = np.vstack([a1,a2,a3])
    return A

def getA_x(p1,p2):
    return np.array([p1[0],p1[1],1,0,0,0,-p1[0]*p2[0],-p2[0]*p1[1],-p2[0]])

def getA_y(p1,p2):
    return np.array([0,0,0,p1[0],p1[1],1,-p1[0]*p2[1],-p2[1]*p1[1],-p2[1]])

def computeH(p1, p2):
    A = np.vstack([np.vstack([getA_x(pp1,pp2),getA_y(pp1,pp2)]) for pp1,pp2 in zip(p1,p2)])
    u,s,v = np.linalg.svd(A)
    h = v[-1]
    H2to1 = (h/h[-1]).reshape(3,3)
    return H2to1

if __name__ == "__main__":
    rospy.init_node("camera_calib", anonymous=True)
    cam = camera_calib()
    controller = controller.ArmController()
    rospy.sleep(2)
    target_joints = np.deg2rad([[0,-20,20,-70,0],[0,-20,20,-80,0],[10,-20,30,-80,0],[-20,-20,30,-80,0],[-20,20,30,-80,0],[20,20,20,-80,0],[20,20,0,-70,0],[20,20,0,-70,20],[20,20,0,-50,10],[30,10,0,-50,-20],[40,10,-20,-50,-20],[0,10,-20,-50,0],[0,10,-10,-50,0],[-20,20,30,-80,0]])
    arm_pts_list = []
    cam_pts_list = []
    for joint in target_joints:
        controller.set_joint_state(joint)
        while(not controller.has_converged()):
            pass
        cam_pos = cam.ar_position
        arm_pos = kin.forward_kinematics(joint)[0][3,:3]
        rospy.loginfo(cam_pos)
        rospy.loginfo(arm_pos)
        arm_pts_list.append(arm_pos)
        cam_pts_list.append(cam_pos)
    rospy.loginfo(arm_pts_list)
    rospy.loginfo(cam_pts_list)
    pickle.dump(arm_pts_list, open("arm_pts_pos.p","wb"))
    pickle.dump(cam_pts_list, open("cam_pts_pos.p","wb"))



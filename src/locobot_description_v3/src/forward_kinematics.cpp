#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <urdf/model.h>
#include "eigen3/Eigen/Dense"

using namespace std;

Eigen::MatrixXd getAxisTransformation(urdf::Joint j,double angle){
        Eigen::AngleAxisd axis= Eigen::AngleAxisd(angle,Eigen::Vector3d(j.axis.x,j.axis.y,j.axis.z));
            Eigen::MatrixXd rot = axis.toRotationMatrix();
                Eigen::MatrixXd T_mat = Eigen::MatrixXd::Identity(4,4);
                    T_mat.block(0,0,3,3) = rot;
                        return T_mat;
}

Eigen::MatrixXd getOriginTransformation(urdf::Joint j){
        Eigen::Vector3d t = Eigen::Vector3d(j.parent_to_joint_origin_transform.position.x,j.parent_to_joint_origin_transform.position.y,j.parent_to_joint_origin_transform.position.z);
            Eigen::Quaterniond q = Eigen::Quaterniond(j.parent_to_joint_origin_transform.rotation.w,j.parent_to_joint_origin_transform.rotation.x,j.parent_to_joint_origin_transform.rotation.y,j.parent_to_joint_origin_transform.rotation.z);
                Eigen::MatrixXd rot = q.toRotationMatrix();
                    Eigen::MatrixXd T_mat = Eigen::MatrixXd::Identity(4,4);
                        T_mat.block(0,0,3,3) = rot;
                            T_mat.block(0,3,3,1) = t;
                                return T_mat;
}

/**
 * Get the wrist pose for given joint angle configuration.
 * list of 16 values which represent the joint end effector pose obtained from the end-effector transformation matrix
 * using column-major ordering.
 * @param joint_angles list of joint angles to get final end effector pose for
 * @return
 */
std::vector<std::vector<double> > getJointPose(std::vector<double> joint_angles,urdf::Model model,std::vector<string> joint_names) {
  /**
  * TODO: You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */
  std::vector<std::vector<double> > joint_vec;
  std::vector<double> fk_vec;
  Eigen::MatrixXd out = Eigen::MatrixXd::Identity(4,4);
    for(int i=0;i<joint_names.size();i++)
          {
              urdf::Joint j= *model.getJoint(joint_names[i]);
              out = out * getOriginTransformation(j) * getAxisTransformation(j,joint_angles[i]);
              Eigen::VectorXd eig_vec(Eigen::Map<Eigen::VectorXd>(out.data(),out.size()));
	      fk_vec.resize(eig_vec.size());
              Eigen::VectorXd::Map(&fk_vec[0], eig_vec.size()) = eig_vec;
	      joint_vec.push_back(fk_vec);
          }
    //Eigen::VectorXd eig_vec(Eigen::Map<Eigen::VectorXd>(out.data(),out.size()));
    //std::vector<double> fk_vec;
    //fk_vec.resize(eig_vec.size());
    //Eigen::VectorXd::Map(&fk_vec[0], eig_vec.size()) = eig_vec;
    return joint_vec;
}

/**
 * Get the wrist pose for given joint angle configuration.
 * list of 16 values which represent the joint end effector pose obtained from the end-effector transformation matrix
 * using column-major ordering.
 * @param joint_angles list of joint angles to get final end effector pose for
 * @return
 */
std::vector<double> getWristJacobian(std::vector<double> joint_angles,urdf::Model model) {
  /**
  * TODO: You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */
    std::string joint_names[5] = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
    Eigen::MatrixXd out = Eigen::MatrixXd::Identity(4,4);
    vector<Eigen::Vector3d> translations; 
    vector<Eigen::MatrixXd> rotations;
    for(int i=0;i<5;i++)
    {
        urdf::Joint j = *model.getJoint(joint_names[i]);
        out = out * getOriginTransformation(j) * getAxisTransformation(j,joint_angles[i]);
        translations.push_back(out.block(0,3,3,1));
        rotations.push_back(out.block(0,0,3,3));
    }
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,5);
    for(int i=0;i<5;i++){
    urdf::Joint j = *model.getJoint(joint_names[i]);
    if (j.type == 1)
        {
        Eigen::Vector3d a,p;
        Eigen::MatrixXd jac_column = Eigen::MatrixXd::Zero(6,1);
        a = rotations.at(i)*Eigen::Vector3d(j.axis.x,j.axis.y,j.axis.z);
        p = translations.back() - translations.at(i);
        //p = Eigen::Vector3d::Zero(3);
        jac_column.block(0,0,3,1) = a.cross(p);
        jac_column.block(3,0,3,1) = a;
        jacobian.block(0,i,6,1) = jac_column;
        }
    else if(j.type == 3){
        Eigen::Vector3d a;
        Eigen::MatrixXd jac_column = Eigen::MatrixXd::Zero(6,1);
        jac_column.block(0,0,3,1) = a;
        jacobian.block(0,i,6,1) = jac_column;
        } 
    }
    Eigen::VectorXd eig_vec(Eigen::Map<Eigen::VectorXd>(jacobian.data(),jacobian.size()));
    std::vector<double> jac_vec;
    jac_vec.resize(eig_vec.size());
    Eigen::VectorXd::Map(&jac_vec[0], eig_vec.size()) = eig_vec;
    return jac_vec;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "fk");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  std::vector<string> joint_names = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
  //std::vector<double> joint_angles = {-0.9501,0.8786,0.5130,-1.4157,-0.1997};
  std::vector<double> joint_angles = {0,0,0,0,0};
  std::vector<double> fk;
  std::vector<std::vector<double>> joints;
  joints = getJointPose(joint_angles,model,joint_names);
  cout<<"FORWARD KINEMATTICS: \n";
  for(int i=0;i<joints.size();i++)
  {
       fk = joints[i];
       for(int j=0;j<fk.size();j++)
       {
           cout<<std::setprecision(3)<<fk[j]<<" ";
       }
  cout<<"\nNEXT JOINT\n"; 
  }
  return 0;
}


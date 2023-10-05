#include "ros/ros.h"
#include <smm_screws/ScrewsKinematics.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle nh;
  
  ScrewsKinematics screw_kin_obj;

  Eigen::Vector3f w(0.0, 0.0, 1.0);
  Eigen::Isometry3f As3 = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f Ast = Eigen::Isometry3f::Identity(); 
  Eigen::Isometry3f A3t;
  //Eigen::Matrix3f skew_w = screw_obj.skew(w);
  //std::cout << skew_w;
  As3(0,0) = 1.0; As3(0,1) = 0.0; As3(0,2) = 0.0; As3(0,3) = 1.0;
  As3(1,0) = 0.0; As3(1,1) = 0.0; As3(1,2) = -1.0; As3(1,3) = 0.0;
  As3(2,0) = 0.0; As3(2,1) = 1.0; As3(2,2) = 0.0; As3(2,3) = 0.0;
  As3(3,0) = 0.0; As3(3,1) = 0.0; As3(3,2) = 0.0; As3(3,3) = 1.0;  

  Ast(0,0) = 1.0; Ast(0,1) = 0.0; Ast(0,2) = 0.0; Ast(0,3) = 1.5;
  Ast(1,0) = 0.0; Ast(1,1) = 0.0; Ast(1,2) = -1.0; Ast(1,3) = 0.0;
  Ast(2,0) = 0.0; Ast(2,1) = 1.0; Ast(2,2) = 0.0; Ast(2,3) = 0.0;
  Ast(3,0) = 0.0; Ast(3,1) = 0.0; Ast(3,2) = 0.0; Ast(3,3) = 1.0; 

  A3t = screw_kin_obj.extractRelativeTf(Ast, As3);
  
  std::cout << A3t.matrix() << std::endl;

  // Define the SMM structure properties (extracted by MATLAB analysis)
  Structure2Pseudos robot_def2;
  RobotAbstractBase* robot_ptr = &robot_def2;
  ScrewsKinematics smm_robot_kin_solver;
  // ACTIVE JOINTS COMPONENTS
  robot_ptr->active_twists[0] << 0, 0, 0, 0, 0, 1;
  robot_ptr->active_twists[1] << 0, 0, 0, 0, -1, 0;
  robot_ptr->active_twists[2] << 0, 0, -1, 0, -1, 0;
  Eigen::Isometry3f* robot_tfs[DOF+1];
  Eigen::Isometry3f gsa10;
  Eigen::Isometry3f gsa20;
  Eigen::Isometry3f gsa30;
  Eigen::Isometry3f gst0;
  gsa10(0,0) = 1.0; gsa10(0,1) = 0.0; gsa10(0,2) = 0.0; gsa10(0,3) = 0.0;
  gsa10(1,0) = 0.0; gsa10(1,1) = 1.0; gsa10(1,2) = 0.0; gsa10(1,3) = 0.0;
  gsa10(2,0) = 0.0; gsa10(2,1) = 0.0; gsa10(2,2) = 1.0; gsa10(2,3) = 0.0;
  gsa10(3,0) = 0.0; gsa10(3,1) = 0.0; gsa10(3,2) = 0.0; gsa10(3,3) = 1.0; 
  robot_ptr->gsai_ptr[0] = &gsa10;
  gsa20(0,0) = 1.0; gsa20(0,1) = 0.0; gsa20(0,2) = 0.0; gsa20(0,3) = 0.0;
  gsa20(1,0) = 0.0; gsa20(1,1) = 0.0; gsa20(1,2) = -1.0; gsa20(1,3) = 0.0;
  gsa20(2,0) = 0.0; gsa20(2,1) = 1.0; gsa20(2,2) = 0.0; gsa20(2,3) = 0.0;
  gsa20(3,0) = 0.0; gsa20(3,1) = 0.0; gsa20(3,2) = 0.0; gsa20(3,3) = 1.0; 
  robot_ptr->gsai_ptr[1] = &gsa20;
  gsa30(0,0) = 1.0; gsa30(0,1) = 0.0; gsa30(0,2) = 0.0; gsa30(0,3) = 1.0;
  gsa30(1,0) = 0.0; gsa30(1,1) = 0.0; gsa30(1,2) = -1.0; gsa30(1,3) = 0.0;
  gsa30(2,0) = 0.0; gsa30(2,1) = 1.0; gsa30(2,2) = 0.0; gsa30(2,3) = 0.0;
  gsa30(3,0) = 0.0; gsa30(3,1) = 0.0; gsa30(3,2) = 0.0; gsa30(3,3) = 1.0; 
  robot_ptr->gsai_ptr[2] = &gsa30;
  gst0(0,0) = 1.0; gst0(0,1) = 0.0; gst0(0,2) = 0.0; gst0(0,3) = 1.5;
  gst0(1,0) = 0.0; gst0(1,1) = 0.0; gst0(1,2) = -1.0; gst0(1,3) = 0.0;
  gst0(2,0) = 0.0; gst0(2,1) = 1.0; gst0(2,2) = 0.0; gst0(2,3) = 0.0;
  gst0(3,0) = 0.0; gst0(3,1) = 0.0; gst0(3,2) = 0.0; gst0(3,3) = 1.0; 
  robot_ptr->gsai_ptr[3] = &gst0;
  // PASSIVE JOINTS COMPONENTS
  //uint8_t metalink1_total_pseudos = 1;
  //uint8_t metalink2_total_pseudos = 1;
  Structure2Pseudos* derived_ptr = static_cast<Structure2Pseudos*>(robot_ptr);
  derived_ptr->META1_PSEUDOS = 1;
  derived_ptr->META2_PSEUDOS = 1;
  derived_ptr->pseudo_angles[0] = 0;
  derived_ptr->pseudo_angles[1] = 0;
  derived_ptr->passive_twists[0] << -0.00 , 0.1660, -0.025, 1.0, 0.0 , 0.0;
  derived_ptr->passive_twists[1] << -0.4685 , 0.00, -0.025, 0.0, -1.0 , 0.0;
  std::shared_ptr<RobotAbstractBase> robot_def_shared = std::shared_ptr<Structure2Pseudos>(derived_ptr);
  smm_robot_kin_solver = ScrewsKinematics(robot_def_shared);

  Eigen::Isometry3f gst;
  float q[3] = {0, 0.0658, 2.0236};
  gst = smm_robot_kin_solver.ForwardKinematicsTCP(q);
  //std::cout << gst.matrix() << std::endl;

  return 0;
}

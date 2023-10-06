//#include "ros/ros.h"
#include <smm_screws/ScrewsKinematics.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::NodeHandle nh;
  /*
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
*/
  // Define the SMM structure properties (extracted by MATLAB analysis)
  Structure2Pseudos robot_def2; // object of the specific structure class, here is specified, later will be selected based the structure yaml file
  RobotAbstractBase *robot_ptr = &robot_def2; // pointer to abstract class(), can access derived class members
  // ACTIVE JOINTS COMPONENTS
  robot_ptr->active_twists[0] << 0, 0, 0, 0, 0, 1;
  robot_ptr->active_twists[1] << 0, 0, 0, 0, -1, 0;
  robot_ptr->active_twists[2] << 0, 0, -1, 0, -1, 0;
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
  robot_def2.META1_PSEUDOS = 1;
  robot_def2.META2_PSEUDOS = 1;
  //ROS_INFO("meta1_1: %d", robot_ptr->get_PSEUDOS_METALINK1());
  //ROS_INFO("meta2_1: %d", robot_ptr->get_PSEUDOS_METALINK2());
  robot_def2.pseudo_angles[0] = 0;
  robot_def2.pseudo_angles[1] = 0;
  robot_def2.passive_twists[0] << -0.00 , 0.1660, -0.025, 1.0, 0.0 , 0.0;
  robot_def2.passive_twists[1] << -0.4685 , 0.00, -0.025, 0.0, -1.0 , 0.0;

  // Tested ForwardKinematicsTCP
  ScrewsKinematics smm_robot_kin_solver(robot_ptr);
  smm_robot_kin_solver.extractPseudoTfs();
  float q[3] = {0, 0.0658, 2.0236};
  smm_robot_kin_solver.ForwardKinematicsTCP(q);

  // Tested ForwardKinematics3DOF_1
  Eigen::Isometry3f* robot_tfs[DOF+1]; // These pointers are uninitialized (they don't yet point to valid memory locations)
  Eigen::Isometry3f g[DOF+1]; // joint frames tfs @q 
  for (size_t i = 0; i < DOF+1; i++)
  {
      robot_tfs[i] = &g[i];
  }
  smm_robot_kin_solver.ForwardKinematics3DOF_1(q, robot_tfs);
  // Tested ForwardKinematics3DOF_2
  smm_robot_kin_solver.ForwardKinematics3DOF_2(q, robot_tfs);

  // Setting TCP Position to ROS_PARAMETER_SERVER
  Eigen::Vector3f  _pos_tcp_vector = robot_tfs[3]->translation();
  nh.setParam("/x_s_TCP", _pos_tcp_vector.x() );
  nh.setParam("/y_s_TCP", _pos_tcp_vector.y() );
  nh.setParam("/z_s_TCP", _pos_tcp_vector.z() );

  return 0;
}

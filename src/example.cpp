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

  return 0;
}

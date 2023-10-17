#include "ros/ros.h"
#include <smm_screws/ScrewsDynamics.h>

ScrewsDynamics* smm_robot_dyn_solver;  // Declare a pointer to the ScrewsDynamics class
ScrewsKinematics* smm_robot_kin_solver;  // Declare a pointer to the ScrewsKinematics class

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_base_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;
    /*
     * ROBOT DEFINITION - FOR A 3DOF ANTHROPOMORPHIC ROBOT 
     */
    // Define the SMM structure properties (extracted by MATLAB analysis)
    Structure2Pseudos robot_def2; // object of the specific structure class, here is specified, later will be selected based the structure yaml file
    RobotAbstractBase *robot_ptr = &robot_def2; // pointer to abstract class(), can access derived class members

    // Spin to keep the node alive.
    ros::spin();

    return 0;
}
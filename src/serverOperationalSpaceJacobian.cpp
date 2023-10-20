#include "ros/ros.h"
#include <smm_screws/robot_shared.h>
#include "sensor_msgs/JointState.h"
#include "smm_screws/SetOperationalSpaceJacobian.h" 

ros::Subscriber joint_states_sub;
robot_shared my_shared_lib;
ScrewsKinematics* smm_robot_kin_solver;

std::vector<double> jointPositions;
std::vector<double> jointVelocities;
float q_received[DOF];
float dq_received[DOF];

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    jointPositions = msg->position;
    jointVelocities = msg->velocity;
    return;
}

bool publish_server_Jop(smm_screws::SetOperationalSpaceJacobian::Request &req,
                       smm_screws::SetOperationalSpaceJacobian::Response &res)
{
    // 1. Get the joint position,velocity data from /joint states
    for (size_t i = 0; i < DOF; ++i) {
        q_received[i] = static_cast<float>(jointPositions[i]);
        dq_received[i] = static_cast<float>(jointVelocities[i]);
    }
    smm_robot_kin_solver->updateJointState(q_received, dq_received);

    // 2. Calculate the Jacobian matrix based on the request data
    smm_robot_kin_solver->ForwardKinematics3DOF_1(); // update g[]
    smm_robot_kin_solver->BodyJacobian_Tool_1();
    smm_robot_kin_solver->OperationalSpaceJacobian();

    // Fill in the srv file elements
    req.Jop_00 = smm_robot_kin_solver->Jop(0,0);
    req.Jop_01 = smm_robot_kin_solver->Jop(0,1);
    req.Jop_02 = smm_robot_kin_solver->Jop(0,2);
    req.Jop_10 = smm_robot_kin_solver->Jop(1,0);
    req.Jop_11 = smm_robot_kin_solver->Jop(1,1);
    req.Jop_12 = smm_robot_kin_solver->Jop(1,2);
    req.Jop_20 = smm_robot_kin_solver->Jop(2,0);
    req.Jop_21 = smm_robot_kin_solver->Jop(2,1);
    req.Jop_22 = smm_robot_kin_solver->Jop(2,2);

    // Set success to true if the calculation is successful
    res.success = true; 

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_operational_space_jacobian");
    ros::NodeHandle nh;

    // 1. Initialize the shared library for robot analytical solvers using screws
    if (my_shared_lib.initializeSharedLib()) {
        smm_robot_kin_solver = &my_shared_lib.get_screws_kinematics_solver();
    }

    // 2. Create a subscriber to acquire joint data
    joint_states_sub = nh.subscribe("/joint_states", 1, jointStatesCallback);

    // 3. Create a service server
    ros::ServiceServer service = nh.advertiseService("current_operational_space_jacobian_srv", publish_server_Jop);

    ROS_INFO("Operational Space Jacobian server is ready to receive requests...");

    ros::spin();
    return 0;
}
#include "ros/ros.h"
#include <smm_screws/robot_shared.h>
#include "sensor_msgs/JointState.h"
#include "smm_screws/SetOperationalSpaceJacobian.h" 

ros::Subscriber joint_states_sub;
robot_shared my_shared_lib;
ScrewsKinematics* ptr2_smm_robot_kin_solver;
ScrewsKinematics smm_robot_kin_solver;;

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
    ptr2_smm_robot_kin_solver->updateJointState(q_received, dq_received);

    // 2. Calculate the Jacobian matrix based on the request data
    ptr2_smm_robot_kin_solver->ForwardKinematics3DOF_2(); // update g[]
    
    // Initializing kinematic matrices for jacobian calculation
    Eigen::Isometry3f* rel_tfs[DOF+1]; 
    //Eigen::Isometry3f B[DOF+1];
    for (size_t i = 0; i < DOF+1; i++)
    {
        rel_tfs[i] = &smm_robot_kin_solver.B[i];
    }
    smm_robot_kin_solver.initializeRelativeTfs(rel_tfs);
    
    Eigen::Matrix<float, 6, 1>* local_screws[DOF+1]; 
    //Eigen::Matrix<float, 6, 1>  iXi[DOF+1]; 
    for (size_t i = 0; i < DOF+1; i++)
    {
        local_screws[i] = &smm_robot_kin_solver.iXi[i];
    }
    smm_robot_kin_solver.initializeLocalScrewCoordVectors(local_screws);
    
    ptr2_smm_robot_kin_solver->BodyJacobian_Tool_1();
    ptr2_smm_robot_kin_solver->OperationalSpaceJacobian();

    // Fill in the srv file elements
    req.Jop_00 = ptr2_smm_robot_kin_solver->Jop(0,0);
    req.Jop_01 = ptr2_smm_robot_kin_solver->Jop(0,1);
    req.Jop_02 = ptr2_smm_robot_kin_solver->Jop(0,2);
    req.Jop_10 = ptr2_smm_robot_kin_solver->Jop(1,0);
    req.Jop_11 = ptr2_smm_robot_kin_solver->Jop(1,1);
    req.Jop_12 = ptr2_smm_robot_kin_solver->Jop(1,2);
    req.Jop_20 = ptr2_smm_robot_kin_solver->Jop(2,0);
    req.Jop_21 = ptr2_smm_robot_kin_solver->Jop(2,1);
    req.Jop_22 = ptr2_smm_robot_kin_solver->Jop(2,2);

    // Set success to true if the calculation is successful
    res.success = true; 

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_operational_space_jacobian");
    ros::NodeHandle nh;

    // 1. Initialize the shared library for robot analytical solvers using screws
    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {
        ROS_INFO("[SERVER Operational Space Jacobian] Initialized Shared Library.");
        ptr2_smm_robot_kin_solver = &smm_robot_kin_solver;
        smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    }

    // 2. Create a subscriber to acquire joint data
    joint_states_sub = nh.subscribe("/anthropomorphic_3dof_gazebo/joint_states", 1, jointStatesCallback);

    // 3. Create a service server
    ros::ServiceServer service = nh.advertiseService("current_operational_space_jacobian_srv", publish_server_Jop);

    ROS_INFO("Operational Space Jacobian server is ready to receive requests...");

    ros::Rate rate(100); // 100Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
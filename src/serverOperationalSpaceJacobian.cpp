#include "ros/ros.h"
#include <smm_screws/robot_shared.h>
#include "sensor_msgs/JointState.h"
#include "smm_screws/SetOperationalSpaceJacobian.h" 

ros::Subscriber joint_states_sub;
robot_shared my_shared_lib;
ScrewsKinematics smm_robot_kin_solver;

std::vector<double> jointPositions;
std::vector<double> jointVelocities;
float q_received[robot_params::DOF];
float dq_received[robot_params::DOF];

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    jointPositions = msg->position;
    jointVelocities = msg->velocity;
    return;
}

bool initServerOperationalJacobian(smm_screws::SetOperationalSpaceJacobian::Request &req,
                       smm_screws::SetOperationalSpaceJacobian::Response &res)
{
    // 1. Get the joint position,velocity data from /joint states
    for (size_t i = 0; i < robot_params::DOF; ++i) {
        q_received[i] = static_cast<float>(jointPositions[i]);
        dq_received[i] = static_cast<float>(jointVelocities[i]);
    }
    smm_robot_kin_solver.updateJointState(q_received, dq_received);
    // 2. Calculate the Jacobian matrix based on the request data
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // update g[]   
    // 3. Extract Body Jacobian
    smm_robot_kin_solver.BodyJacobian_Tool_1();
    // 4. Extract Operational Space Jacobian (from Body and TCP Rot matrix)
    smm_robot_kin_solver.OperationalSpaceJacobian(); // -> computes Jop
    // 5. Fill in the srv file elements
    if (req.give_op_jacob) {
        res.Jop_00 = smm_robot_kin_solver.Jop(0,0);
        res.Jop_01 = smm_robot_kin_solver.Jop(0,1);
        res.Jop_02 = smm_robot_kin_solver.Jop(0,2);
        res.Jop_10 = smm_robot_kin_solver.Jop(1,0);
        res.Jop_11 = smm_robot_kin_solver.Jop(1,1);
        res.Jop_12 = smm_robot_kin_solver.Jop(1,2);
        res.Jop_20 = smm_robot_kin_solver.Jop(2,0);
        res.Jop_21 = smm_robot_kin_solver.Jop(2,1);
        res.Jop_22 = smm_robot_kin_solver.Jop(2,2);    
    }
    else {
        ROS_INFO("[SERVER-OperationalSpaceJacobian] No client request received.");
    }
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
        smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    }

    // 2. Create a subscriber to acquire joint data
    joint_states_sub = nh.subscribe("/anthropomorphic_3dof_gazebo/joint_states", 1, jointStatesCallback);

    // 3. Create a service server
    ros::ServiceServer service = nh.advertiseService("current_operational_space_jacobian_srv", initServerOperationalJacobian);
    // 3.1 Print screen message for server state
    ROS_INFO("[SERVER Operational Space Jacobian] Ready to receive requests...");

    // 4. Loop the server
    ros::Rate rate(100); // 100Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
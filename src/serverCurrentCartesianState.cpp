#include "ros/ros.h"
#include <smm_screws/robot_shared.h>
#include "sensor_msgs/JointState.h"
#include "smm_screws/SetOperationalSpaceJacobian.h" 
#include "smm_screws/SetCurrentCartesianState.h" 

ros::Subscriber joint_states_sub;
robot_shared my_shared_lib;
ScrewsKinematics smm_robot_kin_solver;

std::vector<double> jointPositions;
std::vector<double> jointVelocities;
float q_received[robot_params::DOF];

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    jointPositions = msg->position;
    return;
}

// Callback function for the "current_cartesian_state" service
bool initServerCartesianState(smm_screws::SetCurrentCartesianState::Request &req,
                             smm_screws::SetCurrentCartesianState::Response &res)
{
    // Create a client to call the "current_operational_space_jacobian" service
    ros::NodeHandle nh;
    ros::ServiceClient jacobian_client = nh.serviceClient<smm_screws::SetOperationalSpaceJacobian>("current_operational_space_jacobian_srv");

    smm_screws::SetOperationalSpaceJacobian jacobian_srv;
    jacobian_srv.request.give_op_jacob = true;

    Eigen::Matrix3f Jop_received;
    Eigen::Vector3f p_qs;
    Eigen::Vector3f v_qs;

    // 1. Get the joint position,velocity data from /joint states
    for (size_t i = 0; i < robot_params::DOF; ++i) {
        q_received[i] = static_cast<float>(jointPositions[i]);
    }

    // 2. Execute Forward Kinematics to acquire position data /{S} frame
    p_qs = smm_robot_kin_solver.updatePositionTCP(q_received);
    //res.p_e_s_x = p_qs.x();
    //res.p_e_s_y = p_qs.y();
    //res.p_e_s_z = p_qs.z();
    // 2.3 Print the calculated TCP velocity
    //ROS_INFO("Cartesian Position State [x] : %f ", res.p_e_s_x);
    //ROS_INFO("Cartesian Position State [y] : %f ", res.p_e_s_y);
    //ROS_INFO("Cartesian Position State [z] : %f ", res.p_e_s_z);

    // 3. Calculate Operational Space Jacobian (Position only for 3dof)  
    //    and extract TCP velocity /{S} frame
    if (jacobian_client.call(jacobian_srv))
    {
        // 3.1 Here access the Jacobian data
        Jop_received(0, 0) = jacobian_srv.response.Jop_00;
        Jop_received(0, 1) = jacobian_srv.response.Jop_01;
        Jop_received(0, 2) = jacobian_srv.response.Jop_02;
        Jop_received(1, 0) = jacobian_srv.response.Jop_10;
        Jop_received(1, 1) = jacobian_srv.response.Jop_11;
        Jop_received(1, 2) = jacobian_srv.response.Jop_12;
        Jop_received(2, 0) = jacobian_srv.response.Jop_20;
        Jop_received(2, 1) = jacobian_srv.response.Jop_21;
        Jop_received(2, 2) = jacobian_srv.response.Jop_22;
        // 3.2 pass to function that calculates cartesian state
        smm_robot_kin_solver.CartesianVelocity_jacob(v_qs, Jop_received);
        // Form the response part of the service for velocity
        //res.v_e_s_x = v_qs.x();
        //res.v_e_s_y = v_qs.y();
        //res.v_e_s_z = v_qs.z();
        // 3.3 Print the calculated TCP velocity
        //ROS_INFO("Cartesian Velocity State [x] : %f ", res.v_e_s_x);
        //ROS_INFO("Cartesian Velocity State [y] : %f ", res.v_e_s_y);
        //ROS_INFO("Cartesian Velocity State [z] : %f ", res.v_e_s_z);
    }
    else
    {
        ROS_ERROR("[SERVER-CurrentCartesianState] Failed to call: current_operational_space_jacobian_srv.");
        return false;
    }

    if (req.give_cur_cart_state)
    {
        res.p_e_s_x = p_qs.x();
        res.p_e_s_y = p_qs.y();
        res.p_e_s_z = p_qs.z();
        res.v_e_s_x = v_qs.x();
        res.v_e_s_y = v_qs.y();
        res.v_e_s_z = v_qs.z();
        return true;
    } else {
        ROS_INFO("[SERVER-CurrentCartesianState] No client request received.");
        return false;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_cartesian_state_server");
    ros::NodeHandle nh;

    // 1. Initialize the shared library for robot analytical solvers using screws
    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {
        ROS_INFO("[SERVER-CurrentCartesianState] Initialized Shared Library.");
        smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    }

    // 2. Create a subscriber to acquire joint data
    joint_states_sub = nh.subscribe("/anthropomorphic_3dof_gazebo/joint_states", 1, jointStatesCallback);

    // 3. Create a service server for "current_cartesian_state" server
    ros::ServiceServer service = nh.advertiseService("current_cartesian_state_srv", initServerCartesianState);

    ROS_INFO("[SERVER-CurrentCartesianState] Ready to receive requests...");

    // 4. Loop the server
    ros::Rate rate(100); // 100Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
#include "ros/ros.h"
#include <smm_screws/OperationalSpaceControllers.h>
#include "smm_screws/CurrentCartesianState.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>

/*
 *  Here the simplest controller is implemented. The centralized approach is followed.
 *  -> A simple subscriber to /joint_states updates the joint positions/velocities
 *  -> Each kinematic-dynamic matrix used for controller torque output is computed locally
 *  -> The torques are published in the designated effort_controller topic of the gazebo robot model
 */

ros::Subscriber joint_states_sub;
robot_shared my_shared_lib;
ScrewsKinematics smm_robot_kin_solver;
ScrewsDynamics smm_robot_dyn_solver;
ScrewsKinematics *ptr2_smm_robot_kin_solver;
ScrewsDynamics *ptr2_smm_robot_dyn_solver;
OperationalSpaceControllers::InverseDynamicsController *ptr2_idosc;

std::vector<double> jointPositions;
std::vector<double> jointVelocities;
float q_received[robot_params::DOF];
float dq_received[robot_params::DOF];
Eigen::Matrix<float, IDOSC_STATE_DIM, 1> desired_state;
Eigen::Matrix<float, IDOSC_STATE_DIM, 1> current_state;
Eigen::Vector3f torques;
std_msgs::Float64 torque_msg;

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_state, ros::Publisher& joint1_torque_pub, ros::Publisher& joint2_torque_pub, ros::Publisher& joint3_torque_pub , ros::Publisher& cartesian_state_pub )
{
    bool _debug_verbosity = true;

    ros::Time current_time = joint_state->header.stamp;

    Eigen::Vector3f p_qs;
    Eigen::Vector3f v_qs;

    // 1. Read most recently published data
    jointPositions = joint_state->position;
    jointVelocities = joint_state->velocity;
    // 1. Get the joint position,velocity data from /joint states
    for (size_t i = 0; i < DOF; ++i) {
        q_received[i] = static_cast<float>(jointPositions[i]);
        dq_received[i] = static_cast<float>(jointVelocities[i]);
    }
    // 2. Call ScrewsKinematics Library tools
    smm_robot_kin_solver.updateJointState(q_received, dq_received);
    smm_robot_kin_solver.ForwardKinematics3DOF_2();     // update g[]   
    smm_robot_kin_solver.BodyJacobian_Tool_1();
    smm_robot_kin_solver.OperationalSpaceJacobian();    // -> computes Jop
    smm_robot_kin_solver.DtOperationalSpaceJacobian();  // -> computes dJop
    
    // 3. Call ScrewsDynamics Library tools
    smm_robot_dyn_solver.updateJointPos(q_received);
    smm_robot_dyn_solver.updateJointVel(dq_received);

    // 4. CONTROLLER ACTION
    // 4.1 Update the current cartesian state
    p_qs = smm_robot_kin_solver.updatePositionTCP(q_received);
    smm_robot_kin_solver.CartesianVelocity_jacob(v_qs);
    current_state(0) = v_qs.x();
    current_state(1) = v_qs.y();
    current_state(2) = v_qs.z(); 
    current_state(3) = p_qs.x();
    current_state(4) = p_qs.y();
    current_state(5) = p_qs.z(); 
    for (int i = 0; i < 6; i++) {
        ROS_INFO("[PUBLISHER-IDOSC] Current_state[ %d ]: %f", i, current_state(i));
    }
    // 4.2 Build the current error state
    ptr2_idosc->set_error_state(current_state);

    // 4.3 Execute the main controller
    ptr2_idosc->update_dq(dq_received);
    ptr2_idosc->update_control_input();
    // 4.4 Comput the torques, given control input & robot dynamics
    ptr2_idosc->update_torques(torques);
    // Print extracted torque command
    for (int i = 0; i < robot_params::DOF; i++)
    {
        ROS_INFO("[PUBLISHER-IDOSC] Torque cmd joint [ %d ]: %f", i+1, torques(i));
    }

    // 5.1 Publish torque commands -> gazebo model / rqt_plot
    torque_msg.data = torques(0);
    joint1_torque_pub.publish(torque_msg);
    torque_msg.data = torques(1);
    joint2_torque_pub.publish(torque_msg);
    torque_msg.data = torques(2);
    joint3_torque_pub.publish(torque_msg);

    // 5.2 Publish current cartesian state -> rqt_plot
    smm_screws::CurrentCartesianState cur_cart_state_msg;
    cur_cart_state_msg.header.stamp = current_time;
    cur_cart_state_msg.p_e_s_x = p_qs.x();
    cur_cart_state_msg.p_e_s_y = p_qs.y();
    cur_cart_state_msg.p_e_s_z = p_qs.z();
    cur_cart_state_msg.v_e_s_x = v_qs.x();
    cur_cart_state_msg.v_e_s_y = v_qs.y();
    cur_cart_state_msg.v_e_s_z = v_qs.z();   
    cartesian_state_pub.publish(cur_cart_state_msg); 
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "idosc_centralized");
    ros::NodeHandle nh;

    // 1.1 Initialize the shared library for robot analytical solvers using screws
    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {
        ROS_INFO("[PUBLISHER-IDOSC] Initialized Shared Library.");
        smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
        smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();   
        ptr2_smm_robot_kin_solver = &smm_robot_kin_solver;  
        ptr2_smm_robot_dyn_solver = &smm_robot_dyn_solver;
    }
    // 1.2 Initialize the idosc class object
    OperationalSpaceControllers::InverseDynamicsController idosc(ptr2_smm_robot_kin_solver, ptr2_smm_robot_dyn_solver);
    ptr2_idosc = &idosc;
    // 1.3 Set the desired state to trigger controller action (MUST YAML INPUT)
    desired_state(0) = 0.0f;
    desired_state(1) = 0.0f;
    desired_state(2) = 0.0f;
    desired_state(3) = 0.50f;
    desired_state(4) = 0.0f;
    desired_state(5) = 2.00f; // + 1.0 to the matlab value
    ptr2_idosc->set_desired_state(desired_state);

    // 2.1 Start the publisher to joints' effort controllers /command
    ros::Publisher joint1_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint1_effort_controller/command", 1);
    ros::Publisher joint2_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint2_effort_controller/command", 1);
    ros::Publisher joint3_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint3_effort_controller/command", 1);
    
    // 2.2 Start the publisher to /current_cartesian_state
    ros::Publisher cartesian_state_pub = nh.advertise<smm_screws::CurrentCartesianState>("/current_cartesian_state", 1);

    // 2.3 Start the subscriber to /joint_states
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/anthropomorphic_3dof_gazebo/joint_states", 1, boost::bind(jointStatesCallback, _1, joint1_torque_pub, joint2_torque_pub, joint3_torque_pub, cartesian_state_pub));

    ROS_INFO("[PUBLISHER-IDOSC] Ready to start publishing torques...");

    // 3. Loop the server
    ros::Rate rate(100); // 100Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
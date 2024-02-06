#include "ros/ros.h"
#include <smm_screws/OperationalSpaceControllers.h>
#include "smm_screws/CurrentCartesianState.h"
#include "smm_screws/ForceMeasurements.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include <std_msgs/Float64.h>

/*
 *  Here a simple hybrid(position+force) controller is implemented
 *  -> A simple subscriber to /joint_states updates the joint positions/velocities
 *  -> A simple subscriber to /ft_sensor updates the 3D force sensor measurements (xacro file must include the libgazebo_ros_ft_sensor.so)
 *  -> Xacro file must include a spherical end-effector for better contact simulation
 *  -> Gazebo world file must include a xy rigid plane in the environment
 *  -> Each kinematic-dynamic matrix used for controller torque output is computed locally
 *  -> The torques are published in the designated effort_controller topic of the gazebo robot model
 */

ros::Subscriber joint_states_sub;
robot_shared my_shared_lib;
ScrewsKinematics smm_robot_kin_solver;
ScrewsDynamics smm_robot_dyn_solver;
ScrewsKinematics *ptr2_smm_robot_kin_solver;
ScrewsDynamics *ptr2_smm_robot_dyn_solver;
OperationalSpaceControllers::HybridController3 *ptr2_hybrid3;

std::vector<double> jointPositions;
std::vector<double> jointVelocities;
std::vector<double> ForceMeasurements;
float q_received[DOF];
float dq_received[DOF];
float force_received[DOF];
Eigen::Matrix<float, HYBRID_STATE_DIM, 1> desired_state;
Eigen::Matrix<float, HYBRID_STATE_DIM, 1> current_state;
Eigen::Vector3f torques;
std_msgs::Float64 torque_msg;

// Save parames loaded from launch file
Eigen::Vector3f pi, pf, zf_s;
std::vector<double> pi_param, pf_param, zf_s_param;
float pi_x_param, pi_y_param, pi_z_param;
float pf_x_param, pf_y_param, pf_z_param;
float zf_s_x_param, zf_s_y_param, zf_s_z_param;

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_state, ros::Publisher& joint1_torque_pub, ros::Publisher& joint2_torque_pub, ros::Publisher& joint3_torque_pub , ros::Publisher& cartesian_state_pub)
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
    ptr2_hybrid3->update_q(q_received);
    ptr2_hybrid3->update_dq(dq_received);
    smm_robot_kin_solver.ForwardKinematics3DOF_2();     // update g[]   
    smm_robot_kin_solver.BodyJacobian_Tool_1();
    smm_robot_kin_solver.OperationalSpaceJacobian();    // -> computes Jop
    smm_robot_kin_solver.DtOperationalSpaceJacobian();  // -> computes dJop
    
    // 3. Call ScrewsDynamics Library tools
    smm_robot_dyn_solver.updateJointPos(q_received);
    smm_robot_dyn_solver.updateJointVel(dq_received);

    // 4. CONTROLLER ACTION
    // 4.1 Update the error state
    ptr2_hybrid3->update_error_state();
    // 4.2 Force Controller Part
    ptr2_hybrid3->calculate_force_control_component(); // Needs _lamda_d, K matrices, error state
    // 4.3 Motion Controller Part
    ptr2_hybrid3->calculate_motion_control_component(); // Needs K matrices, error state
    // 4.4 Task Space Dynamic Matrices: Be, Ne
    ptr2_hybrid3->calculate_MassMatrix_task_space();
    ptr2_hybrid3->calculate_CoriolisVector_task_space();
    // 4.5 Comput the torques, given control input & robot dynamics
    ptr2_hybrid3->update_torques(torques); // Needs Be, Ne
    
    // Print extracted torque command
    for (int i = 0; i < DOF; i++)
    {
        ROS_INFO("[hybrid3_centralized] Torque cmd joint [ %d ]: %f", i+1, torques(i));
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

void forceMeasurementsCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg, ros::Publisher& force_measurements_pub ) {
    ros::Time current_time = msg->header.stamp;
    
    double fx = msg->wrench.force.x;
    double fy = msg->wrench.force.y;
    double fz = msg->wrench.force.z;
    force_received[0] = static_cast<float>(fx);
    force_received[1] = static_cast<float>(fy);
    force_received[2] = static_cast<float>(fz);
    for (int i = 0; i < DOF; i++)
    {
        ROS_INFO("[hybrid3_centralized] Force axis [ %d ]: %f", i+1, force_received[i]);
    }
    // update the measurement vector in the controller
    ptr2_hybrid3->update_force_measurements(force_received); // updates the _fe_c

    // publish the force components -> rqt_plot
    smm_screws::ForceMeasurements force_measurements_msg;
    force_measurements_msg.header.stamp = current_time;
    force_measurements_msg.force_meas_x = force_received[0];
    force_measurements_msg.force_meas_y = force_received[1];
    force_measurements_msg.force_meas_z = force_received[2];

    force_measurements_pub.publish(force_measurements_msg);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hybrid3_centralized");
    ros::NodeHandle nh;

    // 1.1 Load the task params from ROS_PARAMETER_SERVER (params MUST be set in the .launch file)
    if (!nh.getParam("hybrid3/pi_x", pi_x_param) ||
        !nh.getParam("hybrid3/pi_y", pi_y_param) ||
        !nh.getParam("hybrid3/pi_z", pi_z_param) ||
        !nh.getParam("hybrid3/pf_x", pf_x_param) ||
        !nh.getParam("hybrid3/pf_y", pf_y_param) ||
        !nh.getParam("hybrid3/pf_z", pf_z_param) ||
        !nh.getParam("hybrid3/zf_s_x", zf_s_x_param) ||
        !nh.getParam("hybrid3/zf_s_y", zf_s_y_param) ||
        !nh.getParam("hybrid3/zf_s_z", zf_s_z_param)) {
        ROS_ERROR("[hybrid3_centralized] Failed to retrieve TASK_PARAMS from ROS_PARAMETER_SERVER.");
        return 1;
    }
    pi << pi_x_param, pi_y_param, pi_z_param;
    pf << pf_x_param, pf_y_param, pf_z_param;
    zf_s << zf_s_x_param, zf_s_y_param, zf_s_z_param;
    
    // 1.2 Initialize the shared library for robot analytical solvers using screws
    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {
        ROS_INFO("[hybrid3_centralized] Initialized Shared Library.");
        smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
        smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();   
        ptr2_smm_robot_kin_solver = &smm_robot_kin_solver;  
        ptr2_smm_robot_dyn_solver = &smm_robot_dyn_solver;
    }
    // 1.3 Initialize the hybrid class object
    OperationalSpaceControllers::HybridController3 hybrid3(ptr2_smm_robot_kin_solver, ptr2_smm_robot_dyn_solver);
    ptr2_hybrid3 = &hybrid3;
    // 1.3.1. The Selection Matrices _Sv_c/_Sf_c and the Weight Matrices are set
    // 1.3.2  The Gain Matrices _Ki_l, _Kp_v, _Kd_v are set

    // 1.4 Set the {C} frame
    ptr2_hybrid3->initialize_constraint_frame(pi, pf, zf_s);
    ptr2_hybrid3->rotate_subspace_matrices_S();
    // 1.4.1 The _gsc tf is computed, _Rsc Rotation Matrix is saved
    // 1.4.2 Selection Matrices are expressed in the {S} frame
    
    // 1.5 Calculate the Pseudo-Inverse of the task Selection matrices
    ptr2_hybrid3->calculate_pinv_subspace_matrices();
    // 1.5.1 Calculated _pi_Sv_s,_pi_Sv_c,_pi_Sf_s,_pi_Sf_c

    // 1.6 Set the desired state to trigger controller action (MUST YAML INPUT)
    desired_state(0) = 0.0f;  // v(1)
    desired_state(1) = 0.0f;  // v(2)
    desired_state(2) = 0.0f;  // r(1)
    desired_state(3) = 0.0f;  // r(2)
    desired_state(4) = 0.0f;  // λ(1)
    desired_state(5) = 0.0f; // Ιλ(1)
    ptr2_hybrid3->set_desired_state(desired_state);

    // 2.1.1 Start the publisher to /torque_commands
    ros::Publisher joint1_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint1_effort_controller/command", 1);
    ros::Publisher joint2_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint2_effort_controller/command", 1);
    ros::Publisher joint3_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint3_effort_controller/command", 1);
    // 2.1.2 Start the publisher to /current_cartesian_state
    ros::Publisher cartesian_state_pub = nh.advertise<smm_screws::CurrentCartesianState>("/current_cartesian_state", 1);
    // 2.1.3 Start the publisher to /force_measurements
    ros::Publisher force_measurements_pub = nh.advertise<smm_screws::ForceMeasurements>("/force_measurements", 1);

    // 2.3.1 Start the subscriber to /joint_states
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/anthropomorphic_3dof_gazebo/joint_states", 1, boost::bind(jointStatesCallback, _1, joint1_torque_pub, joint2_torque_pub, joint3_torque_pub, cartesian_state_pub));
    // 2.3.2 Start the subscriber to /ft_sensor
    //ros::Subscriber ft_sub = nh.subscribe("/ft_sensor", 10, forceMeasurementsCallback);
    ros::Subscriber ft_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor", 1, boost::bind(forceMeasurementsCallback, _1, force_measurements_pub));

    ROS_INFO("[hybrid3_centralized] Ready to start publishing torques...");

    // 3. Loop the server
    ros::Rate rate(100); // 100Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
#include "ros/ros.h"
#include <smm_screws/robot_shared.h>
#include "sensor_msgs/JointState.h"
#include "smm_screws/CurrentCartesianState.h"

// [24-10-23] ROS node, that subscribes to /joint_states, reads current joint positions
// and velocities, computes the current cartesian TCP position/velocity /{S} frame.  
// To vizualize data during task execution run:
// $ rqt_plot /current_cartesian_state/p_e_s_x:p_e_s_y:p_e_s_z
// $ rqt_plot /current_cartesian_state/v_e_s_x:v_e_s_y:v_e_s_z

ros::Subscriber joint_states_sub;
robot_shared my_shared_lib;
ScrewsKinematics smm_robot_kin_solver;

std::vector<double> jointPositions;
std::vector<double> jointVelocities;
float q_received[robot_params::DOF];
float dq_received[robot_params::DOF];

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_state, ros::Publisher& cartesian_state_pub)
{
    ros::Time current_time = joint_state->header.stamp;

    Eigen::Vector3f p_qs;
    Eigen::Vector3f v_qs;

    // 1. Read most recently published data
    jointPositions = joint_state->position;
    jointVelocities = joint_state->velocity;
    // 1. Get the joint position,velocity data from /joint states
    for (size_t i = 0; i < robot_params::DOF; ++i) {
        q_received[i] = static_cast<float>(jointPositions[i]);
        dq_received[i] = static_cast<float>(jointVelocities[i]);
    }
    // 2. Call ScrewsKinematics Library tools
    smm_robot_kin_solver.updateJointState(q_received, dq_received);
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // update g[]   
    smm_robot_kin_solver.BodyJacobian_Tool_1();
    smm_robot_kin_solver.OperationalSpaceJacobian(); // -> computes Jop
    
    // 3. Extract TCP position and velocity
    p_qs = smm_robot_kin_solver.updatePositionTCP(q_received);
    smm_robot_kin_solver.CartesianVelocity_jacob(v_qs);
    
    // 4. Form the Current Cartesian State msg
    smm_screws::CurrentCartesianState cur_cart_state_msg;
    cur_cart_state_msg.header.stamp = current_time;
    cur_cart_state_msg.p_e_s_x = p_qs.x();
    cur_cart_state_msg.p_e_s_y = p_qs.y();
    cur_cart_state_msg.p_e_s_z = p_qs.z();
    cur_cart_state_msg.v_e_s_x = v_qs.x();
    cur_cart_state_msg.v_e_s_y = v_qs.y();
    cur_cart_state_msg.v_e_s_z = v_qs.z();
    
    // 5. Publish to topic
    cartesian_state_pub.publish(cur_cart_state_msg);
    //ROS_INFO("[TOPIC-CurrentCartesianState] Published 'cur_cart_state_msg' to topic.");

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_cartesian_state");
    ros::NodeHandle nh;

    // 1. Initialize the shared library for robot analytical solvers using screws
    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {
        ROS_INFO("[TOPIC-CurrentCartesianState] Initialized Shared Library.");
        smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    }

    // 2.1 Start the publisher to /current_cartesian_state
    ros::Publisher cartesian_state_pub = nh.advertise<smm_screws::CurrentCartesianState>("/current_cartesian_state", 1);

    // 2.2 Start the subscriber to /joint_states
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/anthropomorphic_3dof_gazebo/joint_states", 1, boost::bind(jointStatesCallback, _1, cartesian_state_pub));

    ros::spin();
    return 0;
}
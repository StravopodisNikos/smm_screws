#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <smm_screws/ScrewsDynamics.h>

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Process the received joint states
    std::vector<std::string> jointNames = msg->name;
    std::vector<double> jointPositions = msg->position;
    std::vector<double> jointVelocities = msg->velocity;

    // Your code for converting joint positions/velocities to task space here

    // Print joint names: positions and velocities (DEBUG ONLY)
    for (size_t i = 0; i < jointNames.size(); i++) {
        ROS_INFO("Joint Name: %s, Position: %f, Velocity: %f", jointNames[i].c_str(), jointPositions[i], jointVelocities[i]);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint2task_space");
    ros::NodeHandle nh;

    // Subscribe to the joint_states topic
    ros::Subscriber jointStatesSub = nh.subscribe("/anthropomorphic_3dof_gazebo/joint_states", 10, jointStatesCallback);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
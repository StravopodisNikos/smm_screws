#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "smm_screws/JointAccel.h"

ros::Time last_time;
std::vector<double> last_velocities;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state, ros::Publisher& joint_accel_pub)
{
    ros::Time current_time = joint_state->header.stamp;

    // Calculate the time difference
    double dt = (current_time - last_time).toSec();
    if (dt <= 0)
    {
        ROS_WARN("Invalid time difference (dt <= 0). Skipping acceleration calculation.");
        last_time = current_time;
        return;
    }

    // Initialize the joint accelerations message
    smm_screws::JointAccel joint_accel_msg;
    joint_accel_msg.header.stamp = current_time;

    // Calculate joint accelerations for each joint
    if (joint_state->velocity.size() == last_velocities.size())
    {
        joint_accel_msg.acceleration.resize(joint_state->velocity.size());

        for (size_t i = 0; i < joint_state->velocity.size(); ++i)
        {
            joint_accel_msg.acceleration[i] = (joint_state->velocity[i] - last_velocities[i]) / dt;
        }

        // Publish joint accelerations
        joint_accel_pub.publish(joint_accel_msg);
    }

    // Update variables for the next iteration
    last_time = current_time;
    last_velocities = joint_state->velocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_acceleration_calculator");
    ros::NodeHandle nh;

    ros::Publisher joint_accel_pub = nh.advertise<smm_screws::JointAccel>("/joint_accelerations", 1);
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, boost::bind(jointStateCallback, _1, joint_accel_pub));

    last_time = ros::Time::now();
    last_velocities.resize(0);

    ros::spin();
    return 0;
}
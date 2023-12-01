#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <smm_screws/idoscAction.h>
#include <smm_screws/OperationalSpaceControllers.h>
#include "smm_screws/CurrentCartesianState.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>

// 1. GLOBALS
ros::Time current_time;
robot_shared my_shared_lib;
ScrewsKinematics smm_robot_kin_solver;
ScrewsDynamics smm_robot_dyn_solver;
ScrewsKinematics *ptr2_smm_robot_kin_solver;
ScrewsDynamics *ptr2_smm_robot_dyn_solver;
OperationalSpaceControllers::InverseDynamicsController *ptr2_idosc;
std::vector<double> jointPositions;
std::vector<double> jointVelocities;
float q_received[DOF];
float dq_received[DOF];
Eigen::Matrix<float, IDOSC_STATE_DIM, 1> desired_state;
Eigen::Matrix<float, IDOSC_STATE_DIM, 1> current_state;
Eigen::Matrix<float, IDOSC_STATE_DIM, 1> error_state;
Eigen::Vector3f torques;
Eigen::Vector3f p_qs;
Eigen::Vector3f v_qs;
Eigen::Vector3f pos_error;
Eigen::Vector3f vel_error;

// msgs published in robots joints
std_msgs::Float64 torque_msg;
smm_screws::CurrentCartesianState cur_cart_state_msg;

// ROS subscribers
ros::Subscriber joint_states_sub;

// ROS publishers
ros::Publisher joint1_torque_pub;
ros::Publisher joint2_torque_pub;
ros::Publisher joint3_torque_pub;
ros::Publisher cartesian_state_pub;

// 2. ACTION SERVER CLASS
class idoscAction
{
private:
    /* data */
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<smm_screws::idoscAction> as_;
    std::string action_name_;
    smm_screws::idoscActionFeedback feedback_; // feedback insance
    smm_screws::idoscActionResult result_;

public:
    // Initializing an action server within the idoscAction class,
    // associating a callback function (executeCallback) to handle incoming goals,
    // and then starting the action server to make it ready for receiving 
    // and processing action requests.
    idoscAction(std::string name):
        // This initializes the action server within the idoscAction class.
        // boost::bind : binds the member function executeCallback within the idoscAction class
        // to the action server as the callback function to be executed when a goal is received.
        // This function is responsible for handling the goal.
        as_(nh_, name, boost::bind(&idoscAction::executeCallback, this, _1), false), action_name_(name)
        {
            as_.start();
        }
    
    ~idoscAction(void){}

    void executeCallback(const smm_screws::idoscActionGoalConstPtr& goal) { // goal type is automatic defined in devel/include/smm_screws/idoscActionGoal.h
        ros::Rate r(1);
        bool success = true;
        // Extract desired state from the client received goal
        geometry_msgs::Vector3 desired_state_pos = goal->goal.desired_state_pos;
        geometry_msgs::Vector3 desired_state_vel = goal->goal.desired_state_vel;
        //desired_state << desired_state_vel.x, desired_state_vel.y, desired_state_vel.z, desired_state_pos.x, desired_state_pos.y, desired_state_pos.z;
        desired_state(0) = desired_state_vel.x;
        desired_state(1) = desired_state_vel.y;
        desired_state(2) = desired_state_vel.z;
        desired_state(3) = desired_state_pos.x;
        desired_state(4) = desired_state_pos.y;
        desired_state(5) = desired_state_pos.z;

        // Pass the desired state vector to the controller
        ptr2_idosc->set_desired_state(desired_state);

        // Update kinematics-dynamics
        smm_robot_kin_solver.updateJointState(q_received, dq_received);
        smm_robot_kin_solver.ForwardKinematics3DOF_2();
        smm_robot_kin_solver.BodyJacobian_Tool_1();
        smm_robot_kin_solver.OperationalSpaceJacobian();
        smm_robot_kin_solver.DtOperationalSpaceJacobian();

        smm_robot_dyn_solver.updateJointPos(q_received);
        smm_robot_dyn_solver.updateJointVel(dq_received);

        // Update cartesian state of TCP
        p_qs = smm_robot_kin_solver.updatePositionTCP(q_received);
        smm_robot_kin_solver.CartesianVelocity_jacob(v_qs);
        current_state(0) = v_qs.x();
        current_state(1) = v_qs.y();
        current_state(2) = v_qs.z(); 
        current_state(3) = p_qs.x();
        current_state(4) = p_qs.y();
        current_state(5) = p_qs.z(); 

        // Send the current state as feedback
        feedback_.feedback.current_state_pos.x = p_qs.x();
        feedback_.feedback.current_state_pos.y = p_qs.y();
        feedback_.feedback.current_state_pos.z = p_qs.z();
        feedback_.feedback.current_state_vel.x = v_qs.x();
        feedback_.feedback.current_state_vel.y = v_qs.y();
        feedback_.feedback.current_state_vel.z = v_qs.z();

        // Set the controller error state
        ptr2_idosc->set_error_state(current_state, error_state);

        // Execute the controller
        ptr2_idosc->update_dq(dq_received);
        ptr2_idosc->update_control_input();
        ptr2_idosc->update_torques(torques);

        // Publish torques
        /*
        jointTorquePublisher( joint1_torque_pub, 0);
        jointTorquePublisher( joint1_torque_pub, 1);
        jointTorquePublisher( joint1_torque_pub, 2);

        // Publish Cartesian State
        cartesianStatePublisher(cartesian_state_pub );
        */

        // Prepare the result message
        vel_error.x() = error_state(0);
        vel_error.y() = error_state(1);
        vel_error.z() = error_state(2);
        pos_error.x() = error_state(3);
        pos_error.y() = error_state(4);
        pos_error.z() = error_state(5);

        if (as_.isPreemptRequested() || !ros::ok() )
        {
            ROS_INFO("[ACTION SERVER IDOSC] Preempted during ExecuteCb! ");
            as_.setPreempted();
            success = false;
        }
        
        if (success)
        {
            if ( (pos_error.norm() < 0.01f) && (vel_error.norm() < 0.01f) )
            {
                result_.result.current_state_pos.x = feedback_.feedback.current_state_pos.x;
                result_.result.current_state_pos.y = feedback_.feedback.current_state_pos.y;
                result_.result.current_state_pos.z = feedback_.feedback.current_state_pos.z;
                result_.result.current_state_vel.x = feedback_.feedback.current_state_vel.x;
                result_.result.current_state_vel.y = feedback_.feedback.current_state_vel.y;
                result_.result.current_state_vel.z = feedback_.feedback.current_state_vel.z;                
                as_.setSucceeded(result_.result); 
            }
        }           
    }
    /*
    static void jointTorquePublisher(ros::Publisher& joint_torque_pub, size_t joint_id ) {
        torque_msg.data = torques(joint_id);
        joint_torque_pub.publish(torque_msg);
    }   

    static void cartesianStatePublisher(ros::Publisher& cartesian_state_pub) {
        cur_cart_state_msg.header.stamp = current_time;
        cur_cart_state_msg.p_e_s_x = p_qs.x();
        cur_cart_state_msg.p_e_s_y = p_qs.y();
        cur_cart_state_msg.p_e_s_z = p_qs.z();
        cur_cart_state_msg.v_e_s_x = v_qs.x();
        cur_cart_state_msg.v_e_s_y = v_qs.y();
        cur_cart_state_msg.v_e_s_z = v_qs.z();   
        cartesian_state_pub.publish(cur_cart_state_msg); 
    }
    */
};

// 3. MAIN LOOP
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {
        // in this cb only the joint positions-velocities are updated

        current_time = joint_state->header.stamp;
        // Get the joint position,velocity data from /joint states
        jointPositions = joint_state->position;
        jointVelocities = joint_state->velocity;
        for (size_t i = 0; i < DOF; ++i) {
            q_received[i] = static_cast<float>(jointPositions[i]);
            dq_received[i] = static_cast<float>(jointVelocities[i]);
        }  

        //if (as_.isPreemptRequested() || !ros::ok())
        //{
        //    ROS_INFO("[ACTION SERVER IDOSC] Preempted during JointStatesCb! ");
        //    // Set the action state to preempted and exit
        //    as_.setPreempted();
        //    return;
        //}
        return;
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "idosc_centralized_action_server");
    ros::NodeHandle nh;

    // 1.1 Initialize the shared library for robot analytical solvers using screws
    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {
        ROS_INFO("[ACTION SERVER IDOSC] Initialized Shared Library.");
        smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
        smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();   
        ptr2_smm_robot_kin_solver = &smm_robot_kin_solver;  
        ptr2_smm_robot_dyn_solver = &smm_robot_dyn_solver;
    }
    // 1.2 Initialize the idosc class object
    OperationalSpaceControllers::InverseDynamicsController idosc(ptr2_smm_robot_kin_solver, ptr2_smm_robot_dyn_solver);
    ptr2_idosc = &idosc;

    // 2.1 Initialize and start running the action server
    idoscAction idosc_action_server("idosc_action_server");

    // 2.2 Start the publisher to /current_cartesian_state
    //joint1_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint1_effort_controller/command", 1);
    //joint2_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint2_effort_controller/command", 1);
    //joint3_torque_pub = nh.advertise<std_msgs::Float64>("/anthropomorphic_3dof_gazebo/joint3_effort_controller/command", 1);
    
    // 2.3 Start the publisher to /current_cartesian_state
    //cartesian_state_pub = nh.advertise<smm_screws::CurrentCartesianState>("/current_cartesian_state", 1);

    // 2.4 Start the subscriber to /joint_states
    //joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/anthropomorphic_3dof_gazebo/joint_states", 1, jointStatesCallback);
    joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/anthropomorphic_3dof_gazebo/joint_states", 1, jointStatesCallback);

    ROS_INFO("[ACTION SERVER IDOSC] Ready...");

    // 3. Loop the server
    ros::Rate rate(100); // 100Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
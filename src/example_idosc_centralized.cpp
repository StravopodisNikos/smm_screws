#include "ros/ros.h"
#include <smm_screws/OperationalSpaceControllers.h>

robot_shared my_shared_lib;
ScrewsKinematics smm_robot_kin_solver;
ScrewsDynamics smm_robot_dyn_solver;
ScrewsKinematics *ptr2_smm_robot_kin_solver;
ScrewsDynamics *ptr2_smm_robot_dyn_solver;
OperationalSpaceControllers::InverseDynamicsController *ptr2_idosc;

Eigen::Matrix<float, IDOSC_STATE_DIM, 1> desired_state;
Eigen::Matrix<float, IDOSC_STATE_DIM, 1> current_state;
Eigen::Vector3f torques;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "idosc_centralized");
    ros::NodeHandle nh;

    // Test code:
    float q[3] = {0, 0.9195, 1.8003};
    float dq[3] = {0 , 0, 0};
    Eigen::Vector3f p_qs;
    Eigen::Vector3f v_qs;

    // 1.1 Initialize the shared library for robot analytical solvers using screws
    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {
        ROS_INFO("[EXAMPLE-IDOSC] Initialized Shared Library.");
        smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
        smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();   
        ptr2_smm_robot_kin_solver = &smm_robot_kin_solver;  
        ptr2_smm_robot_dyn_solver = &smm_robot_dyn_solver;
    }
    // 1.2 Initialize the idosc class object
    OperationalSpaceControllers::InverseDynamicsController idosc(ptr2_smm_robot_kin_solver, ptr2_smm_robot_dyn_solver);
    ROS_INFO("[EXAMPLE-IDOSC] Initialized IDOSC class object.");
    ptr2_idosc = &idosc;
    // 1.3 Set the desired state to trigger controller action (MUST YAML INPUT)
    desired_state(0) = 0.0f;
    desired_state(1) = 0.0f;
    desired_state(2) = 0.0f;
    desired_state(3) = 0.5f;
    desired_state(4) = 0.0f;
    desired_state(5) = 2.0f;
    ptr2_idosc->set_desired_state(desired_state);

    // 2.1 Extract the current cartesian state
    // 2. Call ScrewsKinematics Library tools
    smm_robot_kin_solver.updateJointState(q, dq);
    smm_robot_kin_solver.ForwardKinematics3DOF_2();     // update g[]   
    smm_robot_kin_solver.BodyJacobian_Tool_1();
    smm_robot_kin_solver.OperationalSpaceJacobian();    // -> computes Jop
    smm_robot_kin_solver.DtOperationalSpaceJacobian();  // -> computes dJop
    
    // 3. Call ScrewsDynamics Library tools
    smm_robot_dyn_solver.updateJointPos(q);
    smm_robot_dyn_solver.updateJointVel(dq);

    // 4. CONTROLLER ACTION
    // 4.1 Update the current cartesian state
    p_qs = smm_robot_kin_solver.updatePositionTCP(q);
    smm_robot_kin_solver.CartesianVelocity_jacob(v_qs);
    current_state(0) = v_qs.x();
    current_state(1) = v_qs.y();
    current_state(2) = v_qs.z(); 
    current_state(3) = p_qs.x();
    current_state(4) = p_qs.y();
    current_state(5) = p_qs.z(); 
    for (int i = 0; i < 6; i++) {
        ROS_INFO("[EXAMPLE-IDOSC] current_state[ %d ]: %f", i, current_state(i));
    }
    // 4.2 Set the error state
    ptr2_idosc->set_error_state(current_state);

    // 4.3 Execute the main controller
    ptr2_idosc->update_dq(dq);
    ptr2_idosc->update_control_input();
    // 4.4 Comput the torques, given control input & robot dynamics
    ptr2_idosc->update_torques(torques);

    // Print extracted torque command
    for (int i = 0; i < 3; i++)
    {
        ROS_INFO("[EXAMPLE-IDOSC] Torque cmd joint [ %d ]: %f", i+1, torques(i));
    }
    
    return 0;
}
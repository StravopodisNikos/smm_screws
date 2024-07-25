//#include <smm_screws/ScrewsDynamics.h>
#include <smm_screws/robot_shared.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_dynamics");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;

    //robot_shared my_shared_lib;
    //if (my_shared_lib.initializeSharedLib()) {}
    // [13-7-24] Added for derived class - structure customization
    // Choose the robot structure you want to use
    /*
    robot_shared my_shared_lib(robot_shared::STRUCTURE_3_PSEUDOS);
    if (my_shared_lib.initializeSharedLib()) {
        // Initialization successful
    }
    */

    // Initialize the robot structure
    ROS_INFO("[example_dynamics] Initializing robot structure");
    RobotAbstractBase* robot_ptr = new Structure3Pseudos();

    // Debug statement
    ROS_INFO("[example_dynamics] Initializing shared library");

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[example_dynamics] Failed to initialize shared library.");
        return -1;
    }

    // Initialize the shared library for robot analytical solvers using screws
    ROS_INFO("[example_dynamics] Initialized Shared Library.");
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver(); 

    //ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    //ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();    
    
    // Test code:
    float q[3]  = {-0.4675 , 1.4464 , -1.5092};
    float dq[3] = {0.2516  , 0.0147 , -0.0227};
    smm_robot_kin_solver.updateJointState(q, dq);

    // Working on ScrewsKinematics shake-up
    smm_robot_kin_solver.ForwardKinematicsTCP();
    smm_robot_kin_solver.ForwardKinematics3DOF_1();
    smm_robot_kin_solver.ForwardKinematics3DOF_2();
    smm_robot_kin_solver.ForwardKinematicsComFrames3DOF_2();
    smm_robot_kin_solver.SpatialJacobian_Tool_1();
    smm_robot_kin_solver.SpatialJacobian_Tool_2();
    
    // update q,dq in ScrewsDynamics
    smm_robot_dyn_solver.updateJointPos(q);
    smm_robot_dyn_solver.updateJointVel(dq);

    // Calculate Mass Matrix
    smm_robot_dyn_solver.MM = smm_robot_dyn_solver.MassMatrix();

    // Calculate Coriolis Matrix
    smm_robot_dyn_solver.CM = smm_robot_dyn_solver.CoriolisMatrix();

    // Calculate Gravity Vector
    smm_robot_dyn_solver.GV = smm_robot_dyn_solver.GravityVectorAnalytical();

    // Calculate the Friction Vector
    smm_robot_dyn_solver.FV = smm_robot_dyn_solver.FrictionVector();
    
    return 0;
}
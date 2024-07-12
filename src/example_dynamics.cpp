//#include <smm_screws/ScrewsDynamics.h>
#include <smm_screws/robot_shared.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_dynamics");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;

    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {}
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();    
    
/*
    // Robot structure definition
    Structure2Pseudos robot_def2; // object of the specific structure class, here is specified, later will be selected based the structure yaml file
    RobotAbstractBase *robot_ptr = &robot_def2; // pointer to abstract class(), can access derived class members
    // Kinematics solver using screws
    ScrewsKinematics smm_robot_kin_solver(robot_ptr);
    smm_robot_kin_solver.initializePseudoTfs();
    // Dynamics solver using screws
    ScrewsDynamics smm_robot_dyn_solver(robot_ptr);
    smm_robot_dyn_solver.intializeLinkMassMatrices();
    smm_robot_kin_solver.extractPassiveTfs(smm_robot_dyn_solver.ptr2passive_tfs); 
*/
    // Test code:
    float q[3]  = {-0.4675 , 1.4464 , -1.5092};
    float dq[3] = {0.2516  , 0.0147 , -0.0227};
    smm_robot_kin_solver.updateJointState(q, dq);
    
    // Calculate Mass Matrix
    smm_robot_dyn_solver.updateJointPos(q);
    smm_robot_dyn_solver.MM = smm_robot_dyn_solver.MassMatrix();

    // Calculate Coriolis Matrix
    smm_robot_dyn_solver.updateJointVel(dq);
    //smm_robot_dyn_solver.CM = smm_robot_dyn_solver.CoriolisMatrix();

    // Calculate Gravity Vector
    //smm_robot_dyn_solver.GV = smm_robot_dyn_solver.GravityVector();
    smm_robot_dyn_solver.GV = smm_robot_dyn_solver.GravityVectorAnalytical();

    // Calculate the Friction Vector
    smm_robot_dyn_solver.FV = smm_robot_dyn_solver.FrictionVector();

    return 0;
}
#include <smm_screws/ScrewsDynamics.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_dynamics");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;

    Structure2Pseudos robot_def2; // object of the specific structure class, here is specified, later will be selected based the structure yaml file
    RobotAbstractBase *robot_ptr = &robot_def2; // pointer to abstract class(), can access derived class members

    /*
     * Kinematics 
     */
    ScrewsKinematics smm_robot_kin_solver(robot_ptr);
    smm_robot_kin_solver.initializePseudoTfs();
    float q[3] = {1.5748, 0.2758, 2.5436};
    float dq[3] = {-0.25 , 0.8934, 1.5748};
    /*
     * Dynamics 
     */
    ScrewsDynamics smm_robot_dyn_solver(robot_ptr);
    smm_robot_dyn_solver.intializeLinkMassMatrices();
    smm_robot_kin_solver.extractPassiveTfs(smm_robot_dyn_solver.ptr2passive_tfs); 

    // Calculate Mass Matrix
    smm_robot_dyn_solver.updateJointPos(q);
    smm_robot_dyn_solver.MM = smm_robot_dyn_solver.MassMatrix();

    // Calculate Coriolis Matrix
    smm_robot_dyn_solver.updateJointVel(dq);
    smm_robot_dyn_solver.CM = smm_robot_dyn_solver.CoriolisMatrix();

    // Calculate Gravity Vector
    smm_robot_dyn_solver.GV = smm_robot_dyn_solver.GravityVector();

    // Calculate the Friction Vector
    smm_robot_dyn_solver.FV = smm_robot_dyn_solver.FrictionVector();

    return 0;
}
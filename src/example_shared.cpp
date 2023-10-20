#include <smm_screws/robot_shared.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_shared");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;

    robot_shared my_shared_lib;
    if (my_shared_lib.initializeSharedLib()) {ROS_INFO("[example_shared] Initialized Shared Library.");}
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();    
    
    // Test code:
    float q[3] = {0, 0, -1.5436};
    float dq[3] = {0 , -0.500, 0.7854};

    // Calculate Forward Kinematics
    smm_robot_kin_solver.updateJointState(q, dq);
    smm_robot_kin_solver.ForwardKinematics3DOF_2();
    smm_robot_kin_solver.BodyJacobian_Tool_1();
    smm_robot_kin_solver.OperationalSpaceJacobian();

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
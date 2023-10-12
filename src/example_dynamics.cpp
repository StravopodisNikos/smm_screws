#include <smm_screws/ScrewsDynamics.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_dynamics");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;
    /*
     * ROBOT DEFINITION - ADDED BY USER (FOR NOW...)
     */
    // Define the SMM structure properties (extracted by MATLAB analysis)
    Structure2Pseudos robot_def2; // object of the specific structure class, here is specified, later will be selected based the structure yaml file
    RobotAbstractBase *robot_ptr = &robot_def2; // pointer to abstract class(), can access derived class members
    // ACTIVE JOINTS COMPONENTS
    robot_ptr->active_twists[0] << 0, 0, 0, 0, 0, 1;
    robot_ptr->active_twists[1] << 0, 0, 0, 0, -1, 0;
    robot_ptr->active_twists[2] << 0, 0, -1, 0, -1, 0;
    Eigen::Isometry3f gsa10;
    Eigen::Isometry3f gsa20;
    Eigen::Isometry3f gsa30;
    Eigen::Isometry3f gst0;
    gsa10(0,0) = 1.0; gsa10(0,1) = 0.0; gsa10(0,2) = 0.0; gsa10(0,3) = 0.0;
    gsa10(1,0) = 0.0; gsa10(1,1) = 1.0; gsa10(1,2) = 0.0; gsa10(1,3) = 0.0;
    gsa10(2,0) = 0.0; gsa10(2,1) = 0.0; gsa10(2,2) = 1.0; gsa10(2,3) = 0.0;
    gsa10(3,0) = 0.0; gsa10(3,1) = 0.0; gsa10(3,2) = 0.0; gsa10(3,3) = 1.0; 
    robot_ptr->gsai_ptr[0] = &gsa10;
    gsa20(0,0) = 1.0; gsa20(0,1) = 0.0; gsa20(0,2) = 0.0; gsa20(0,3) = 0.0;
    gsa20(1,0) = 0.0; gsa20(1,1) = 0.0; gsa20(1,2) = -1.0; gsa20(1,3) = 0.0;
    gsa20(2,0) = 0.0; gsa20(2,1) = 1.0; gsa20(2,2) = 0.0; gsa20(2,3) = 0.0;
    gsa20(3,0) = 0.0; gsa20(3,1) = 0.0; gsa20(3,2) = 0.0; gsa20(3,3) = 1.0; 
    robot_ptr->gsai_ptr[1] = &gsa20;
    gsa30(0,0) = 1.0; gsa30(0,1) = 0.0; gsa30(0,2) = 0.0; gsa30(0,3) = 1.0;
    gsa30(1,0) = 0.0; gsa30(1,1) = 0.0; gsa30(1,2) = -1.0; gsa30(1,3) = 0.0;
    gsa30(2,0) = 0.0; gsa30(2,1) = 1.0; gsa30(2,2) = 0.0; gsa30(2,3) = 0.0;
    gsa30(3,0) = 0.0; gsa30(3,1) = 0.0; gsa30(3,2) = 0.0; gsa30(3,3) = 1.0; 
    robot_ptr->gsai_ptr[2] = &gsa30;
    gst0(0,0) = 1.0; gst0(0,1) = 0.0; gst0(0,2) = 0.0; gst0(0,3) = 1.5;
    gst0(1,0) = 0.0; gst0(1,1) = 0.0; gst0(1,2) = -1.0; gst0(1,3) = 0.0;
    gst0(2,0) = 0.0; gst0(2,1) = 1.0; gst0(2,2) = 0.0; gst0(2,3) = 0.0;
    gst0(3,0) = 0.0; gst0(3,1) = 0.0; gst0(3,2) = 0.0; gst0(3,3) = 1.0; 
    robot_ptr->gsai_ptr[3] = &gst0;
    // PASSIVE JOINTS COMPONENTS
    robot_def2.META1_PSEUDOS = 1;
    robot_def2.META2_PSEUDOS = 1;
    //ROS_INFO("meta1_1: %d", robot_ptr->get_PSEUDOS_METALINK1());
    //ROS_INFO("meta2_1: %d", robot_ptr->get_PSEUDOS_METALINK2());
    robot_def2.pseudo_angles[0] = 0;
    robot_def2.pseudo_angles[1] = 0;
    robot_def2.passive_twists[0] << -0.00 , 0.1660, -0.025, 1.0, 0.0 , 0.0;
    robot_def2.passive_twists[1] << -0.4685 , 0.00, -0.025, 0.0, -1.0 , 0.0;

    // Masses - COM - Inertias
    float masses[DOF] = {0.5f, 1.0f, 1.0f};
    robot_ptr->link_mass[0] = &masses[0];
    robot_ptr->link_mass[1] = &masses[1];
    robot_ptr->link_mass[2] = &masses[2];
    float inertias[DOF] = {0.5f, 5.0f, 5.0f}; // simplified for example only
    robot_ptr->link_inertia[0] = &inertias[0];
    robot_ptr->link_inertia[1] = &inertias[1];
    robot_ptr->link_inertia[2] = &inertias[2];
    Eigen::Isometry3f gsl10;
    Eigen::Isometry3f gsl20;
    Eigen::Isometry3f gsl30;
    gsl10(0,0) = 1.0; gsl10(0,1) = 0.0; gsl10(0,2) = 0.0; gsl10(0,3) = 0.0;
    gsl10(1,0) = 0.0; gsl10(1,1) = 0.0; gsl10(1,2) = -1.0; gsl10(1,3) = 0.0;
    gsl10(2,0) = 0.0; gsl10(2,1) = 1.0; gsl10(2,2) = 0.0; gsl10(2,3) = 0.0;
    gsl10(3,0) = 0.0; gsl10(3,1) = 0.0; gsl10(3,2) = 0.0; gsl10(3,3) = 1.0; 
    robot_ptr->gsli_ptr[0] = &gsl10;
    gsl20(0,0) = 1.0; gsl20(0,1) = 0.0; gsl20(0,2) = 0.0; gsl20(0,3) = 0.5;
    gsl20(1,0) = 0.0; gsl20(1,1) = 0.0; gsl20(1,2) = -1.0; gsl20(1,3) = 0.0;
    gsl20(2,0) = 0.0; gsl20(2,1) = 1.0; gsl20(2,2) = 0.0; gsl20(2,3) = 0.0;
    gsl20(3,0) = 0.0; gsl20(3,1) = 0.0; gsl20(3,2) = 0.0; gsl20(3,3) = 1.0; 
    robot_ptr->gsli_ptr[1] = &gsl20;
    gsl30(0,0) = 1.0; gsl30(0,1) = 0.0; gsl30(0,2) = 0.0; gsl30(0,3) = 1.25;
    gsl30(1,0) = 0.0; gsl30(1,1) = 0.0; gsl30(1,2) = -1.0; gsl30(1,3) = 0.0;
    gsl30(2,0) = 0.0; gsl30(2,1) = 1.0; gsl30(2,2) = 0.0; gsl30(2,3) = 0.0;
    gsl30(3,0) = 0.0; gsl30(3,1) = 0.0; gsl30(3,2) = 0.0; gsl30(3,3) = 1.0; 
    robot_ptr->gsli_ptr[2] = &gsl30;

    /*
     * Kinematics Essentials
     */
    ScrewsKinematics smm_robot_kin_solver(robot_ptr);
    smm_robot_kin_solver.initializePseudoTfs();
    float q[3] = {0, 0.0658, 2.0236};
    /*
     * Dynamics 
     */
    ScrewsDynamics smm_robot_dyn_solver(robot_ptr);
    smm_robot_dyn_solver.intializeLinkMassMatrices();
    Eigen::Isometry3f* passive_tfs[METALINKS];
    for (size_t i = 0; i < METALINKS; i++)
    {
        passive_tfs[i] = &smm_robot_dyn_solver.gpj[i]; // Now the gai's can be used from ScrewsDynamics methods
    } 
    smm_robot_kin_solver.extractPassiveTfs(passive_tfs); 
    Eigen::Isometry3f* active_tfs[DOF]; // These pointers are uninitialized (they don't yet point to valid memory locations)
    for (size_t i = 0; i < DOF; i++)
    {
        active_tfs[i] = &smm_robot_dyn_solver.gai[i]; // Now the gai's can be used from ScrewsDynamics methods
    }
    smm_robot_kin_solver.extractActiveTfs(q, active_tfs); // Run this function to update active expos in ScrewsDynamics
    
    // Calculate Mass Matrix
    smm_robot_dyn_solver.MM = smm_robot_dyn_solver.MassMatrix();

    return 0;
}
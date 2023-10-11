//#include "ros/ros.h"
#include <smm_screws/ScrewsKinematics.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;

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

    // Tested ForwardKinematicsTCP
    ScrewsKinematics smm_robot_kin_solver(robot_ptr);
    smm_robot_kin_solver.initializePseudoTfs();
    float q[3] = {0, 0.0658, 2.0236};
    smm_robot_kin_solver.ForwardKinematicsTCP(q);

    // Tested ForwardKinematics3DOF_1
    Eigen::Isometry3f* robot_tfs[DOF+1]; // These pointers are uninitialized (they don't yet point to valid memory locations)
    //Eigen::Isometry3f g[DOF+1]; // joint frames tfs @q 
    for (size_t i = 0; i < DOF+1; i++)
    {
        robot_tfs[i] = &smm_robot_kin_solver.g[i];
    }
    smm_robot_kin_solver.ForwardKinematics3DOF_1(q, robot_tfs);
    // Tested ForwardKinematics3DOF_2
    smm_robot_kin_solver.ForwardKinematics3DOF_2(q, robot_tfs);

    // Setting TCP Position to ROS_PARAMETER_SERVER
    Eigen::Vector3f  _pos_tcp_vector = robot_tfs[3]->translation();
    nh.setParam("/x_s_TCP", _pos_tcp_vector.x() );
    nh.setParam("/y_s_TCP", _pos_tcp_vector.y() );
    nh.setParam("/z_s_TCP", _pos_tcp_vector.z() );

    // Initializing kinematic matrices for jacobian calculation
    Eigen::Isometry3f* rel_tfs[DOF+1]; 
    //Eigen::Isometry3f B[DOF+1];
    for (size_t i = 0; i < DOF+1; i++)
    {
        rel_tfs[i] = &smm_robot_kin_solver.B[i];
    }
    smm_robot_kin_solver.initializeRelativeTfs(rel_tfs);
    
    Eigen::Matrix<float, 6, 1>* local_screws[DOF+1]; 
    //Eigen::Matrix<float, 6, 1>  iXi[DOF+1]; 
    for (size_t i = 0; i < DOF+1; i++)
    {
        local_screws[i] = &smm_robot_kin_solver.iXi[i];
    }
    smm_robot_kin_solver.initializeLocalScrewCoordVectors(local_screws);
    
    // Jacobian Spatial 1
    Eigen::Matrix<float, 6, 1>* ptr2Jsp1[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        ptr2Jsp1[i] = &smm_robot_kin_solver.Jsp_t_1[i];
    }
    smm_robot_kin_solver.SpatialJacobian_Tool_1(ptr2Jsp1);

    // Jacobian Spatial 2
    Eigen::Matrix<float, 6, 1>* ptr2Jsp2[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        ptr2Jsp2[i] = &smm_robot_kin_solver.Jsp_t_2[i];
    }
    smm_robot_kin_solver.SpatialJacobian_Tool_2(ptr2Jsp2);

    // Jacobian Body 1
    Eigen::Matrix<float, 6, 1>* ptr2Jbd_t_1[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        ptr2Jbd_t_1[i] = &smm_robot_kin_solver.Jbd_t_1[i];
    }
    smm_robot_kin_solver.BodyJacobian_Tool_1(ptr2Jbd_t_1);

    // Jacobian Body 2
    Eigen::Matrix<float, 6, 1>* ptr2Jbd_t_2[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        ptr2Jbd_t_2[i] = &smm_robot_kin_solver.Jbd_t_2[i];
    }
    smm_robot_kin_solver.BodyJacobian_Tool_2(ptr2Jbd_t_2);

    // Body Jacobians Frames:
    Eigen::Matrix<float, 6, 1>** PTR2BodyJacobiansFrames[DOF+1];
    for (int i = 0; i < DOF+1; ++i) {
        PTR2BodyJacobiansFrames[i] = new Eigen::Matrix<float, 6, 1>*[DOF];
        for (int j = 0; j < DOF; ++j) {
            PTR2BodyJacobiansFrames[i][j] = new Eigen::Matrix<float, 6, 1>;
        }
    }
    smm_robot_kin_solver.BodyJacobians(PTR2BodyJacobiansFrames);
    // Dont deallocate the memory space because matrices will be lost!
    /*
    for (int i = 0; i < DOF+1; ++i) {
        for (int j = 0; j < DOF; ++j) {
            delete PTR2BodyJacobiansFrames[i][j];
        }
        delete[] PTR2BodyJacobiansFrames[i];
    }
    */
    float ddq[3] = {10.025 , -1.8294, 5.0236};
    float dq[3] = {0.25 , 0.8954, -2.0236};
    // 1st Time Derivative of Body Jacobians
    Eigen::Matrix<float, 6, 1>* ptr2dJbd_t_1[DOF];
    Eigen::Matrix<float, 6, 1>* ptr2dJbd_t_2[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        ptr2dJbd_t_1[i] = &smm_robot_kin_solver.dJbd_t_1[i];
        ptr2dJbd_t_2[i] = &smm_robot_kin_solver.dJbd_t_2[i];
    }   
    smm_robot_kin_solver.DtBodyJacobian_Tool_1(dq, PTR2BodyJacobiansFrames, ptr2dJbd_t_1);
    smm_robot_kin_solver.DtBodyJacobian_Tool_2(dq, PTR2BodyJacobiansFrames, ptr2dJbd_t_2);

    // Operational Space Jacobian
    smm_robot_kin_solver.OperationalSpaceJacobian(smm_robot_kin_solver.Jop);

    // Velocity twists
    smm_robot_kin_solver.ToolVelocityTwist(ScrewsKinematics::JacobianSelection::SPATIAL , dq, smm_robot_kin_solver.Vsp_tool_twist);
    smm_robot_kin_solver.ToolVelocityTwist(ScrewsKinematics::JacobianSelection::BODY , dq, smm_robot_kin_solver.Vbd_tool_twist);
    // Acceleration twists
    smm_robot_kin_solver.DtToolVelocityTwist(ScrewsKinematics::JacobianSelection::SPATIAL , ddq, dq, smm_robot_kin_solver.dVsp_tool_twist);
    smm_robot_kin_solver.DtToolVelocityTwist(ScrewsKinematics::JacobianSelection::BODY , ddq, dq, smm_robot_kin_solver.dVbd_tool_twist);
    
    // Operational Space Spatial Velocity & acceleration
    smm_robot_kin_solver.CartesianVelocity_twist(smm_robot_kin_solver.Vop);
    smm_robot_kin_solver.CartesianAcceleration_twist(smm_robot_kin_solver.Aop, smm_robot_kin_solver.Vop);

    return 0;
}

#include "smm_screws/robot_shared.h"

// Constructor for backward compatibility
robot_shared::robot_shared(RobotStructure structure)
    : smm_robot_kin_solver(nullptr), // Initialize with null pointers
      smm_robot_dyn_solver(nullptr),
      smm_robot_viz_solver(nullptr) // Default constructor without NodeHandle
{
    switch (structure) {
        case STRUCTURE_2_PSEUDOS:
            robot_ptr = new Structure2Pseudos();
            break;
        case STRUCTURE_3_PSEUDOS:
            robot_ptr = new Structure3Pseudos();
            break;
        default:
            robot_ptr = new Structure3Pseudos();
            break;
    }

    smm_robot_kin_solver = ScrewsKinematics(robot_ptr);
    smm_robot_dyn_solver = ScrewsDynamics(robot_ptr);
    smm_robot_viz_solver = ScrewsVisualization(robot_ptr); // Default constructor
}

// Constructor with NodeHandle for visualization
robot_shared::robot_shared(RobotStructure structure, ros::NodeHandle& nh)
    : smm_robot_kin_solver(nullptr), // Initialize with null pointers
      smm_robot_dyn_solver(nullptr),
      smm_robot_viz_solver(nullptr, nh)
{
    switch (structure) {
        case STRUCTURE_2_PSEUDOS:
            robot_ptr = new Structure2Pseudos();
            break;
        case STRUCTURE_3_PSEUDOS:
            robot_ptr = new Structure3Pseudos();
            break;
        default:
            throw std::invalid_argument("[robot_shared] Unsupported robot structure");
    }

    smm_robot_kin_solver = ScrewsKinematics(robot_ptr);
    smm_robot_dyn_solver = ScrewsDynamics(robot_ptr);
    smm_robot_viz_solver = ScrewsVisualization(robot_ptr, nh);
}

robot_shared::~robot_shared() {
    delete robot_ptr;
}

bool robot_shared::initializeSharedLib() {
    smm_robot_kin_solver.initializePseudoTfs();
    smm_robot_kin_solver.initializeAnatomyActiveTwists();
    smm_robot_dyn_solver.intializeLinkMassMatrices();
    smm_robot_kin_solver.extractPassiveTfs(smm_robot_dyn_solver.ptr2passive_tfs);
    smm_robot_kin_solver.initializeRelativeTfs();
    smm_robot_kin_solver.initializeLocalScrewCoordVectors();
    return true;
}

ScrewsKinematics& robot_shared::get_screws_kinematics_solver() {
    return smm_robot_kin_solver;
}

ScrewsDynamics& robot_shared::get_screws_dynamics_solver() {
    return smm_robot_dyn_solver;
}

ScrewsVisualization& robot_shared::get_screws_visualization_solver() {
    return smm_robot_viz_solver;
}

RobotAbstractBase* robot_shared::get_robot_ptr() {
    return robot_ptr;
}
#include "smm_screws/robot_shared.h"

// Default constructor for backward compatibility
robot_shared::robot_shared()
    : smm_robot_kin_solver(nullptr), // Initialize with null pointers
      smm_robot_dyn_solver(nullptr),
      smm_robot_viz_solver(nullptr)
{
    robot_ptr = new Structure3Pseudos();
    smm_robot_kin_solver = ScrewsKinematics(robot_ptr);
    smm_robot_dyn_solver = ScrewsDynamics(robot_ptr);
    smm_robot_viz_solver = ScrewsVisualization(robot_ptr);
}

// Constructor with structure type
robot_shared::robot_shared(RobotStructure structure)
    : smm_robot_kin_solver(nullptr), // Initialize with null pointers
      smm_robot_dyn_solver(nullptr),
      smm_robot_viz_solver(nullptr)
{
    switch (structure) {
        case STRUCTURE_2_PSEUDOS:
            robot_ptr = new Structure2Pseudos();
            break;
        case STRUCTURE_3_PSEUDOS:
            robot_ptr = new Structure3Pseudos();
            break;
        case STRUCTURE_4_PSEUDOS:
            robot_ptr = new Structure4Pseudos();
            break;            
        default:
            robot_ptr = new FixedStructure();
            break;
    }

    smm_robot_kin_solver = ScrewsKinematics(robot_ptr);
    smm_robot_dyn_solver = ScrewsDynamics(robot_ptr);
    smm_robot_viz_solver = ScrewsVisualization(robot_ptr);
}

// Constructor with structure and nodehandle pointers
robot_shared::robot_shared(RobotAbstractBase* robot_ptr, ros::NodeHandle& nh)
    : robot_ptr(robot_ptr),
      smm_robot_kin_solver(robot_ptr),
      smm_robot_dyn_solver(robot_ptr),
      smm_robot_viz_solver(robot_ptr, nh) 
{
    // Initialization code if needed
}

robot_shared::~robot_shared() {
    delete robot_ptr;
}

bool robot_shared::initializeSharedLib() {
    smm_robot_kin_solver.initializePseudoTfs(); // initializes _Pi
    smm_robot_kin_solver.initializeReferenceAnatomyActiveTwists(); // needs _Pi, initializes _ptr2abstract->active_twists
    smm_robot_kin_solver.initializeReferenceAnatomyActiveTfs(); // needs _Pi, initializes _ptr2abstract->gsai_ptr
    //smm_robot_kin_solver.initializeAnatomyActiveTwists();
    smm_robot_dyn_solver.intializeLinkMassMatrices();
    //smm_robot_kin_solver.extractPassiveTfs(smm_robot_dyn_solver.ptr2passive_tfs); // [25-7-24] Removed. ScrewsDynamics must not get passive expos, because works with anat twists! HUGE BUG
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
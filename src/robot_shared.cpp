#include "smm_screws/robot_shared.h"

/*
robot_shared::robot_shared()
    : robot_def2(),
      robot_ptr(&robot_def2),
      smm_robot_kin_solver(robot_ptr),
      smm_robot_dyn_solver(robot_ptr) {}

robot_shared::~robot_shared() {}
*/
// [13-7-24] Modified to allow derived class customization
robot_shared::robot_shared(RobotStructure structure) {
    // Initialize the appropriate robot structure
    switch (structure) {
        case STRUCTURE_2_PSEUDOS:
            robot_ptr = new Structure2Pseudos();
            break;
        case STRUCTURE_3_PSEUDOS:
            robot_ptr = new Structure3Pseudos();
            break;
        // case STRUCTURE_4_PSEUDOS: next!
        default:
            throw std::invalid_argument("[robot_shared] Unsupported robot structure");
    }
    
    smm_robot_kin_solver = ScrewsKinematics(robot_ptr);
    smm_robot_dyn_solver = ScrewsDynamics(robot_ptr);
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
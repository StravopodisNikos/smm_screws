#include "smm_screws/robot_shared.h"

robot_shared::robot_shared()
    : robot_def2(),
      robot_ptr(&robot_def2),
      smm_robot_kin_solver(robot_ptr),
      smm_robot_dyn_solver(robot_ptr) {}

robot_shared::~robot_shared() {}

bool robot_shared::initializeSharedLib() {
    smm_robot_kin_solver.initializePseudoTfs();
    smm_robot_dyn_solver.intializeLinkMassMatrices();
    smm_robot_kin_solver.extractPassiveTfs(smm_robot_dyn_solver.ptr2passive_tfs);
    //robot_ptr->initializeActiveElements();
    return true;
}

ScrewsKinematics& robot_shared::get_screws_kinematics_solver() {
    return smm_robot_kin_solver;
}

ScrewsDynamics& robot_shared::get_screws_dynamics_solver() {
    return smm_robot_dyn_solver;
}
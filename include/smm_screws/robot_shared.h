#ifndef ROBOT_SHARED_H
#define ROBOT_SHARED_H

#include "smm_screws/ScrewsDynamics.h"

class robot_shared
{
private:
    Structure2Pseudos robot_def2;
    RobotAbstractBase *robot_ptr = &robot_def2;
    ScrewsKinematics smm_robot_kin_solver;
    ScrewsDynamics smm_robot_dyn_solver;

public:
    robot_shared(/* args */);
    ~robot_shared();

    bool initializeSharedLib();
    ScrewsKinematics& get_screws_kinematics_solver();
    ScrewsDynamics& get_screws_dynamics_solver();
};
#endif
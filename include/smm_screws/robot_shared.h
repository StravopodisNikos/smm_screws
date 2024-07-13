#ifndef ROBOT_SHARED_H
#define ROBOT_SHARED_H

#include "smm_screws/ScrewsDynamics.h"

class robot_shared
{
private:
    //Structure2Pseudos robot_def2;
    //RobotAbstractBase *robot_ptr = &robot_def2;
    RobotAbstractBase *robot_ptr;
    ScrewsKinematics smm_robot_kin_solver;
    ScrewsDynamics smm_robot_dyn_solver;

public:
    // Enumeration for supported robot structures
    enum RobotStructure {
        STRUCTURE_2_PSEUDOS,
        STRUCTURE_3_PSEUDOS
    };

    //robot_shared(/* args */);
    robot_shared(RobotStructure structure = STRUCTURE_3_PSEUDOS);
    ~robot_shared();

    bool initializeSharedLib();
    ScrewsKinematics& get_screws_kinematics_solver();
    ScrewsDynamics& get_screws_dynamics_solver();
};
#endif
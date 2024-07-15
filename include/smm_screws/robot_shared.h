#ifndef ROBOT_SHARED_H
#define ROBOT_SHARED_H

#include "smm_screws/ScrewsDynamics.h"
#include "smm_screws/ScrewsVisualization.h"

class robot_shared
{
private:
    //Structure2Pseudos robot_def2;
    //RobotAbstractBase *robot_ptr = &robot_def2;
    RobotAbstractBase *robot_ptr;
    ScrewsKinematics smm_robot_kin_solver;
    ScrewsDynamics smm_robot_dyn_solver;
    ScrewsVisualization smm_robot_viz_solver;

public:
    // Enumeration for supported robot structures
    enum RobotStructure {
        STRUCTURE_2_PSEUDOS,
        STRUCTURE_3_PSEUDOS
    };

    //robot_shared(/* args */);
    robot_shared(RobotStructure structure = STRUCTURE_3_PSEUDOS); // keep for backward compatibility
    robot_shared(RobotStructure structure, ros::NodeHandle& nh); //  removed default value for arg1 = STRUCTURE_3_PSEUDOS
    ~robot_shared();

    bool initializeSharedLib();
    ScrewsKinematics& get_screws_kinematics_solver();
    ScrewsDynamics& get_screws_dynamics_solver();
    ScrewsVisualization& get_screws_visualization_solver();
    RobotAbstractBase* get_robot_ptr();
};
#endif
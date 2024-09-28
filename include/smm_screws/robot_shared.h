#ifndef ROBOT_SHARED_H
#define ROBOT_SHARED_H

#include "smm_screws/ScrewsDynamics.h"
#include "smm_screws/ScrewsVisualization.h"
#include <hardware_interface/robot_hw.h> // [31-7-24] fighting idosc
//class robot_shared
class robot_shared : public hardware_interface::RobotHW
{
private:
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

    robot_shared();  // Default constructor for backward compatibility
    robot_shared(RobotStructure structure); // Constructor with structure type
    robot_shared(RobotAbstractBase* robot_ptr, ros::NodeHandle& nh);
    ~robot_shared();

    bool initializeSharedLib();
    ScrewsKinematics& get_screws_kinematics_solver();
    ScrewsDynamics& get_screws_dynamics_solver();
    ScrewsVisualization& get_screws_visualization_solver();
    RobotAbstractBase* get_robot_ptr();
};

#endif // ROBOT_SHARED_H
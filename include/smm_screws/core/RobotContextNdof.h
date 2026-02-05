#ifndef ROBOT_CONTEXT_NDOF_H
#define ROBOT_CONTEXT_NDOF_H

#include <memory>
#include <string>

#include "smm_screws/core/RobotAbstractBaseNdof.h"
#include "smm_screws/core/ScrewsKinematicsNdof.h"

class RobotContextNdof
{
public:
    // RobotContextNdof owns the Ndof robot model and wires it into ScrewsKinematicsNdof
    explicit RobotContextNdof(std::unique_ptr<RobotAbstractBaseNdof> robot);

    ~RobotContextNdof() = default;

    // Keep the same pattern as the 3-DOF version; implementation can later
    // delegate to whatever Ndof initialization you define.
    bool initializeSharedLib();

    ScrewsKinematicsNdof & get_kinematics();
    RobotAbstractBaseNdof * get_robot();

private:
    // Owns the Ndof robot model
    std::unique_ptr<RobotAbstractBaseNdof> robot_;

    // Ndof kinematics (no dynamics yet)
    ScrewsKinematicsNdof kin_;
};

#endif // ROBOT_CONTEXT_NDOF_H

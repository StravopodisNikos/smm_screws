#include "smm_screws/core/RobotContextNdof.h"
#include <stdexcept>

RobotContextNdof::RobotContextNdof(std::unique_ptr<RobotAbstractBaseNdof> robot)
: robot_(std::move(robot))
, kin_(robot_.get())                // pass raw ptr into ScrewsKinematicsNdof
{
    if (!robot_) {
        throw std::invalid_argument("[RobotContextNdof] robot is null");
    }
}

ScrewsKinematicsNdof & RobotContextNdof::get_kinematics()
{
    return kin_;
}

RobotAbstractBaseNdof * RobotContextNdof::get_robot()
{
    return robot_.get();
}

// For now you can just return true or mirror whatever your 3-DOF version does.
bool RobotContextNdof::initializeSharedLib()
{
    kin_ndof_.initializePseudoTfs();
    kin_ndof_.initializeReferenceAnatomyActiveTwists();
    kin_ndof_.initializeReferenceAnatomyActiveTfs();
    kin_.initializeRelativeTfs();
    kin_.initializeLocalScrewCoordVectors();

    return true;
}
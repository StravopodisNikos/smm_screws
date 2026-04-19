#include "smm_screws/core/RobotContextNdof.h"
#include <stdexcept>

RobotContextNdof::RobotContextNdof(std::unique_ptr<RobotAbstractBaseNdof> robot)
: robot_(std::move(robot))
, kin_(robot_.get())
, dyn_(robot_.get())
{
    if (!robot_) {
        throw std::invalid_argument("[RobotContextNdof] robot is null");
    }
}

ScrewsKinematicsNdof & RobotContextNdof::get_kinematics()
{
    return kin_;
}

ScrewsDynamicsNdof & RobotContextNdof::get_dynamics()
{
    return dyn_;
}

RobotAbstractBaseNdof * RobotContextNdof::get_robot()
{
    return robot_.get();
}

void RobotContextNdof::_initializeKinematicState(ScrewsKinematicsNdof & obj)
{
    obj.initializePseudoTfs();
    obj.initializeReferenceAnatomyActiveTwists();
    obj.initializeReferenceAnatomyActiveTfs();
    obj.initializeRelativeTfs();
    obj.initializeLocalScrewCoordVectors();
    obj.initializeSpatialJointScrewCoordVectors();
    obj.initializeHomeAnatomyActiveTfs();
    obj.initializeHomeAnatomyCOMTfs();
}

bool RobotContextNdof::initializeSharedLib()
{
    // -----------------------------
    // KINEMATICS OBJECT
    // -----------------------------    
    _initializeKinematicState(kin_);
    // -----------------------------
    // DYNAMICS OBJECT
    // -----------------------------    
    _initializeKinematicState(dyn_);

    // Dynamics-specific initialization
    dyn_.initializeLinkMassMatrices();

    return true;
}
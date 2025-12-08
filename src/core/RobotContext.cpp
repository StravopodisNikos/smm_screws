#include "smm_screws/core/RobotContext.h"

RobotContext::RobotContext(std::unique_ptr<RobotAbstractBase> robot)
: robot_(std::move(robot)),
  kin_(robot_.get()),
  dyn_(robot_.get())
{
}

bool RobotContext::initializeSharedLib()
{
    // EXACTLY like ROS1 robot_shared
    kin_.initializePseudoTfs();
    kin_.initializeReferenceAnatomyActiveTwists();
    kin_.initializeReferenceAnatomyActiveTfs();
    dyn_.intializeLinkMassMatrices();
    kin_.initializeRelativeTfs();
    kin_.initializeLocalScrewCoordVectors();

    return true;
}

ScrewsKinematics & RobotContext::get_kinematics()
{
    return kin_;
}

ScrewsDynamics & RobotContext::get_dynamics()
{
    return dyn_;
}

RobotAbstractBase * RobotContext::get_robot()
{
    return robot_.get();
}

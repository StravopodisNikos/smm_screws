#ifndef ROBOT_CONTEXT_H
#define ROBOT_CONTEXT_H

#include <memory>
#include <string>

#include "smm_screws/core/RobotAbstractBase.h"
#include "smm_screws/core/ScrewsKinematics.h"
#include "smm_screws/core/ScrewsDynamics.h"

class RobotContext
{
public:

    explicit RobotContext(std::unique_ptr<RobotAbstractBase> robot);

    ~RobotContext() = default;

    bool initializeSharedLib();

    ScrewsKinematics & get_kinematics();
    ScrewsDynamics  & get_dynamics();
    RobotAbstractBase * get_robot();

private:
    std::unique_ptr<RobotAbstractBase> robot_;   // owns robot model
    ScrewsKinematics kin_;
    ScrewsDynamics dyn_;
};

#endif

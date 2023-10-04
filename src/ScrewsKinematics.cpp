#include "smm_screws/ScrewsKinematics.h"

ScrewsKinematics::ScrewsKinematics() {};

ScrewsKinematics::ScrewsKinematics(const std::shared_ptr<RobotAbstractBase>& robot_def)
    : _robot_def(robot_def) {
    //PseudoTfs(); // Initialize the product exponentials
}
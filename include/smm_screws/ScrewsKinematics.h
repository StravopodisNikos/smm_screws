#ifndef SCREWS_KINEMATICS_H
#define SCREWS_KINEMATICS_H

//#include <Eigen/Dense>
//#include <Eigen/Core>
#include <memory>
#include "smm_screws/ScrewsMain.h"
#include "smm_screws/RobotAbstractBase.h"

/*
 *  C++ Library(integrated in a ros_pkg) for computation of forward kinematics, Task-space End-Effector 
 *  Velocity & Acceleration, Manipulator Jacobians, First-Order Derivatives of Manipulator Jacobians.
 *
 *  Special Thanks and Citations:
 *  [1] Murray, R. M., Li, Z., Sastry, S. S., & Sastry, S. S. (1994). A mathematical introduction to robotic manipulation. CRC press.
 *  [2] Müller, A. (2018). Screw and Lie group theory in multibody kinematics: Motion representation and recursive kinematics of tree-topology systems. Multibody System Dynamics, 43(1), 37-70.
 *  [3] Müller, A. (2018). Screw and Lie group theory in multibody dynamics: recursive algorithms and equations of motion of tree-topology systems. Multibody System Dynamics, 42(2), 219-248.
 *   * 
 *  ros_pkg smm_screws, test file: example.cpp
 * 
 *  Author: Nikos Stravopodis, PhD Candidate
 */
 
class ScrewsKinematics: public ScrewsMain {
	public:
		ScrewsKinematics();
		ScrewsKinematics(const std::shared_ptr<RobotAbstractBase>& robot_def);

	private:
		std::shared_ptr<RobotAbstractBase> _robot_def;

};

#endif // SCREWS_KINEMATICS_H

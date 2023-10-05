#ifndef SCREWS_KINEMATICS_H
#define SCREWS_KINEMATICS_H

#include <memory>
#include <iostream>
#include "smm_screws/ScrewsMain.h"
#include "smm_screws/RobotAbstractBase.h"
#include "ros/ros.h"

#define DOF 3
#define METALINKS 2

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
		ScrewsKinematics(RobotAbstractBase *ptr2abstract);
		void extractPseudoTfs(); // Calculates the exponentials of the metamorphic links, updates _Pi[] private member
		void ForwardKinematicsTCP(float *q); // Calculates the tf of the {T} frame, updates _gst private member

	private:
		RobotAbstractBase *_ptr2abstract; // pointer that has memory address of the abstract class (defined structure parameters of the smm)
		uint8_t _total_pseudojoints;
		uint8_t _meta1_pseudojoints;
		uint8_t _meta2_pseudojoints;
		Eigen::Isometry3f _last_expo;
		uint8_t _last_twist_cnt;
		Eigen::Isometry3f _Pi[METALINKS];
		Eigen::Isometry3f _gst;
};

#endif // SCREWS_KINEMATICS_H

#ifndef SCREWS_KINEMATICS_H
#define SCREWS_KINEMATICS_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include "smm_screws/ScrewsMain.h"

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
		Eigen::Isometry3f extractRelativeTf(Eigen::Isometry3f Ai, Eigen::Isometry3f Ai_1); // Computes: Bi [Mueller], g_i_i-1(0) [Murray]
		Eigen::Matrix<float, 6, 1> extractLocalScrewCoordVector(Eigen::Isometry3f Ai, Eigen::Matrix<float, 6, 1> Yi); // Computes the inverse of 1st relation in eq.(95),p.241,[3]
		Eigen::Matrix<float, 6, 1> extractLocalScrewPrevCoordVector(Eigen::Isometry3f Bi, Eigen::Matrix<float, 6, 1> iXi); // Computes eq.(7)/p.44,[2] 
	
	private:
		Eigen::Isometry3f _Tf;
		Eigen::Matrix<float, 6, 1> _Yi;
		Eigen::Matrix<float, 6, 1> _iXi;
		Eigen::Matrix<float, 6, 1> _i_1Xi;
		Eigen::Matrix<float, 6, 1> _iXi_1;
		Eigen::Matrix<float, 6, 6> _ad;
};

#endif // SCREWS_KINEMATICS_H

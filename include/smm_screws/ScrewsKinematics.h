#ifndef SCREWS_KINEMATICS_H
#define SCREWS_KINEMATICS_H

#include <memory>
#include <iostream>
#include "smm_screws/ScrewsMain.h"
#include "smm_screws/RobotAbstractBase.h"
#include "ros/ros.h"
#include <ros/console.h>

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
		// Initialize kinematic data @ zero configuration
		void initializeRelativeTfs(Eigen::Isometry3f* Bi[DOF+1]);
		void initializeLocalScrewCoordVectors(Eigen::Matrix<float, 6, 1> *iXi[DOF+1]);
		void initializePseudoTfs(); // Calculates the exponentials of the metamorphic links, updates _Pi[] private member
		// Extract kinematic data @ current configuration 
		void ForwardKinematicsTCP(float *q); // Calculates the tf of the {T} frame, updates _gst private member
		void ForwardKinematics3DOF_1(float *q, Eigen::Isometry3f* gs_a_i[DOF+1]);
		void ForwardKinematics3DOF_2(float *q, Eigen::Isometry3f* gs_a_i[DOF+1]);
		void SpatialJacobian_1(float *q, Eigen::Matrix<float, 6, 1> *Jsp1[DOF]) ;
		void SpatialJacobian_2(float *q, Eigen::Matrix<float, 6, 1> *Jsp2[DOF]) ;

		// Public Kinematic data members
		Eigen::Matrix<float, 6, 1> iXi[DOF+1];
		Eigen::Isometry3f g[DOF+1];
		Eigen::Isometry3f B[DOF+1]; 
		Eigen::Matrix<float, 6, 1> Jsp1[DOF];
		Eigen::Matrix<float, 6, 1> Jsp2[DOF];

	private:
		RobotAbstractBase *_ptr2abstract; // pointer that has memory address of the abstract class (defined structure parameters of the smm)
		uint8_t _total_pseudojoints;
		uint8_t _meta1_pseudojoints;
		uint8_t _meta2_pseudojoints;
		Eigen::Isometry3f _last_expo;
		uint8_t _last_twist_cnt;
		Eigen::Isometry3f _Pi[METALINKS];
		Eigen::Isometry3f _gst;
		Eigen::Isometry3f _Bi; 
		Eigen::Matrix<float, 6, 1> _X;
		Eigen::Vector3f _trans_vector;
		bool _debug_verbosity;
		Eigen::Matrix<float, 6, 6> _ad;

		// Auxiliary functions
		void printIsometryMatrix(const Eigen::Isometry3f& matrix);
		void print6nMatrix(Eigen::Matrix<float, 6, 1>* matrices[], const int n);
};		

#endif // SCREWS_KINEMATICS_H

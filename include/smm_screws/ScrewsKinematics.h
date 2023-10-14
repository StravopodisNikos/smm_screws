#ifndef SCREWS_KINEMATICS_H
#define SCREWS_KINEMATICS_H

#include <memory>
#include <iostream>
#include <cstdlib>
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
		enum class JacobianSelection { SPATIAL, BODY};
		typedef JacobianSelection typ_jacobian;

		ScrewsKinematics();
		ScrewsKinematics(RobotAbstractBase *ptr2abstract);
		// Initialize kinematic data @ zero configuration
		void extractPassiveTfs(Eigen::Isometry3f* passive_expos[METALINKS]);
		void initializeRelativeTfs(Eigen::Isometry3f* Bi[DOF+1]);
		void initializeLocalScrewCoordVectors(Eigen::Matrix<float, 6, 1> *iXi[DOF+1]);
		void initializePseudoTfs(); // Calculates the exponentials of the metamorphic links, updates _Pi[] private member
		// Extract kinematic data @ current configuration 
		//void extractActiveTfs(float *q, Eigen::Isometry3f* active_expos[DOF]);
		void extractActiveTfs(float *q, Eigen::Isometry3f* active_expos[DOF]);
		Eigen::Isometry3f* extractActiveTfs(float *q);
		void ForwardKinematicsTCP(float *q); // Calculates the tf of the {T} frame, updates _gst private member
		void ForwardKinematics3DOF_1(float *q, Eigen::Isometry3f* gs_a_i[DOF+1]);
		void ForwardKinematics3DOF_2(float *q, Eigen::Isometry3f* gs_a_i[DOF+1]);
		void SpatialJacobian_Tool_1(Eigen::Matrix<float, 6, 1> *Jsp_t_1[DOF]); // Returns {T} frame Spatial Jacobian
		void SpatialJacobian_Tool_2(Eigen::Matrix<float, 6, 1> *Jsp_t_2[DOF]); // Returns {T} frame Spatial Jacobian
		void BodyJacobians(Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[DOF+1]); // Returns Body Jacobians calculated @ active joint frames and the {T} frame
		void BodyJacobian_Tool_1(Eigen::Matrix<float, 6, 1> *Jbd_t_1[DOF]); // Returns {T} frame Body Jacobian
		void BodyJacobian_Tool_2(Eigen::Matrix<float, 6, 1> *Jbd_t_2[DOF]); // Returns {T} frame Body Jacobian
		void ToolVelocityTwist(typ_jacobian jacob_selection, float *dq, Eigen::Matrix<float, 6, 1> &Vtwist ); // Calculates {T} frame Velocity twists
		void DtSpatialJacobian_Tool_1( float *dq, Eigen::Matrix<float, 6, 1> *Jsp_t_1[DOF], Eigen::Matrix<float, 6, 1> *dJsp_t_1[DOF] ); // Time derivative of spatial jacobian
		void DtBodyJacobian_Tool_1( float *dq, Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[DOF+1], Eigen::Matrix<float, 6, 1> *dJbd_t_1[DOF]) ;		
		void DtBodyJacobian_Tool_2( float *dq, Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[DOF+1], Eigen::Matrix<float, 6, 1> *dJbd_t_2[DOF]) ;
		void OperationalSpaceJacobian(Eigen::Matrix3f &Jop_t);
		void DtToolVelocityTwist(typ_jacobian jacob_selection, float *ddq, float *dq, Eigen::Matrix<float, 6, 1> &dVtwist );
		void CartesianVelocity_twist(Eigen::Vector4f &v_qs); // Returns spatial velocity {T} using the Spatial Velocity twist
		void CartesianVelocity_jacob(float *dq, Eigen::Vector3f &v_qs); // Returns spatial velocity {T} using the Operational Space Jacobian
		void CartesianVelocity_jacob(float *dq, Eigen::Vector4f &v_qs);
		void CartesianAcceleration_twist(Eigen::Vector4f &a_qs, Eigen::Vector4f v_qs );
		void CartesianAcceleration_jacob(float *ddq, float *dq, Eigen::Vector3f &a_qs);
		void CartesianAcceleration_jacob(float *ddq, float *dq, Eigen::Vector4f &a_qs);		
		void DtOperationalSpaceJacobian(Eigen::Matrix3f &dJop_t);

		// Public Kinematic data members
		
		Eigen::Matrix<float, 6, 1> iXi[DOF+1];
		Eigen::Isometry3f g[DOF+1];
		Eigen::Isometry3f B[DOF+1]; 
		Eigen::Matrix<float, 6, DOF> Jsp63; // the concatenated form of the Spatial Jacobian
		Eigen::Matrix<float, 6, DOF> Jbd63;
		Eigen::Matrix<float, 6, DOF> dJbd63;
		Eigen::Matrix<float, 6, 1> Jsp_t_1[DOF];
		Eigen::Matrix<float, 6, 1> Jsp_t_2[DOF];
		Eigen::Matrix<float, 6, 1> BodyJacobiansFrames[DOF+1][DOF];
		Eigen::Matrix<float, 6, 1> Jbd_t_1[DOF];
		Eigen::Matrix<float, 6, 1> Jbd_t_2[DOF];
		Eigen::Matrix<float, 6, 1> dJsp_t_1[DOF];
		Eigen::Matrix<float, 6, 1> dJbd_t_1[DOF];
		Eigen::Matrix<float, 6, 1> dJbd_t_2[DOF];
		Eigen::Matrix<float, 6, 1> Vsp_tool_twist;
		Eigen::Matrix<float, 6, 1> Vbd_tool_twist;
		Eigen::Matrix3f dJbd_pos;
		Eigen::Matrix3f Jbd_pos;
		Eigen::Matrix3f dJop; // Time Derivative of the Operational Space Jacobian
		Eigen::Matrix3f Jop; // Operational Space Jacobian
		Eigen::Matrix<float, 6, 1> dVsp_tool_twist;
		Eigen::Matrix<float, 6, 1> dVbd_tool_twist;
		Eigen::Vector4f Vop4;
		Eigen::Vector4f Aop4;
		Eigen::Matrix3f dRst; 
		void printIsometryMatrix(const Eigen::Isometry3f& matrix);
	private:
		RobotAbstractBase *_ptr2abstract; // pointer that has memory address of the abstract class (defined structure parameters of the smm)
		uint8_t _total_pseudojoints;
		uint8_t _meta1_pseudojoints;
		uint8_t _meta2_pseudojoints;
		Eigen::Isometry3f _last_expo;
		uint8_t _last_twist_cnt;
		Eigen::Isometry3f _gst;
		Eigen::Isometry3f _Bi; 
		Eigen::Matrix<float, 6, 1> _X;
		Eigen::Vector3f _trans_vector;
		bool _debug_verbosity;
		Eigen::Matrix<float, 6, 6> _ad;  // adjoint(screw product) result
		Eigen::Matrix<float, 6, 6> _scp; // spatial cross profuct result
		Eigen::Matrix4f _twist_se3;
		Eigen::Isometry3f _active_expos[DOF];
		Eigen::Isometry3f _Pi[METALINKS];

		// Set functions for elememts used in calculations
		void setExponentials(float *q);		
		void setBodyPositionJacobian();
		void setDerivativeBodyPositionJacobian();
		void setDtRotationMatrix();

		// Auxiliary functions for printing
		
		void print6nMatrix(Eigen::Matrix<float, 6, 1>* matrices[], const int n);
		void printTwist(Eigen::Matrix<float, 6, 1> Twist);		
};		

#endif // SCREWS_KINEMATICS_H

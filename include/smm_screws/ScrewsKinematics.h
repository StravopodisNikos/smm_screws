#ifndef SCREWS_KINEMATICS_H
#define SCREWS_KINEMATICS_H

#include <memory>
#include <iostream>
#include <cstdlib>
#include <stdexcept> // for exception error handling
#include <memory>    // for unique_ptr
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

		// Update function to store new data from ROS topics
		void updateJointState(float *q_new, float *dq_new, float *ddq_new);
		void updateJointState(float *q_new, float *dq_new);
		// Initialize kinematic data @ zero configuration
		void extractPassiveTfs(Eigen::Isometry3f* passive_expos[robot_params::METALINKS]);
		void initializeRelativeTfs(Eigen::Isometry3f* Bi[robot_params::DOF+1]);
		void initializeRelativeTfs();
		void initializeLocalScrewCoordVectors(Eigen::Matrix<float, 6, 1> *iXi[robot_params::DOF+1]);
		void initializeLocalScrewCoordVectors();
		void initializePseudoTfs(); // Calculates the exponentials of the metamorphic links, updates _Pi[] private member
		void initializeReferenceAnatomyActiveTwists(); // added for backward compatibility after auto yaml kinematics extraction
		void initializeReferenceAnatomyActiveTfs(); // added for backward compatibility after auto yaml kinematics extraction
		void initializeAnatomyActiveTwists();
		// Extract kinematic data @ current configuration 
		//void extractActiveTfs(float *q, Eigen::Isometry3f* active_expos[DOF]);
		void extractActiveTfs(float *q, Eigen::Isometry3f* active_expos[robot_params::DOF]);
		void extractActiveTfsAnat(float *q, Eigen::Isometry3f* active_expos[robot_params::DOF]);
		//Eigen::Isometry3f* extractActiveTfs(float *q);
		void ForwardKinematicsTCP(float *q); // Calculates the tf of the {T} frame, updates _gst private member
		Eigen::Vector3f updatePositionTCP(float *q);
		Eigen::Vector3f updatePositionTCP(Eigen::Matrix<float, 3, 1>& q); 
		Eigen::Vector3f updateSpatialVelocityTCP(float *q, float *dq);
		void ForwardKinematicsTCP();
		void ForwardKinematics3DOF_1(float *q, Eigen::Isometry3f* gs_a_i[robot_params::DOF+1]);
		void ForwardKinematics3DOF_1();
		void ForwardKinematics3DOF_2(float *q, Eigen::Isometry3f* gs_a_i[robot_params::DOF+1]);
		void ForwardKinematics3DOF_2();
		void ForwardKinematicsComFrames3DOF_2(float *q, Eigen::Isometry3f* gs_l_i[robot_params::DOF]);
		void ForwardKinematicsComFrames3DOF_2();
		void SpatialJacobian_Tool_1(Eigen::Matrix<float, 6, 1> *Jsp_t_1[robot_params::DOF]); // Returns {T} frame Spatial Jacobian
		void SpatialJacobian_Tool_1(); // sets Jsp_t_1 @ memory position: ptr2Jsp1
		void SpatialJacobian_Tool_2(Eigen::Matrix<float, 6, 1> *Jsp_t_2[robot_params::DOF]); // Returns {T} frame Spatial Jacobian
		void SpatialJacobian_Tool_2(); // sets Jsp_t_2 @ memory position: ptr2Jsp2
		void BodyJacobians(Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[robot_params::DOF+1]); // Returns Body Jacobians calculated @ active joint frames and the {T} frame
		void BodyJacobians(); // Sets BodyJacobiansFrames @ memory position: ptr2BodyJacobiansFrames
		void BodyCOMJacobians();
		void BodyJacobian_Tool_1(Eigen::Matrix<float, 6, 1> *Jbd_t_1[robot_params::DOF]); // Returns {T} frame Body Jacobian
		void BodyJacobian_Tool_1();
		void BodyJacobian_Tool_2(Eigen::Matrix<float, 6, 1> *Jbd_t_2[robot_params::DOF]); // Returns {T} frame Body Jacobian
		void BodyJacobian_Tool_2();
		void ToolVelocityTwist(typ_jacobian jacob_selection, float *dq, Eigen::Matrix<float, 6, 1> &Vtwist ); // Calculates {T} frame Velocity twists
		void ToolVelocityTwist(typ_jacobian jacob_selection); 
		void DtSpatialJacobian_Tool_1( float *dq, Eigen::Matrix<float, 6, 1> *Jsp_t_1[robot_params::DOF], Eigen::Matrix<float, 6, 1> *dJsp_t_1[robot_params::DOF] ); // Time derivative of spatial jacobian
		void DtSpatialJacobian_Tool_1();
		void DtBodyJacobian_Tool_1( float *dq, Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[robot_params::DOF+1], Eigen::Matrix<float, 6, 1> *dJbd_t_1[robot_params::DOF]) ;		
		void DtBodyJacobian_Tool_1();
		void DtBodyJacobian_Tool_2( float *dq, Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[robot_params::DOF+1], Eigen::Matrix<float, 6, 1> *dJbd_t_2[robot_params::DOF]) ;
		void DtBodyJacobian_Tool_2();
		void OperationalSpaceJacobian(Eigen::Matrix3f &Jop_t);
		Eigen::Matrix3f OperationalSpaceJacobian(const Eigen::Vector3f& qs);
		void OperationalSpaceJacobian();
		Eigen::Matrix3f* getOperationalJacobian();
		std::unique_ptr<Eigen::Matrix3f> OperationalSpaceJacobian_ptr(); 
		void inverseOperationalSpaceJacobian();
		Eigen::Matrix3f* getInverseOperationalJacobian();
		std::unique_ptr<Eigen::Matrix3f> inverseOperationalSpaceJacobian_ptr();
		Eigen::Matrix3f OperationalSpaceJacobian2();	
		void DtOperationalSpaceJacobian(Eigen::Matrix3f &dJop_t);
		void DtOperationalSpaceJacobian(); 
		Eigen::Matrix3f* getDerivativeOperationalJacobian();
		std::unique_ptr<Eigen::Matrix3f> DtOperationalSpaceJacobian_ptr();
		void DtToolVelocityTwist(typ_jacobian jacob_selection, float *ddq, float *dq, Eigen::Matrix<float, 6, 1> &dVtwist );
		void DtToolVelocityTwist(typ_jacobian jacob_selection);
		void CartesianVelocity_twist(Eigen::Vector4f &v_qs); // Returns spatial velocity {T} using the Spatial Velocity twist
		void CartesianVelocity_jacob(Eigen::Vector3f &v_qs); // Returns spatial velocity {T} using the Operational Space Jacobian
		void CartesianVelocity_jacob(Eigen::Vector4f &v_qs);
		void CartesianVelocity_jacob(Eigen::Vector3f &v_qs, Eigen::Matrix3f Jop_loc); 
		void CartesianAcceleration_twist(Eigen::Vector4f &a_qs, Eigen::Vector4f v_qs );
		void CartesianAcceleration_jacob(Eigen::Vector3f &a_qs);
		void CartesianAcceleration_jacob(Eigen::Vector4f &a_qs);		
		
		// Kinematic Performance Measures
		float KinematicManipulabilityIndex(const Eigen::Matrix3f& J);
		float KinematicManipulabilityIndex();
		std::pair<Eigen::Matrix3f, Eigen::Vector3f> KinematicManipulabilityEllipsoid(const Eigen::Matrix3f& J);
		std::pair<Eigen::Matrix3f, Eigen::Vector3f> KinematicManipulabilityEllipsoid();
		float JacobianConditionNumber(const Eigen::Matrix3f& J);
		std::unique_ptr<Eigen::Matrix3f> DLSInverseJacobian(const Eigen::Matrix3f& J, float base_lambda_dls = 0.005f);

		// Public Kinematic data members
		Eigen::Matrix<float, 6, 1> iXi[robot_params::DOF+1];
		Eigen::Isometry3f g[robot_params::DOF+1]; // stores joint frames+tcp
		Eigen::Isometry3f* g_ptr[robot_params::DOF+1]; // stores joint frames+tcp
		Eigen::Isometry3f gl[robot_params::DOF];  // stores links' COM frames
		Eigen::Isometry3f* ptr2_gl[robot_params::DOF];  // stores links' COM frames
		Eigen::Isometry3f Bi[robot_params::DOF+1]; 
		Eigen::Matrix<float, 6, robot_params::DOF> Jsp63; // the concatenated form of the Spatial Jacobian
		Eigen::Matrix<float, 6, robot_params::DOF> Jbd63;
		Eigen::Matrix<float, 6, robot_params::DOF> dJbd63;
		Eigen::Matrix<float, 6, 1>* ptr2Jsp1[robot_params::DOF]; // Declares an array of 3 pointers. Each pointer in this array can point to a different Eigen::Matrix<float, 6, 1> object
		Eigen::Matrix<float, 6, 1>* ptr2Jsp2[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> Jsp_t_1[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> Jsp_t_2[robot_params::DOF];
		Eigen::Matrix<float, 6, 1>** ptr2BodyJacobiansFrames[robot_params::DOF+1];
		Eigen::Matrix<float, 6, 1> BodyJacobiansFrames[robot_params::DOF+1][robot_params::DOF];
		Eigen::Matrix<float, 6, 1>* ptr2Jbd_t_1[robot_params::DOF];
		Eigen::Matrix<float, 6, 1>* ptr2Jbd_t_2[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> Jbd_t_1[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> Jbd_t_2[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> *ptr2dJsp_t_1[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> dJsp_t_1[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> *ptr2dJbd_t_1[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> *ptr2dJbd_t_2[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> dJbd_t_1[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> dJbd_t_2[robot_params::DOF];

		Eigen::Matrix<float, 6, robot_params::DOF> Jbsli63[robot_params::DOF];
		Eigen::Matrix<float, 6, robot_params::DOF>* ptr_Jbsli63[robot_params::DOF];
		Eigen::Matrix<float, 6, 1> Jbsli[robot_params::DOF][robot_params::DOF];
		Eigen::Matrix<float, 6, 1>* ptr_Jbsli[robot_params::DOF][robot_params::DOF];		

		Eigen::Matrix<float, 6, 1> Vsp_tool_twist;
		Eigen::Matrix<float, 6, 1> Vbd_tool_twist;
		Eigen::Matrix3f dJbd_pos;
		Eigen::Matrix3f Jbd_pos;
		Eigen::Matrix3f dJop; // Time Derivative of the Operational Space Jacobian
		Eigen::Matrix3f Jop; // Operational Space Jacobian
		Eigen::Matrix3f iJop;
		Eigen::Matrix3f* ptr2Jop;
		Eigen::Matrix3f* ptr2dJop;
		Eigen::Matrix3f* ptr2iJop;
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
		Eigen::Isometry3f _active_expos[robot_params::DOF];
		Eigen::Isometry3f _active_expos_anat[robot_params::DOF];
		Eigen::Isometry3f _Pi[robot_params::METALINKS];
        float _joint_pos[robot_params::DOF];
        float _joint_vel[robot_params::DOF];
		float _joint_accel[robot_params::DOF];

		// Set functions for elememts used in calculations
		void setExponentials(float *q);		
		void setExponentialsAnat(float *q);
		void setBodyPositionJacobian();
		void setDtBodyPositionJacobian();
		void setDtRotationMatrix();

		// Auxiliary functions for printing
		
		void print6nMatrix(Eigen::Matrix<float, 6, 1>* matrices[], const int n);
		void printTwist(Eigen::Matrix<float, 6, 1> Twist);		
		void print63MatrixByColumn(const Eigen::Matrix<float, 6, robot_params::DOF> J63[robot_params::DOF]);
};		

#endif // SCREWS_KINEMATICS_H

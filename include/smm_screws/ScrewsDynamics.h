#ifndef SCREWS_DYNAMICS_H
#define SCREWS_DYNAMICS_H

#include "smm_screws/ScrewsKinematics.h"

/*
 *  C++ Library(integrated in a ros_pkg) for computation of Dynamic Matrices for
 *  serial manipulators of fixed/reconfigurable structure, implementing Screw Theory
 *  Tools.
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
 
class ScrewsDynamics: public ScrewsKinematics {
	public:
        ScrewsDynamics();
        ScrewsDynamics(RobotAbstractBase *ptr2abstract);

        // Update the arrays that store the joint data
        void updateJointPos(float *q_new);
        void updateJointVel(float *dq_new);

        // Initialize Functions
        void intializeLinkMassMatrices();

        // Matrices exported for usage in ROS nodes
        Eigen::Matrix3f MM; // Marc Marquez (!), or Mass Matrix
        Eigen::Matrix3f CM; // Coriolis Matrix
        Eigen::Matrix<float, 3, 1> GV; // Gravity Vector
        Eigen::Matrix<float, 3, 1> FV; // Friction Vector
        Eigen::Isometry3f * ptr2active_tfs[DOF];
        Eigen::Isometry3f gai[DOF]; // Active joints frames
        Eigen::Isometry3f * ptr2passive_tfs[DOF];
        Eigen::Isometry3f gpj[METALINKS]; // Passive joints frames
        Eigen::Isometry3f * ptr2links_com_tfs[DOF];
        Eigen::Isometry3f gsli[DOF]; // Links COM frames

        // Basic functions for dynamic matrices
        Eigen::Matrix3f MassMatrix();
        Eigen::Matrix3f CoriolisMatrix();
        Eigen::Matrix<float, DOF, 1> GravityVector();
        Eigen::Matrix<float, DOF, 1> FrictionVector();
        void MassMatrix_loc();
        void CoriolisMatrix_loc();
        void GravityVector_loc();
        void FrictionVector_loc();
	private:
        static constexpr float _g_z = -9.80665;

        // Constructor essentials to inherit from Structure Abstract Class
		RobotAbstractBase *_ptr2abstract; // pointer that has memory address of the abstract class (defined structure parameters of the smm)
		uint8_t _total_pseudojoints;
		uint8_t _meta1_pseudojoints;
		uint8_t _meta2_pseudojoints;        
		Eigen::Isometry3f _last_expo;
		uint8_t _last_twist_cnt;
        bool _debug_verbosity;

        // Matrices used for internal compuatations
        float _delta_joint_pos[DOF];
        float _joint_pos_prev[DOF];
        float _joint_pos[DOF];
        float _joint_vel[DOF];
        Eigen::Matrix<float, 6, 6> _Mib[DOF]; // Link Mass matrices /{Links' Body Frames}
        Eigen::Matrix<float, 6, 6> _Ml_temp;
        Eigen::Matrix<float, 6, 6> _ad_temp;
        Eigen::Matrix<float, 6, 6> _alpha_temp;
        Eigen::Matrix<float, 6, 6> _alpha[2];
        Eigen::Matrix<float, 6, 6> _alphaParDer[5];
        Eigen::Matrix<float, 1, 1> _parDer_MassIJ_ThetaK;
        Eigen::Matrix<float, 6, 6> _alpha_transpose;
        Eigen::Matrix3f parDerMass[3];
        Eigen::Matrix3f ChristoffelSymbols[3];
        Eigen::Matrix<float, 6, 1> _LieBracketParDer[2];
        Eigen::Matrix<float, 1, 6> _xi_traspose;
        float _PotEnergy_prev;;
        float _PotEnergy;
        

        // Internal functions to set auxiliary tfs and matrix elements
        Eigen::Matrix<float, 6, 6> setAlphamatrix(size_t i, size_t j);
        Eigen::Matrix<float, 1, 1> computeParDerMassElement(size_t i, size_t j, size_t k);
        void updateCOMTfs();
        float computePotentialEnergy();
        void extractActiveTfs();

        // Simple auxiliary functions (for debugging)
		void print66Matrix(Eigen::Matrix<float, 6, 6> matrix);
        void print61Matrix(Eigen::Matrix<float, 6, 1> matrix);
        void print16Matrix(Eigen::Matrix<float, 1, 6> matrix);
};    
#endif // SCREWS_DYNAMICS_H
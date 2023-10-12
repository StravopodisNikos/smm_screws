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

        // Initialize Functions
        void intializeLinkMassMatrices();
        // Matrices exported for usage in ROS nodes
        Eigen::Matrix3f MM; // Marc Marquez (!), or Mass Matrix
        Eigen::Matrix3f CM; // Coriolis Matrix
        Eigen::Isometry3f gai[DOF];
        Eigen::Isometry3f gpj[METALINKS];

        // Basic functions for dynamic matrices
        Eigen::Matrix3f MassMatrix();

	private:
        // Constructor essentials to inherit from Structure Abstract Class
		RobotAbstractBase *_ptr2abstract; // pointer that has memory address of the abstract class (defined structure parameters of the smm)
		uint8_t _total_pseudojoints;
		uint8_t _meta1_pseudojoints;
		uint8_t _meta2_pseudojoints;        
		Eigen::Isometry3f _last_expo;
		uint8_t _last_twist_cnt;
        bool _debug_verbosity;
        

        // Matrices used for internal compuatations
        Eigen::Matrix<float, 6, 6> _Mib[DOF]; // Link Mass matrices /{Links' Body Frames}
        Eigen::Matrix<float, 6, 6> _Ml_temp;
        Eigen::Matrix<float, 6, 6> _ad_temp;
        Eigen::Matrix<float, 6, 6> _alpha_temp;
        Eigen::Matrix<float, 6, 6> _alpha[2];
        Eigen::Matrix<float, 6, 6> _alpha_transpose;
        Eigen::Matrix3f parDer_M[DOF];
        Eigen::Matrix<float, 6, 6> mult_temp;
        Eigen::Matrix<float, 6, 1> mult_temp1;
        Eigen::Matrix<float, 1, 6> _xi_traspose;
        // Internal functions to set auxiliary tfs 
        Eigen::Matrix<float, 6, 6> setAlphamatrix(size_t i, size_t j);

        // Simple auxiliary functions
		void print66Matrix(Eigen::Matrix<float, 6, 6> matrix);
        void print61Matrix(Eigen::Matrix<float, 6, 1> matrix);
        void print16Matrix(Eigen::Matrix<float, 1, 6> matrix);
};    
#endif // SCREWS_DYNAMICS_H
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
	private:

};    
#endif // SCREWS_DYNAMICS_H
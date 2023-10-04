#ifndef SCREWS_MAIN_H
#define SCREWS_MAIN_H

#include <Eigen/Dense>
#include <Eigen/Core>

/*
 *  C++ Library(integrated in a ros_pkg) for implementation of Screw Theory tools.
 *  Special Thanks and Citations:
 *  [1] Murray, R. M., Li, Z., Sastry, S. S., & Sastry, S. S. (1994). A mathematical introduction to robotic manipulation. CRC press.
 *  [2] Müller, A. (2018). Screw and Lie group theory in multibody kinematics: Motion representation and recursive kinematics of tree-topology systems. Multibody System Dynamics, 43(1), 37-70.
 *  [3] Müller, A. (2018). Screw and Lie group theory in multibody dynamics: recursive algorithms and equations of motion of tree-topology systems. Multibody System Dynamics, 42(2), 219-248.
 *   * 
 *  ros_pkg smm_screws, test file: example.cpp
 * 
 *  Author: Nikos Stravopodis, PhD Candidate
 */

class ScrewsMain {
    //
public:
    // Constructor
    ScrewsMain();
    
    void formTwist(Eigen::Matrix<float, 6, 1> & xi_R6, Eigen::Vector3f v, Eigen::Vector3f w);
    void formTwist(Eigen::Matrix4f & xi_se3, Eigen::Vector3f v, Eigen::Matrix3f wHat);
    void splitTwist(const Eigen::Matrix<float, 6, 1> xi_R6, Eigen::Vector3f & v, Eigen::Vector3f & w);
    void splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Vector3f & w);
    void splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Matrix3f & wHat);
    void vee(Eigen::Matrix<float, 6, 1>& xi , Eigen::Matrix4f xi_se3);
    Eigen::Matrix3f skew(const Eigen::Vector3f& w);
    Eigen::Vector3f unskew(const Eigen::Matrix3f& wHat);
    Eigen::Vector3f crossProduct(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
    void ad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );
    void Ad(Eigen::Matrix<float, 6, 6> & Ad, const Eigen::Isometry3f& Ci );
    void iad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );    
    Eigen::Matrix<float, 6, 1> lb(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6);    
    void spatialCrossProduct(Eigen::Matrix<float, 6, 6> & A, const Eigen::Matrix<float, 6, 1> xi_R6);
    Eigen::Matrix<float, 6, 1> screwProduct(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6);
    Eigen::Matrix3f skewExp(const Eigen::Vector3f& w, float theta);
    Eigen::Isometry3f twistExp(const Eigen::Matrix<float, 6, 1>& xi, float theta);
    Eigen::Isometry3f twistExp(const Eigen::Matrix4f& xi, float theta);

private:
    float _st;
    float _ct;
};

#endif // SCREWS_MAIN_H


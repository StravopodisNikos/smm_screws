#ifndef SCREWS_MAIN_H
#define SCREWS_MAIN_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath> // Include the cmath header for mathematical functions
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    void formTwist(Eigen::Matrix4f & xi_se3, Eigen::Matrix<float, 6, 1> xi_R6);
    void splitTwist(const Eigen::Matrix<float, 6, 1> xi_R6, Eigen::Vector3f & v, Eigen::Vector3f & w);
    void splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Vector3f & w);
    void splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Matrix3f & wHat);
    void vee(Eigen::Matrix<float, 6, 1>& xi , Eigen::Matrix4f xi_se3);
    Eigen::Matrix3f skew(const Eigen::Vector3f& w);
    Eigen::Vector3f unskew(const Eigen::Matrix3f& wHat);
    Eigen::Vector3f crossProduct(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
    void ad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );
    Eigen::Matrix<float, 6, 6> ad(const Eigen::Isometry3f& g );    
    void Ad(Eigen::Matrix<float, 6, 6> & Ad, const Eigen::Isometry3f& Ci );
    Eigen::Matrix<float, 6, 6>  Ad(const Eigen::Isometry3f& Ci );
    void iad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g );    
    Eigen::Matrix<float, 6, 6> iad(const Eigen::Isometry3f& g );
    Eigen::Matrix<float, 6, 1> lb(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6);    
    void spatialCrossProduct(Eigen::Matrix<float, 6, 6> & A, const Eigen::Matrix<float, 6, 1> xi_R6);
    Eigen::Matrix<float, 6, 1> screwProduct(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6);
    Eigen::Matrix3f skewExp(const Eigen::Vector3f& w, float theta);
    Eigen::Isometry3f twistExp(const Eigen::Matrix<float, 6, 1>& xi, float theta);
    Eigen::Isometry3f twistExp(const Eigen::Matrix4f& xi, float theta);
    Eigen::Isometry3f extractRelativeTf(Eigen::Isometry3f Ai, Eigen::Isometry3f Ai_1); // Computes: Bi [Mueller], g_i_i-1(0) [Murray]
    Eigen::Matrix<float, 6, 1> extractLocalScrewCoordVector(Eigen::Isometry3f Ai, Eigen::Matrix<float, 6, 1> Yi); // Computes the inverse of 1st relation in eq.(95),p.241,[3]
    Eigen::Matrix<float, 6, 1> extractLocalScrewPrevCoordVector(Eigen::Isometry3f Bi, Eigen::Matrix<float, 6, 1> iXi); // Computes eq.(7)/p.44,[2] 
    Eigen::Matrix<float, 6, 3> mergeColumns2Matrix63(const Eigen::Matrix<float, 6, 1> * column_array);    
    void extract_twist_points(Eigen::Matrix<float, 6, 1> & xi_R6, Eigen::Vector3f& start, Eigen::Vector3f& end);
    Eigen::Vector4f extractRotationQuaternion(const Eigen::Isometry3f g);

private:
    float _st;
    float _ct;
    Eigen::Isometry3f _Tf;
    Eigen::Matrix<float, 6, 1> _Yi;
    Eigen::Matrix<float, 6, 1> _iXi;
    Eigen::Matrix<float, 6, 1> _i_1Xi;
    Eigen::Matrix<float, 6, 1> _iXi_1;
    Eigen::Matrix<float, 6, 6> _ad;
    Eigen::Matrix<float, 6, 6> _iad;

    // Auxiliary functions 
    bool isequalf(double a, double b);
    bool isrot(const Eigen::Matrix3f& R);
    Eigen::Vector3f null(const Eigen::Matrix3f& A);
    Eigen::Vector3f rotaxis(const Eigen::Matrix3f& R, const float theta);
    std::pair<Eigen::Vector3f, float> rotparam(const Eigen::Matrix3f& R);
    Eigen::Vector4f quaternion_from_screws(const std::pair<Eigen::Vector3f, float>& omega_theta);
    
};

#endif // SCREWS_MAIN_H


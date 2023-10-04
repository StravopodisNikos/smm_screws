#include "smm_screws/ScrewsKinematics.h"

ScrewsKinematics::ScrewsKinematics() {};

Eigen::Isometry3f ScrewsKinematics::extractRelativeTf(Eigen::Isometry3f Ai, Eigen::Isometry3f Ai_1) {
    _Tf = Ai_1.inverse() * Ai;
    return _Tf;
}

Eigen::Matrix<float, 6, 1> ScrewsKinematics::extractLocalScrewCoordVector(Eigen::Isometry3f Ai, Eigen::Matrix<float, 6, 1> Yi) {
    // Computes the constant screw coordinate vector of joint <i> represented in the joint frame
    // fixed at body <i> @ reference configuration (q=0), iXi /in R(6)
    ad(_ad, Ai.inverse());
    _iXi = _ad * Yi;
    return _iXi;
}

Eigen::Matrix<float, 6, 1> ScrewsKinematics::extractLocalScrewPrevCoordVector(Eigen::Isometry3f Bi, Eigen::Matrix<float, 6, 1> iXi) {
    // Computes the constant screw coordinate vector of joint <i> represented in the joint frame
    // fixed at the previous body <i-1> @ reference configuration (q=0), i_1Xi /in R(6) 
    ad(_ad, Bi);
    _i_1Xi = _ad * iXi;  
    return _i_1Xi;
} 





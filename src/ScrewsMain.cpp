#include "smm_screws/ScrewsMain.h"

// Constructor implementation
ScrewsMain::ScrewsMain() {
    // Initialize member variables or perform any setup here
}

void ScrewsMain::formTwist(Eigen::Matrix<float, 6, 1> & xi_R6, Eigen::Vector3f v, Eigen::Vector3f w) {
    xi_R6 << v , w;
}

void ScrewsMain::formTwist(Eigen::Matrix4f & xi_se3, Eigen::Vector3f v, Eigen::Matrix3f wHat) {
    xi_se3.block<3, 3>(0, 0) =  wHat;
    xi_se3.block<3, 1>(0, 3) =  v;
    xi_se3.block<1, 4>(3, 0) =  Eigen::Vector4f::Zero();
}

void ScrewsMain::formTwist(Eigen::Matrix4f & xi_se3, Eigen::Matrix<float, 6, 1> xi_R6) {
    // Converts a R(6) twist to the se(3) representation
    Eigen::Vector3f v = xi_R6.block(0, 0, 3, 1);  // Extract the first 3 elements as a vector
    Eigen::Vector3f w = xi_R6.block(3, 0, 3, 1);
    xi_se3.block<3, 3>(0, 0) =  skew(w);
    xi_se3.block<3, 1>(0, 3) =  v;
    xi_se3.block<1, 4>(3, 0) =  Eigen::Vector4f::Zero();
}

void ScrewsMain::splitTwist(const Eigen::Matrix<float, 6, 1> xi_R6, Eigen::Vector3f & v, Eigen::Vector3f & w) {
    v = xi_R6.block<3, 1>(0, 0);
    w = xi_R6.block<3, 1>(3, 0);
}

void ScrewsMain::splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Vector3f & w) {
    Eigen::Matrix<float, 6, 1> xi_R6;
    vee(xi_R6, xi_se3);
    v = xi_R6.block<3, 1>(0, 0);
    w = xi_R6.block<3, 1>(3, 0);    
}

void ScrewsMain::splitTwist(const Eigen::Matrix4f xi_se3, Eigen::Vector3f & v, Eigen::Matrix3f & wHat) {
    wHat = xi_se3.block<3, 3>(0, 0);
    v    = xi_se3.block<3, 1>(0, 3); 
}

void ScrewsMain::vee(Eigen::Matrix<float, 6, 1>& xi , Eigen::Matrix4f xi_se3) {
    // forms a 6x1 vector twist from a se(3) twist marix
    Eigen::Matrix3f wHat = xi_se3.block<3, 3>(0, 0);
    formTwist( xi, xi_se3.block<3, 1>(0, 3), unskew(wHat));
}

Eigen::Matrix3f ScrewsMain::skew(const Eigen::Vector3f& w) {
    // w \in R3 -> wHat \in so(3)
    Eigen::Matrix3f m;
    m << 0    , -w.z(),  w.y(),
         w.z(),      0, -w.x(),
        -w.y(),  w.x(),      0;
    return m;    
}

Eigen::Vector3f ScrewsMain::unskew(const Eigen::Matrix3f& wHat) {
    // wHat \in so(3) -> w \in R3
    Eigen::Vector3f w;
    w.x() = -wHat(1,2);
    w.y() =  wHat(0,2);
    w.z() =  wHat(1,0);
    return w;
}

Eigen::Vector3f ScrewsMain::crossProduct(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    Eigen::Vector3f result;
    result << v1.y() * v2.z() - v1.z() * v2.y(),
            v1.z() * v2.x() - v1.x() * v2.z(),
            v1.x() * v2.y() - v1.y() * v2.x();
    return result;
}

void ScrewsMain::ad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g ) {
    // Described in eq.(2.58)/p.55,[1]
    // g \in SE(3) homogeneous rigid body tf
    Eigen::Matrix3f R = g.linear();
    Eigen::Vector3f p = g.translation();

    Eigen::Matrix3f pHat = skew(p);

    A.block<3, 3>(0, 0) = R;
    A.block<3, 3>(0, 3) = pHat * R;
    A.block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();
    A.block<3, 3>(3, 3) = R;
}

Eigen::Matrix<float, 6, 6> ScrewsMain::ad(const Eigen::Isometry3f& g ) {
    // Described in eq.(2.58)/p.55,[1]
    // g \in SE(3) homogeneous rigid body tf
    Eigen::Matrix3f R = g.linear();
    Eigen::Vector3f p = g.translation();

    Eigen::Matrix3f pHat = skew(p);

    _ad.block<3, 3>(0, 0) = R;
    _ad.block<3, 3>(0, 3) = pHat * R;
    _ad.block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();
    _ad.block<3, 3>(3, 3) = R;

    return _ad;
}

void ScrewsMain::Ad(Eigen::Matrix<float, 6, 6> & Ad, const Eigen::Isometry3f& Ci ) {
    // Described in eq.(96)/p.241,[3] 
    // Ci \in SE(3) homogeneous rigid body tf wrt the base frame {S}(or {IFR} in Mueller's terms)
    Eigen::Matrix3f Ri = Ci.linear();
    Eigen::Vector3f ri = Ci.translation();

    Eigen::Matrix3f riHat = skew(ri);

    Ad.block<3, 3>(0, 0) = Ri;
    Ad.block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
    Ad.block<3, 3>(3, 0) = riHat * Ri;
    Ad.block<3, 3>(3, 3) = Ri;
}

Eigen::Matrix<float, 6, 6>  ScrewsMain::Ad(const Eigen::Isometry3f& Ci ) {
    // Described in eq.(96)/p.241,[3] 
    // Ci \in SE(3) homogeneous rigid body tf wrt the base frame {S}(or {IFR} in Mueller's terms)
    Eigen::Matrix3f Ri = Ci.linear();
    Eigen::Vector3f ri = Ci.translation();

    Eigen::Matrix3f riHat = skew(ri);

    _ad.block<3, 3>(0, 0) = Ri;
    _ad.block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
    _ad.block<3, 3>(3, 0) = riHat * Ri;
    _ad.block<3, 3>(3, 3) = Ri;
    
    return _ad;
}

void ScrewsMain::iad(Eigen::Matrix<float, 6, 6> & A, const Eigen::Isometry3f& g ) {
// g \in SE(3) homogeneous rigid body tf
    Eigen::Matrix3f R = g.linear();
    Eigen::Vector3f p = g.translation();

    Eigen::Matrix3f pHat = skew(p);

    A.block<3, 3>(0, 0) =  R.transpose();
    A.block<3, 3>(0, 3) = -R.transpose() * pHat;
    A.block<3, 3>(3, 0) =  Eigen::Matrix3f::Zero();
    A.block<3, 3>(3, 3) =  R.transpose();
}

Eigen::Matrix<float, 6, 6> ScrewsMain::iad(const Eigen::Isometry3f& g ) {
// g \in SE(3) homogeneous rigid body tf
    Eigen::Matrix3f R = g.linear();
    Eigen::Vector3f p = g.translation();

    Eigen::Matrix3f pHat = skew(p);

    _iad.block<3, 3>(0, 0) =  R.transpose();
    _iad.block<3, 3>(0, 3) = -R.transpose() * pHat;
    _iad.block<3, 3>(3, 0) =  Eigen::Matrix3f::Zero();
    _iad.block<3, 3>(3, 3) =  R.transpose();
    return _iad;
}

Eigen::Matrix<float, 6, 1>  ScrewsMain::lb(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6) {
    // Lie bracket operator on R6, as in [1]/p.175
    // Screw Product of twists \in R6 (Mueller),
    // Generalization of the cross product on R3 to vectors on R6

    Eigen::Vector3f v_i;
    Eigen::Vector3f w_i;
    Eigen::Vector3f v_j;
    Eigen::Vector3f w_j;
    Eigen::Matrix3f w_i_hat;
    Eigen::Matrix3f w_j_hat;
    Eigen::Matrix4f xi_i_se3;
    Eigen::Matrix4f xi_j_se3;
    Eigen::Matrix4f cross_se3;
    Eigen::Matrix<float, 6, 1> LB;

    splitTwist(xi_i_R6, v_i, w_i);
    splitTwist(xi_j_R6, v_j, w_j);

    w_i_hat = skew(w_i);
    w_j_hat = skew(w_j);

    formTwist(xi_i_se3, v_i, w_i_hat);
    formTwist(xi_j_se3, v_j, w_j_hat);

    cross_se3 = xi_i_se3 * xi_j_se3 - xi_j_se3 * xi_i_se3;
    vee(LB, cross_se3);
    return LB;
}

void ScrewsMain::spatialCrossProduct(Eigen::Matrix<float, 6, 6> & A, const Eigen::Matrix<float, 6, 1> xi_R6) {
    // Described in [3]/p.243/eq.(116), "spatial cross product". This is Muellers ad(X) tf.
    // This should be used for derivatives of the Jacobians, based on Mueller papers.
    Eigen::Vector3f v;
    Eigen::Vector3f w;
    Eigen::Matrix3f v_hat;
    Eigen::Matrix3f w_hat;

    splitTwist(xi_R6, v, w);
    v_hat = skew(v);
    w_hat = skew(w);

    // [11-10-23] Follows the Murray twist notation /[1]: xi = [v w]', not Muellers' in /[2,3]!
    A.block<3, 3>(0, 0) = w_hat;
    A.block<3, 3>(0, 3) = v_hat;
    A.block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
    A.block<3, 3>(3, 3) = w_hat;
}

Eigen::Matrix<float, 6, 1>  ScrewsMain::screwProduct(Eigen::Matrix<float, 6, 1> xi_i_R6, Eigen::Matrix<float, 6, 1> xi_j_R6) {
    // Lie bracket operator, as in [3]/p.243/eq.(115), "screw product"
    Eigen::Matrix<float, 6, 1> LB;
    Eigen::Matrix<float, 6, 6> ad_twist; // sqp
    spatialCrossProduct(ad_twist, xi_i_R6); // Mueller defined adjoint, or "spatial cross product"
    LB = ad_twist * xi_j_R6;
    return LB;
}

Eigen::Matrix3f ScrewsMain::skewExp(const Eigen::Vector3f& w, float theta) {
    // Since 3x1 vector is provided, must be transformed to 3x3 skew-symmetric matrix
    Eigen::Matrix3f R = skew(w);

    // theta must be 1x1 float value
    _st = std::sin(theta);
    _ct = std::cos(theta);
    Eigen::Matrix3f exp_w_theta = Eigen::Matrix3f::Identity() + R * _st + (R * R) * (1 - _ct);
    
    return exp_w_theta;
}

// Overloaded twistExp function for 4x4 twist R(6)
Eigen::Isometry3f ScrewsMain::twistExp(const Eigen::Matrix<float, 6, 1>& xi, float theta) {
    Eigen::Isometry3f g = Eigen::Isometry3f::Identity();
    Eigen::Vector3f v = xi.block<3, 1>(0, 0);
    Eigen::Vector3f w = xi.block<3, 1>(3, 0);

    if (w.isZero()) {
        // Pure translation case
        g.translation() = v * theta;
    } else {
        // Rotation and translation case
        Eigen::Matrix3f e = skewExp(w, theta);
        g.linear() = e;
        g.translation() = (Eigen::Matrix3f::Identity() - e) * (skew(w) * v) + w * w.transpose() * v * theta;
    }

    return g;
}

// Overloaded twistExp function for 4x4 twist se(3)
Eigen::Isometry3f ScrewsMain::twistExp(const Eigen::Matrix4f& xi, float theta) {
    Eigen::Matrix<float, 6, 1> xi_R6;
    vee(xi_R6, xi);
    Eigen::Vector3f v = xi_R6.block<3, 1>(0, 0);
    Eigen::Vector3f w = xi_R6.block<3, 1>(3, 0);

    Eigen::Isometry3f g = Eigen::Isometry3f::Identity();

    if (w.isZero()) {
        // Pure translation case
        g.translation() = v * theta;
    } else {
        // Rotation and translation case
        Eigen::Matrix3f e = skewExp(w, theta);
        g.linear() = e;
        g.translation() = (Eigen::Matrix3f::Identity() - e) * (skew(w) * v) + w * w.transpose() * v * theta;
    }

    return g;
}

Eigen::Isometry3f ScrewsMain::extractRelativeTf(Eigen::Isometry3f Ai, Eigen::Isometry3f Ai_1) {
    _Tf = Ai_1.inverse() * Ai;
    return _Tf;
}

Eigen::Matrix<float, 6, 1> ScrewsMain::extractLocalScrewCoordVector(Eigen::Isometry3f Ai, Eigen::Matrix<float, 6, 1> Yi) {
    // Computes the constant screw coordinate vector of joint <i> represented in the joint frame
    // fixed at body <i> @ reference configuration (q=0), iXi /in R(6)
    // [14-7-24] This is the MATLAB var: s_mueller.XI_ii calculated in relateMurray2Mueller.m
    ad(_ad, Ai.inverse());
    _iXi = _ad * Yi;
    return _iXi;
}

Eigen::Matrix<float, 6, 1> ScrewsMain::extractLocalScrewPrevCoordVector(Eigen::Isometry3f Bi, Eigen::Matrix<float, 6, 1> iXi) {
    // Computes the constant screw coordinate vector of joint <i> represented in the joint frame
    // fixed at the previous body <i-1> @ reference configuration (q=0), i_1Xi /in R(6) 
    // [14-7-24] This is the MATLAB var: s_mueller.XI_i_i1 calculated in relateMurray2Mueller.m
    ad(_ad, Bi);
    _i_1Xi = _ad * iXi;  
    return _i_1Xi;
} 

//Eigen::Matrix<float, 6, Eigen::Dynamic> ScrewsMain::mergeColumns2Matrix(const Eigen::Matrix<float, 6, 1> * column_array)  {
Eigen::Matrix<float, 6, 3> ScrewsMain::mergeColumns2Matrix63(const Eigen::Matrix<float, 6, 1> * column_array)  {    
    int n = column_array->size(); // Number of columns 
    if (n != 3)
    {
        // ERROR MSG
    }

    //Eigen::Matrix<float, 6, Eigen::Dynamic> matrix(6, n);
    Eigen::Matrix<float, 6, 3> matrix;

    for (int i = 0; i < 3; i++) {
        matrix.col(i) = column_array[i];
    }

    return matrix;
}

Eigen::Vector4f ScrewsMain::extractRotationQuaternion(const Eigen::Isometry3f g) {
    // [19-7-24] Extracts quaternions for providing active tfs rotation
    //           when needed for ros built-in tools.
    //           Before call, always update active tfs with ForwardKinematics3DOF_2()

    // Extract rotation part from Eigen::Isometry3f g
    Eigen::Matrix3f R = g.rotation();
    
    // Get the rotation parameters (omega and theta)
    std::pair<Eigen::Vector3f, float> omega_theta = rotparam(R);
    
    // Convert to quaternion
    Eigen::Vector4f quaternion = quaternion_from_screws(omega_theta);

    return quaternion;
}

void ScrewsMain::extract_twist_points(Eigen::Matrix<float, 6, 1> & xi_R6, Eigen::Vector3f& start, Eigen::Vector3f& end) {
    // [19-7-24] Extracts 2 points for line defined by twist

    // Split the twist into linear (v) and angular (w) parts
    Eigen::Vector3f v;
    Eigen::Vector3f w;
    splitTwist(xi_R6,v,w);

    // Compute the norm of the angular part (omega)
    float w_norm = w.norm();
    
    // Avoid division by zero
    if (w_norm > 1e-6) {
        // Compute the point (p) using the formula (w cross v) / norm(w)^2
        Eigen::Vector3f p = w.cross(v) / (w_norm * w_norm);

        // Lambda value to compute the end point
        float lambda = 0.25f; // You can change this value to scale the direction vector

        // Compute the start point (p)
        start = p;

        // Compute the end point (p + lambda * w)
        end = p + lambda * w;
    } else {
        // If w is zero, the twist represents pure translation
        start = Eigen::Vector3f::Zero();
        end = v;
    }
}

// ==================================================================== //
// ================ AUXILIARY CLASS METHODS FOR SCREWS ================ //
// ==================================================================== //

bool ScrewsMain::isequalf(double a, double b) {
    double thresh = std::numeric_limits<double>::epsilon() * 100.0;
    return std::abs(a - b) <= thresh;
}

bool ScrewsMain::isrot(const Eigen::Matrix3f& R) {
    Eigen::Matrix3f Rt = R.transpose();
    Eigen::Matrix3f shouldBeIdentity = Rt * R;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    return shouldBeIdentity.isApprox(I);
}

Eigen::Vector3f ScrewsMain::null(const Eigen::Matrix3f& A) {
    // Find the null space of a matrix
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullV);
    return svd.matrixV().col(2);
}

Eigen::Vector3f ScrewsMain::rotaxis(const Eigen::Matrix3f& R, const float theta) {
    // Calculate the rotation axis
    if (!isrot(R)) {
        throw std::invalid_argument("[rotaxis] R must be a rotation matrix");
    }

    if (theta < 0.0f || theta > (M_PI+0.001) ) {
        throw std::invalid_argument("[rotaxis] Theta must be between 0 and pi");
    }

    Eigen::Vector3f axis;
    if (isequalf(theta, M_PI) || isequalf(theta, 0)) {
        if (R.isApprox(Eigen::Matrix3f::Identity())) {
            axis << 0, 1, 0; // y-axis is arbitrarily selected based on theory p.30
        } else {
            axis = null(R - Eigen::Matrix3f::Identity());
        }
        axis.normalize();
    } else {
        axis << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        axis /= (2.0f * std::sin(theta) );
    }

    return axis;
}

// Calculate the rotation parameters
std::pair<Eigen::Vector3f, float> ScrewsMain::rotparam(const Eigen::Matrix3f& R) {
    if (!isrot(R)) {
        throw std::invalid_argument("[rotparam] T must be a rotation");
    }

    float t = (R.trace() - 1.0f) / 2.0f;

    // Handle numerical errors
    if (t < -1.0f) {
        t = -1.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }

    float theta = std::acos(t);
    std::cout << "Rotation angle (theta): " << theta << std::endl;

    Eigen::Vector3f omega = rotaxis(R, theta);
    std::cout << "Rotation axis (omega): " << omega.transpose() << std::endl;
    return std::make_pair(omega, theta);
}

Eigen::Vector4f ScrewsMain::quaternion_from_screws(const std::pair<Eigen::Vector3f, float>& omega_theta) {
    // [19-7-24] Implements p.34 in [1]
    // [w,theta] must be extracted before
    const Eigen::Vector3f& omega = omega_theta.first;
    float theta = omega_theta.second;

    Eigen::Vector4f quaternion;

    float half_theta = theta / 2.0f;
    float sin_half_theta = std::sin(half_theta);

    quaternion(0) = std::cos(half_theta);    // w component
    quaternion(1) = omega(0) * sin_half_theta; // x component
    quaternion(2) = omega(1) * sin_half_theta; // y component
    quaternion(3) = omega(2) * sin_half_theta; // z component

    return quaternion;
}
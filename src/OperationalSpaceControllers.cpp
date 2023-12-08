#include "smm_screws/OperationalSpaceControllers.h"

using namespace OperationalSpaceControllers;

// ****************** INVERSE DYNAMICS CONTROLLER ******************

InverseDynamicsController::InverseDynamicsController() {};
InverseDynamicsController::InverseDynamicsController(ScrewsKinematics *ptr2kinematics, ScrewsDynamics *ptr2dynamics):  _ptr2_screws_kin_object(ptr2kinematics), _ptr2_screws_dyn_object(ptr2dynamics) {
    _I_dof = Eigen::Matrix<float, DOF, DOF>::Identity();
    _O_dof = Eigen::Matrix<float, DOF, DOF>::Zero();
    initialize_gain_matrices();
    set_state_transition_matrix(); // not used yet
}

void InverseDynamicsController::initialize_gain_matrices() {
    ros::NodeHandle nh;
    
    if (!nh.getParam("idosc/kp", _kp)) {
        ROS_ERROR("[InverseDynamicsController/initialize_gain_matrices] Failed to load: Kp");
        _Kp = Eigen::Matrix<float, DOF, DOF>::Zero();
    } else {
        _Kp = _kp * _I_dof;
    }
    if (!nh.getParam("idosc/kd", _kd)) {
        ROS_ERROR("[InverseDynamicsController/initialize_gain_matrices] Failed to load: Kd");
        _Kd = Eigen::Matrix<float, DOF, DOF>::Zero();
    } else {
        _Kd = _kd * _I_dof;
    }
    return;
}

void InverseDynamicsController::set_state_transition_matrix() {
    transition_matrix.block<3, 3>(0, 0) = -_Kd;
    transition_matrix.block<3, 3>(0, 3) = -_Kp;
    transition_matrix.block<3, 3>(3, 0) = _I_dof;
    transition_matrix.block<3, 3>(3, 3) = _O_dof;
    return;
}

void InverseDynamicsController::set_desired_state(Eigen::Matrix<float, IDOSC_STATE_DIM, 1> desired_state_received) {
    _D = desired_state_received;
    return;
}

void InverseDynamicsController::set_error_state(Eigen::Matrix<float, IDOSC_STATE_DIM, 1> current_state_received) {
    _X = _D - current_state_received;
    _x1 = _X.block<DOF, 1>(0, 0); // velocity error
    _x2 = _X.block<DOF, 1>(3, 0); // position error
    for (int i = 0; i < 6; i++) {
        ROS_INFO("error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void InverseDynamicsController::set_error_state(Eigen::Matrix<float, IDOSC_STATE_DIM, 1> current_state_received, Eigen::Matrix<float, IDOSC_STATE_DIM, 1> & error_state) {
    error_state = _D - current_state_received;
    _X = error_state;
    _x1 = _X.block<DOF, 1>(0, 0); // velocity error
    _x2 = _X.block<DOF, 1>(3, 0); // position error
    for (int i = 0; i < 6; i++) {
        ROS_INFO("error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void InverseDynamicsController::set_error_state(float *q_new) {
    Eigen::Vector3f p_qs;
    Eigen::Vector3f v_qs;
    Eigen::Matrix<float, IDOSC_STATE_DIM, 1> current_state;
    p_qs = _ptr2_screws_kin_object->updatePositionTCP(q_new);
    _ptr2_screws_kin_object->CartesianVelocity_jacob(v_qs);
    current_state(0) = v_qs.x();
    current_state(1) = v_qs.y();
    current_state(2) = v_qs.z(); 
    current_state(3) = p_qs.x();
    current_state(4) = p_qs.y();
    current_state(5) = p_qs.z(); 
    for (int i = 0; i < 6; i++) {
        ROS_INFO("current_state[ %d ]: %f", i, current_state(i));
    }    

    _X = _D - current_state;
    _x1 = _X.block<DOF, 1>(0, 0); // velocity error
    _x2 = _X.block<DOF, 1>(3, 0); // position error
    for (int i = 0; i < 6; i++) {
        ROS_INFO("error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void InverseDynamicsController::update_dq(float *dq_new) {
    for (size_t i = 0; i < DOF; i++) {_dq[i] = dq_new[i];}
    return;
}

void InverseDynamicsController::update_inverse_operational_jacob() {
    _iJop = _ptr2_screws_kin_object->Jop.inverse();
    //ROS_INFO("Inverse Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
    //_iJop(0, 0), _iJop(0, 1), _iJop(0, 2),
    //_iJop(1, 0), _iJop(1, 1), _iJop(1, 2),
    //_iJop(2, 0), _iJop(2, 1), _iJop(2, 2)); 
    return;
}

void InverseDynamicsController::update_derivative_operational_jacob() {
    _dtJop = _ptr2_screws_kin_object->dJop;
    return;
}

void InverseDynamicsController::update_control_input() {
    update_inverse_operational_jacob();
    update_derivative_operational_jacob();
    _y = _iJop * ( _Kd * _x1 + _Kp * _x2 - _dtJop * _dq);
    //ROS_INFO(" y[0]: %f", _y(0));
    //ROS_INFO(" y[1]: %f", _y(1));
    //ROS_INFO(" y[2]: %f", _y(2));
    return;
}

void InverseDynamicsController::update_torques() {
    _torque_cmd = _ptr2_screws_dyn_object->MassMatrix() * _y + (_ptr2_screws_dyn_object->CoriolisMatrix() * _dq + _ptr2_screws_dyn_object->GravityVector() + _ptr2_screws_dyn_object->FrictionVector() ) ;
    return;
}

void InverseDynamicsController::update_torques(Eigen::Vector3f &torque_out) {
    torque_out = _ptr2_screws_dyn_object->MassMatrix() * _y + (_ptr2_screws_dyn_object->CoriolisMatrix() * _dq + _ptr2_screws_dyn_object->GravityVector() + _ptr2_screws_dyn_object->FrictionVector() ) ;
    //torque_out = _ptr2_screws_dyn_object->MassMatrix() * _y + (_ptr2_screws_dyn_object->CoriolisMatrix() * _dq + _ptr2_screws_dyn_object->FrictionVector() ) ;
    return;
}

// ****************** IMPEDANCE CONTROLLER ******************

ImpedanceController::ImpedanceController() {};
ImpedanceController::ImpedanceController(ScrewsKinematics *ptr2kinematics, ScrewsDynamics *ptr2dynamics):  _ptr2_screws_kin_object(ptr2kinematics), _ptr2_screws_dyn_object(ptr2dynamics) {
    _I_dof = Eigen::Matrix<float, DOF, DOF>::Identity();
    _O_dof = Eigen::Matrix<float, DOF, DOF>::Zero();
    initialize_gain_matrices();
}

void ImpedanceController::initialize_gain_matrices() {
    ros::NodeHandle nh;
    
    if (!nh.getParam("impedance/md", _md)) {
        ROS_ERROR("[ImpedanceController/initialize_gain_matrices] Failed to load: Md");
        _Md = Eigen::Matrix<float, DOF, DOF>::Zero();
    } else {
        _Md = _md * _I_dof;
    }    
    if (!nh.getParam("impedance/kp", _kp)) {
        ROS_ERROR("[ImpedanceController/initialize_gain_matrices] Failed to load: Kp");
        _Kp = Eigen::Matrix<float, DOF, DOF>::Zero();
    } else {
        _Kp = _kp * _I_dof;
    }
    if (!nh.getParam("impedance/kd", _kd)) {
        ROS_ERROR("[ImpedanceController/initialize_gain_matrices] Failed to load: Kd");
        _Kd = Eigen::Matrix<float, DOF, DOF>::Zero();
    } else {
        _Kd = _kd * _I_dof;
    }
    return;
}

void ImpedanceController::set_desired_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> desired_state_received) {
    _D = desired_state_received;
    return;
}

void ImpedanceController::set_error_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> current_state_received) {
    _X = _D - current_state_received;
    _x1 = _X.block<DOF, 1>(0, 0); // velocity error
    _x2 = _X.block<DOF, 1>(3, 0); // position error
    for (int i = 0; i < 6; i++) {
        ROS_INFO("[ImpedanceController/set_error_state] Error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void ImpedanceController::set_error_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> current_state_received, Eigen::Matrix<float, IDOSC_STATE_DIM, 1> & error_state) {
    error_state = _D - current_state_received;
    _X = error_state;
    _x1 = _X.block<DOF, 1>(0, 0); // velocity error
    _x2 = _X.block<DOF, 1>(3, 0); // position error
    for (int i = 0; i < 6; i++) {
        ROS_INFO("[ImpedanceController/set_error_state] Error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void ImpedanceController::set_error_state(float *q_new) {
    Eigen::Vector3f p_qs;
    Eigen::Vector3f v_qs;
    Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> current_state;
    p_qs = _ptr2_screws_kin_object->updatePositionTCP(q_new);
    _ptr2_screws_kin_object->CartesianVelocity_jacob(v_qs);
    current_state(0) = v_qs.x();
    current_state(1) = v_qs.y();
    current_state(2) = v_qs.z(); 
    current_state(3) = p_qs.x();
    current_state(4) = p_qs.y();
    current_state(5) = p_qs.z(); 
    for (int i = 0; i < 6; i++) {
        ROS_INFO("[ImpedanceController/set_error_state] Current_state[ %d ]: %f", i, current_state(i));
    }    

    _X = _D - current_state;
    _x1 = _X.block<DOF, 1>(0, 0); // velocity error
    _x2 = _X.block<DOF, 1>(3, 0); // position error
    for (int i = 0; i < 6; i++) {
        ROS_INFO("[ImpedanceController/set_error_state] Error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void ImpedanceController::update_dq(float *dq_new) {
    for (size_t i = 0; i < DOF; i++) {_dq[i] = dq_new[i];}
    return;
}

void ImpedanceController::update_force_measurements(float *force_meas) {
    for (int i = 0; i < DOF; i++) {
        _he[i] = force_meas[i];
        _hA[i] = _he[i]; // only in the 3dof case study!
        ROS_INFO("[ImpedanceController/update_force_measurements] Force axis [ %d ]: %f", i, _he[i]);
    }
    return;
}

void ImpedanceController::update_inverse_operational_jacob() {
    _iJop = _ptr2_screws_kin_object->Jop.inverse();
    return;
}

void ImpedanceController::update_derivative_operational_jacob() {
    _dtJop = _ptr2_screws_kin_object->dJop;
    return;
}

void ImpedanceController::update_control_input() {
    // Based on eq.9.31/p.373 /in [1]
    update_inverse_operational_jacob();
    update_derivative_operational_jacob();
    _y = _iJop * _Md.inverse() * ( _Kd * _x1 + _Kp * _x2 - _Md * _dtJop * _dq - _hA);
    //ROS_INFO(" y[0]: %f", _y(0));
    //ROS_INFO(" y[1]: %f", _y(1));
    //ROS_INFO(" y[2]: %f", _y(2));
    return;
}

void ImpedanceController::update_torques() {
    _torque_cmd = _ptr2_screws_dyn_object->MassMatrix() * _y + (_ptr2_screws_dyn_object->CoriolisMatrix() * _dq + _ptr2_screws_dyn_object->GravityVector() + _ptr2_screws_dyn_object->FrictionVector() ) + _ptr2_screws_kin_object->Jop.transpose() * _he;
    return;
}

void ImpedanceController::update_torques(Eigen::Vector3f &torque_out) {
    torque_out = _ptr2_screws_dyn_object->MassMatrix() * _y + (_ptr2_screws_dyn_object->CoriolisMatrix() * _dq + _ptr2_screws_dyn_object->GravityVector() + _ptr2_screws_dyn_object->FrictionVector() ) + _ptr2_screws_kin_object->Jop.transpose() * _he;
    return;
}

// ****************** HYBRID CONTROLLER ******************

HybridController::HybridController() {};
HybridController::HybridController(ScrewsKinematics *ptr2kinematics, ScrewsDynamics *ptr2dynamics):  _ptr2_screws_kin_object(ptr2kinematics), _ptr2_screws_dyn_object(ptr2dynamics) {
    _I_dof = Eigen::Matrix<float, DOF, DOF>::Identity();
    _O_dof = Eigen::Matrix<float, DOF, DOF>::Zero();
    initialize_gain_matrices();
    initialize_subspace_matrices();
}

void HybridController::initialize_gain_matrices() {
    ros::NodeHandle nh;
    
    if (!nh.getParam("hybrid/kil", _ki_l)) {
        ROS_ERROR("[HybridController/initialize_gain_matrices] Failed to load: Kil");
        _Ki_l = Eigen::Matrix<float, DOF, DOF>::Zero();
    } else {
        _Ki_l = _ki_l * _I_dof;
    }    
    if (!nh.getParam("hybrid/kpv", _kp_v)) {
        ROS_ERROR("[HybridController/initialize_gain_matrices] Failed to load: Kpv");
        _Kp_v = Eigen::Matrix<float, DOF, DOF>::Zero();
    } else {
        _Kp_v = _kp_v * _I_dof;
    }
    if (!nh.getParam("hybrid/kdv", _kd_v)) {
        ROS_ERROR("[HybridController/initialize_gain_matrices] Failed to load: Kdv");
        _Kd_v = Eigen::Matrix<float, DOF, DOF>::Zero();
    } else {
        _Kd_v = _kd_v * _I_dof;
    }
    return;
}

void HybridController::initialize_constraint_frame(Eigen::Vector3f pi, Eigen::Vector3f pf, Eigen::Vector3f & zf_s) {
    zf_s = zf_s.normalized();
    Eigen::Vector3f yv_s;
    Eigen::Vector3f xv_s = (pf - pi).normalized();
    bool hybrid_feas = false;
    if ( ( xv_s.cross(zf_s)).norm() != 0 )
    {
        hybrid_feas = true;
        yv_s = zf_s.cross(xv_s);
    }
    // Build the rotation matrix
    _gsc = Eigen::Isometry3f::Identity();
    _gsc.linear().col(0) = xv_s;
    _gsc.linear().col(1) = yv_s;
    _gsc.linear().col(2) = zf_s;

    // Set the translation part (first three elements of the last column)
    _gsc.translation() = pi;
    
    return;   
}

void HybridController::initialize_subspace_matrices() {
    // Since {C} frame is strictly defined, the selection matrices can
    // be uniquely defined (given the limitations presented in header file)
    _Sv_c = Eigen::Matrix<float, DOF, VELOCITY_CONTROL_SUBSPACE_DIM>::Zero();
    _Sv_c(0,0) = 1.0f;
    _Sv_c(1,1) = 1.0f;
    _Sf_c = Eigen::Matrix<float, DOF, FORCE_CONTROL_SUBSPACE_DIM>::Zero();
    _Sf_c(2,0) = 1.0f;

    _W_v = Eigen::Matrix<float, DOF, DOF>::Identity();
    _W_f = Eigen::Matrix<float, DOF, DOF>::Identity();
    return;
}

template<typename Derived>  
void HybridController::calculate_pinv_subspace_matrices(Eigen::MatrixBase<Derived>& S, bool isVelocitySubspace, bool isSpatial) {
    Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, VELOCITY_CONTROL_SUBSPACE_DIM> tempIv;
    Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, FORCE_CONTROL_SUBSPACE_DIM> tempIf;

    if (isVelocitySubspace)
    {
        tempIv = (S.transpose() * _W_v * S).inverse();
        if (isSpatial) {
            _pi_Sv_s = tempIv * S.transpose() * _W_v;
        } else {
            _pi_Sv_c = tempIv * S.transpose() * _W_v;
        }
    } else {
        tempIf = (S.transpose() * _W_f * S).inverse();
        if (isSpatial) {
            _pi_Sf_s = tempIf * S.transpose() * _W_f;
        } else {
            _pi_Sf_c = tempIf * S.transpose() * _W_f;
        }
    }
    
    return;
}

void HybridController::set_desired_state(Eigen::Matrix<float, HYBRID_STATE_DIM, 1> desired_state_received) {
    _D = desired_state_received;
    return;
}
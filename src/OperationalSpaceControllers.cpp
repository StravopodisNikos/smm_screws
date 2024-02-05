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
    // Based on eq.9.30/p.373 /in [1]
    _torque_cmd = _ptr2_screws_dyn_object->MassMatrix() * _y + (_ptr2_screws_dyn_object->CoriolisMatrix() * _dq + _ptr2_screws_dyn_object->GravityVector() + _ptr2_screws_dyn_object->FrictionVector() ) + _ptr2_screws_kin_object->Jop.transpose() * _he;
    return;
}

void ImpedanceController::update_torques(Eigen::Vector3f &torque_out) {
    // Based on eq.9.30/p.373 /in [1]
    torque_out = _ptr2_screws_dyn_object->MassMatrix() * _y + (_ptr2_screws_dyn_object->CoriolisMatrix() * _dq + _ptr2_screws_dyn_object->GravityVector() + _ptr2_screws_dyn_object->FrictionVector() ) + _ptr2_screws_kin_object->Jop.transpose() * _he;
    return;
}

// ****************** HYBRID CONTROLLER ******************

HybridController3::HybridController3() {};
HybridController3::HybridController3(ScrewsKinematics *ptr2kinematics, ScrewsDynamics *ptr2dynamics):  _ptr2_screws_kin_object(ptr2kinematics), _ptr2_screws_dyn_object(ptr2dynamics) {
    _I_dof = Eigen::Matrix<float, DOF, DOF>::Identity();
    _O_dof = Eigen::Matrix<float, DOF, DOF>::Zero();
    initialize_gain_matrices();
    initialize_subspace_matrices();
}

void HybridController3::initialize_gain_matrices() {
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

void HybridController3::initialize_constraint_frame(Eigen::Vector3f pi, Eigen::Vector3f pf, Eigen::Vector3f & zf_s) {
    // Executed in hybrid3_centralized.cpp node 
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
    _Rsc = _gsc.linear(); // save rotation matrix

    // Set the translation part (first three elements of the last column)
    _gsc.translation() = pi;
    
    return;   
}

void HybridController3::initialize_subspace_matrices() {
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

void HybridController3::rotate_subspace_matrices_S() {
    // Executed in hybrid3_centralized.cpp node 
    // The Selection Matrices defined in {C} are expressed in {S}
    _Sv_s = _Rsc * _Sv_c;
    _Sf_s = _Rsc * _Sf_c;
    return;
}

void HybridController3::calculate_pinv_subspace_matrices() {
    // Executed in hybrid3_centralized.cpp node 
    // Implements eq.9.55/pdf.386 /in [1]
    Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, VELOCITY_CONTROL_SUBSPACE_DIM> tempIv_s;
    Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, VELOCITY_CONTROL_SUBSPACE_DIM> tempIv_c;
    Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, FORCE_CONTROL_SUBSPACE_DIM> tempIf_s;
    Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, FORCE_CONTROL_SUBSPACE_DIM> tempIf_c;
    
    tempIv_s = (_Sv_s.transpose() * _W_v * _Sv_s).inverse();
    tempIv_c = (_Sv_c.transpose() * _W_v * _Sv_c).inverse();
    tempIf_s = (_Sf_s.transpose() * _W_f * _Sf_s).inverse();
    tempIf_c = (_Sf_c.transpose() * _W_f * _Sf_c).inverse();

    _pi_Sv_s = tempIv_s * _Sv_s.transpose() * _W_v;
    _pi_Sv_c = tempIv_c * _Sv_c.transpose() * _W_v;
    _pi_Sf_s = tempIf_s * _Sf_s.transpose() * _W_f;
    _pi_Sf_c = tempIf_c * _Sf_c.transpose() * _W_f;
    return;
}

template<typename Derived>  
void HybridController3::calculate_pinv_subspace_matrices(Eigen::MatrixBase<Derived>& S, bool isVelocitySubspace, bool isSpatial) {
    // Implements eq.9.55/pdf.386 /in [1]
    // This is just a practice example of hoe to construct
    // a template that supports input arguments of different
    // matrix dimensions
    // It is NOT USED, because selection matrices are private members
    // and no extra complexity should be added for my simple tasks
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
        // Implements eq.9.55/pdf.386 /in [1]
        tempIf = (S.transpose() * _W_f * S).inverse();
        if (isSpatial) {
            _pi_Sf_s = tempIf * S.transpose() * _W_f;
        } else {
            _pi_Sf_c = tempIf * S.transpose() * _W_f;
        }
    }
    
    return;
}

void HybridController3::set_desired_state(Eigen::Matrix<float, HYBRID_STATE_DIM, 1> desired_state_received) {
    // Must be expressed in {C} frame and in the velocity/force - control subspaces
    _D = desired_state_received;
    set_lamda_desired_S(_D[4]); // assigns 5th element to the lamda desired value
    return;
}

void HybridController3::set_lamda_desired_S(float lamda_desired) {
    _lamda_d << lamda_desired;
    return;
}

void HybridController3::set_error_state(Eigen::Matrix<float, HYBRID_STATE_DIM, 1> current_state_received) {
    // internal usage of the error_state
    _X = _D - current_state_received;
    _x1 = _X.block<VELOCITY_CONTROL_SUBSPACE_DIM, 1>(0, 0); // constraint frame velocity error
    _x2 = _X.block<VELOCITY_CONTROL_SUBSPACE_DIM, 1>(2, 0); // constraint frame position error
    _x3 = _X.block<FORCE_CONTROL_SUBSPACE_DIM, 1>(4, 0); // constraint frame force error
    _x4 = _X.block<FORCE_CONTROL_SUBSPACE_DIM, 1>(5, 0); // constraint frame force sum error
    for (int i = 0; i < HYBRID_STATE_DIM; i++) {
        ROS_INFO("[HybridController/set_error_state] Error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void HybridController3::set_error_state(Eigen::Matrix<float, HYBRID_STATE_DIM, 1> current_state_received, Eigen::Matrix<float, HYBRID_STATE_DIM, 1> & error_state) {
    // makes the error_state available to the main node
    error_state = _D - current_state_received;
    _X = error_state;
    _x1 = _X.block<VELOCITY_CONTROL_SUBSPACE_DIM, 1>(0, 0); // constraint frame velocity error
    _x2 = _X.block<VELOCITY_CONTROL_SUBSPACE_DIM, 1>(2, 0); // constraint frame position error
    _x3 = _X.block<FORCE_CONTROL_SUBSPACE_DIM, 1>(4, 0); // constraint frame force error
    _x4 = _X.block<FORCE_CONTROL_SUBSPACE_DIM, 1>(5, 0); // constraint frame force sum error
    for (int i = 0; i < HYBRID_STATE_DIM; i++) {
        ROS_INFO("[HybridController/set_error_state] Error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void HybridController3::update_error_state() {
    // Pos, Vel, Force measurements are already acquired. The new erros state is
    // computed internally and updated.
    // Error state is calculated in {C} frame.

    // 1. Extract the variables from {S}
    Eigen::Matrix<float, HYBRID_STATE_DIM, 1> current_state;
    _pe_s = _ptr2_screws_kin_object->updatePositionTCP(_q_meas); // set new _pe /in {S}
    _ptr2_screws_kin_object->CartesianVelocity_jacob(_ve_s);     // set new _ve /in {S}
    // update_force_measurements is already executed -> _fe is already set

    // 2. Transfrom from {S} to {C}
    // _ve_s -> _velocity_c
    // _pe_s -> _position_c
    // _fe_s -> _lamda_c // _Integral_lamda_c
    update_velocity_S();
    update_position_S(); // Need Rsc!
    update_lamda_S(); // Needs force measurments! fe_c! -> implemented in other cb fn
                                                        // in the main hybrid3_centralized.cpp node!
    current_state.segment<2>(0) = _velocity_c;
    current_state.segment<2>(2) = _position_c;
    current_state.segment<1>(4) = _lamda_c;
    current_state.segment<1>(5) = _Integral_lamda_c; 

    for (int i = 0; i < HYBRID_STATE_DIM; i++) {
        ROS_INFO("[HybridController/set_error_state] Current_state[ %d ]: %f", i, current_state(i));
    }    

    // 3. Extract error state
    _X = _D - current_state;
    _x1 = _X.block<VELOCITY_CONTROL_SUBSPACE_DIM, 1>(0, 0); // constraint frame velocity error
    _x2 = _X.block<VELOCITY_CONTROL_SUBSPACE_DIM, 1>(2, 0); // constraint frame position error
    _x3 = _X.block<FORCE_CONTROL_SUBSPACE_DIM, 1>(4, 0); // constraint frame force error
    _x4 = _X.block<FORCE_CONTROL_SUBSPACE_DIM, 1>(5, 0); // constraint frame force sum error
    for (int i = 0; i < HYBRID_STATE_DIM; i++) {
        ROS_INFO("[HybridController/set_error_state] Error_state[ %d ]: %f", i, _X(i));
    }
    return;
}

void HybridController3::update_q(float *q_new) {
    for (size_t i = 0; i < DOF; i++) {_q_meas[i] = q_new[i];}
    return;
}

void HybridController3::update_dq(float *dq_new) {
    for (size_t i = 0; i < DOF; i++) {_dq_meas[i] = dq_new[i];}
    return;
}

void HybridController3::update_force_measurements(float *force_meas) {
    for (int i = 0; i < DOF; i++) {
        _f_meas[i] = force_meas[i];
        _fe_c[i] = _f_meas[i]; // here _fe_c ~ he_c (Siciliano), 
                               // index {c} denotes that measurement is expressed 
                               // in constraint frame {C}={T} tool frame
        ROS_INFO("[HybridController/update_force_measurements] Force axis [ %d ]: %f", i, _fe_c[i]);
    }

    return;
}

void HybridController3::update_lamda_C() {
    // Implements eq.9.54/pdf 403/in [1], in {C} frame
    // lamda is vector in the force-controlled-subspace
    // index {c} is redundant for _lamda !!!
    _lamda_c = _pi_Sf_c * _fe_c;
    _Integral_lamda_c += _lamda_c;
    return;
}

void HybridController3::update_lamda_S() {
    // Implements eq.9.54/pdf 403/in [1], in {C} frame
    // lamda is vector in the force-controlled-subspace
    // index {c} is redundant for _lamda !!!
    // Here the tf between sensor frame~constraint frame {C} and
    // spatial frame {S} is executed. Lamda vector must remain 
    // the same for both tfs!
    _fe_s = _Rsc * _fe_c;          // tfs force measurements to spatial frame
    _lamda_c = _pi_Sf_s * _fe_s;   // cuts dimensions to force control subspace
    _Integral_lamda_c += _lamda_c;
    // SOS: _lamda_c can also be regared as lamda_s
    return;
}

void HybridController3::update_velocity_C() {
    // Implements eq.9.60/pdf 405/in [1], using 
    // TCP velocity expressed in {C} frame
    _velocity_c = _pi_Sv_c * _ve_c;
    return;
}

void HybridController3::update_position_C() { 
    // TCP position is expressed in {C} frame
    _position_c = _pi_Sv_c * _pe_c;
    return;
}

void HybridController3::update_velocity_S() {
    // Implements eq.9.60/pdf 405/in [1], using 
    // TCP velocity expressed in {S} frame
    _velocity_c = _pi_Sv_c * _Rsc.inverse() * _ve_s;
    return;
}

void HybridController3::update_position_S() {
     // TCP position is expressed in {S} frame
    _position_c = _pi_Sv_c * _Rsc.inverse() * _pe_s;
    return;
}

void HybridController3::update_inverse_operational_jacob() {
    _iJop = _ptr2_screws_kin_object->Jop.inverse();
    return;
}

void HybridController3::update_inverse_transpose_operational_jacob() {
    _itJop = _ptr2_screws_kin_object->Jop.transpose().inverse();
    return;
}

void HybridController3::update_derivative_operational_jacob() {
    _dtJop = _ptr2_screws_kin_object->dJop;
    return;
}

void HybridController3::calculate_MassMatrix_task_space() {
    // Transforms Mass Matrix to task frame (end-effector).
    // Implements the 1st eq /in p.414(pdf)/[1]
    update_inverse_operational_jacob();
    update_inverse_transpose_operational_jacob();
    _Be = _itJop * _ptr2_screws_dyn_object->MassMatrix() * _iJop;
    return;
}

void HybridController3::calculate_CoriolisVector_task_space() {
    // Transforms Coriolis Vector to task frame (end-effector).
    // Implements the 2nd eq /in p.414(pdf)/[1]    
    update_inverse_operational_jacob();
    update_inverse_transpose_operational_jacob();
    _Ne = _itJop * (_ptr2_screws_dyn_object->CoriolisMatrix() * _dq_meas + _ptr2_screws_dyn_object->GravityVector() + _ptr2_screws_dyn_object->FrictionVector() ) - (_Be * _dtJop * _dq_meas);
    return;
}

void HybridController3::calculate_force_control_component() {
    // Computes f_lamda
    // Implements eq.(9.94) /in p.420(pdf)/[1] 
    _f_lamda = _lamda_d + _Ki_l * _x4;
    return;
}

void HybridController3::calculate_motion_control_component() {
    // Computes alpha_v 
    // Implements eq.(9.95) /in p.420(pdf)/[1] 
    _alpha_v = _Kd_v * _x1 + _Kp_v * _x2; // misses r_d_ddot because it is set to zero for my tasks
    return;
}

void HybridController3::update_torques() {
    // Based on eq.9.91/pdf.419 /in [1]
    _torque_cmd = _Be * _Sv_s * _alpha_v + _Sf_s * _f_lamda + _Ne;
    return;
}

void HybridController3::update_torques(Eigen::Vector3f &torque_out) {
    // Based on eq.9.91/pdf.419 /in [1]
    torque_out = _Be * _Sv_s * _alpha_v + _Sf_s * _f_lamda + _Ne;
    return;
}
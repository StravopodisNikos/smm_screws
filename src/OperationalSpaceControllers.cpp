#include "smm_screws/OperationalSpaceControllers.h"

using namespace OperationalSpaceControllers;

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
        ROS_ERROR("[InverseDynamicsController/initialize_gain_matrices] Failed to load: Kp");
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
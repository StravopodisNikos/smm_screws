#ifndef OPERATIONAL_SPACE_CONTROLLERS_H
#define OPERATIONAL_SPACE_CONTROLLERS_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>
#include "smm_screws/robot_shared.h"
#include <ros/ros.h>

// IDOSC CONSTANTS
#define IDOSC_STATE_DIM     6

// IMPEDANCE CONTROLLER CONSTANTS
#define IMPEDANCE_STATE_DIM 6

// HYBRID CONTROLLER CONSTANTS
#define HYBRID_STATE_DIM                6
#define FORCE_CONTROL_SUBSPACE_DIM      1  // m
#define VELOCITY_CONTROL_SUBSPACE_DIM   2  // (DOF - m)

/*
 *  Special Thanks and Citations:
 *  [1] Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani, and Giuseppe Oriolo. 2010. Robotics: Modelling, Planning and Control. Springer Publishing Company, Incorporated.
 */

namespace OperationalSpaceControllers {

class InverseDynamicsController {
    public:
        InverseDynamicsController();
        InverseDynamicsController(ScrewsKinematics *ptr2kinematics, ScrewsDynamics *ptr2dynamics);

        Eigen::Matrix<float, IDOSC_STATE_DIM, 1> error_state;
        Eigen::Matrix<float, IDOSC_STATE_DIM, IDOSC_STATE_DIM> transition_matrix;
        Eigen::Matrix<float, DOF, DOF> Kp;
        Eigen::Matrix<float, DOF, DOF> Kd;

        // Initialize functions
        void initialize_gain_matrices();

        // Set functions
        //void update_current_state();
        void set_desired_state(Eigen::Matrix<float, IDOSC_STATE_DIM, 1> desired_state_received);    // desired state is 'updated' between segments not execution steps
        
        //void update_error_state();
        void set_error_state(Eigen::Matrix<float, IDOSC_STATE_DIM, 1> current_state_received);
        void set_error_state(Eigen::Matrix<float, IDOSC_STATE_DIM, 1> current_state_received, Eigen::Matrix<float, IDOSC_STATE_DIM, 1> & error_state);
        void set_error_state(float *q_new);
        void set_state_transition_matrix();
        
        // Update functions (functions that update matrices from outside class)
        void update_control_input();
        void update_torques(); // control output command -> torque command
        void update_torques(Eigen::Vector3f &torque_out);
        void update_dq(float *dq_new);

    private:
        ScrewsKinematics *_ptr2_screws_kin_object;
        ScrewsDynamics *_ptr2_screws_dyn_object;
        float _kp;
        float _kd;
        Eigen::Matrix<float, DOF, DOF> _I_dof;
        Eigen::Matrix<float, DOF, DOF> _O_dof;
        Eigen::Matrix<float, DOF, DOF> _Kp;
        Eigen::Matrix<float, DOF, DOF> _Kd;
        Eigen::Matrix<float, DOF, 1> _x1;
        Eigen::Matrix<float, DOF, 1> _x2;
        Eigen::Matrix<float, IDOSC_STATE_DIM, 1> _X;
        Eigen::Matrix<float, DOF, 1> _d1;
        Eigen::Matrix<float, DOF, 1> _d2;
        Eigen::Matrix<float, IDOSC_STATE_DIM, 1> _D;    // Desired state vector
        Eigen::Matrix<float, DOF, 1> _y;                // control input vector
        Eigen::Matrix<float, DOF, 1> _dq;
        Eigen::Matrix<float, DOF, 1> _torque_cmd;
        Eigen::Matrix3f _iJop;
        Eigen::Matrix3f _dtJop;
        
        // Update functions
        void update_inverse_operational_jacob();
        void update_derivative_operational_jacob();
};

class ImpedanceController {
    public:
        ImpedanceController();
        ImpedanceController(ScrewsKinematics *ptr2kinematics, ScrewsDynamics *ptr2dynamics);

        // Initialize functions
        void initialize_gain_matrices();

        // Set functions
        void set_desired_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> desired_state_received);
        void set_error_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> current_state_received);
        void set_error_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> current_state_received, Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> & error_state);
        void set_error_state(float *q_new);

        // Update functions
        void update_control_input();
        void update_torques(); // control output command -> torque command
        void update_torques(Eigen::Vector3f &torque_out);
        void update_dq(float *dq_new);  
        void update_force_measurements(float *force_meas);
              
    private:
        ScrewsKinematics *_ptr2_screws_kin_object;
        ScrewsDynamics *_ptr2_screws_dyn_object;
        float _md;
        float _kp;
        float _kd;
        Eigen::Matrix<float, DOF, DOF> _I_dof;
        Eigen::Matrix<float, DOF, DOF> _O_dof;
        Eigen::Matrix<float, DOF, DOF> _Md;
        Eigen::Matrix<float, DOF, DOF> _Kp;
        Eigen::Matrix<float, DOF, DOF> _Kd;
        Eigen::Matrix<float, DOF, 1> _x1;
        Eigen::Matrix<float, DOF, 1> _x2;
        Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> _X;
        Eigen::Matrix<float, DOF, 1> _d1;
        Eigen::Matrix<float, DOF, 1> _d2;
        Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> _D;    // Desired state vector
        Eigen::Matrix<float, DOF, 1> _y;                    // control input vector
        Eigen::Matrix<float, DOF, 1> _dq;
        Eigen::Matrix<float, DOF, 1> _he;
        Eigen::Matrix<float, DOF, 1> _hA;
        Eigen::Matrix<float, DOF, 1> _torque_cmd;
        Eigen::Matrix3f _iJop;
        Eigen::Matrix3f _dtJop;

        void update_inverse_operational_jacob();
        void update_derivative_operational_jacob();
};

/*
 *  [8-12-23] Hybrid Controller implemented here, has some key limitations:
 *  - used only in 3dof manipulators
 *  - motion on xy plane and force along the z axis.
 *  - only rectilinear path along the x axis (in {C} frame)
 *  - {C} is the fixed constraint frame. origin: initial task point, x_c: pi->pf
 *  - {S} is the fixed robot base frame (spatial frame)
 *  - {T} is the end-effector frame (tool frame)
 */
class HybridController3 {
    public:
        HybridController3();
        HybridController3(ScrewsKinematics *ptr2kinematics, ScrewsDynamics *ptr2dynamics);

        // Initialize functions
        void initialize_gain_matrices();
        void initialize_subspace_matrices();
        void initialize_constraint_frame(Eigen::Vector3f pi, Eigen::Vector3f pf, Eigen::Vector3f & zf_s);
        
        // Set functions
        void set_desired_state(Eigen::Matrix<float, HYBRID_STATE_DIM, 1> desired_state_received);
        void set_error_state(Eigen::Matrix<float, HYBRID_STATE_DIM, 1> current_state_received);
        void set_error_state(Eigen::Matrix<float, HYBRID_STATE_DIM, 1> current_state_received, Eigen::Matrix<float, HYBRID_STATE_DIM, 1> & error_state);
        void set_lamda_desired_S(float lamda_desired);

        // Calculate functions
        template<typename Derived>
        void calculate_pinv_subspace_matrices(Eigen::MatrixBase<Derived>& S, bool isVelocitySubspace, bool isSpatial);  
        void calculate_MassMatrix_task_space();
        void calculate_CoriolisVector_task_space();
        void calculate_motion_control_component();
        void calculate_force_control_component();

        // Update functions
        void update_q(float *q_new) ;
        void update_dq(float *dq_new);  
        void update_force_measurements(float *force_meas);
        void update_error_state();
        void update_lamda_C();
        void update_lamda_S();
        void update_velocity_C();
        void update_velocity_S();
        void update_position_C();
        void update_position_S();
        void update_torques();
        void update_torques(Eigen::Vector3f &torque_out);

    private:
        ScrewsKinematics *_ptr2_screws_kin_object;
        ScrewsDynamics *_ptr2_screws_dyn_object;
        float _ki_l;
        float _kp_v;
        float _kd_v;
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _x1;
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _x2;
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _x3;
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _x4;
        Eigen::Matrix<float, HYBRID_STATE_DIM, 1> _X;
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _d1;
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _d2;
        Eigen::Matrix<float, DOF, 1> _d3;
        Eigen::Matrix<float, DOF, 1> _d4;
        Eigen::Matrix<float, HYBRID_STATE_DIM, 1> _D;    // Desired state vector, always expressed in {C}
        Eigen::Matrix<float, DOF, DOF> _I_dof;
        Eigen::Matrix<float, DOF, DOF> _O_dof;
        Eigen::Matrix<float, DOF, DOF> _Ki_l;
        Eigen::Matrix<float, DOF, DOF> _Kp_v;
        Eigen::Matrix<float, DOF, DOF> _Kd_v;
        Eigen::Matrix<float, DOF, DOF> _W_v; // weight matrices for pseudoinverse calculation of Velocity Subscpace Selection Matrix
        Eigen::Matrix<float, DOF, DOF> _W_f; // weight matrices for pseudoinverse calculation of Force Subscpace Selection Matrix
        Eigen::Matrix<float, DOF, VELOCITY_CONTROL_SUBSPACE_DIM> _Sv_s;
        Eigen::Matrix<float, DOF, VELOCITY_CONTROL_SUBSPACE_DIM> _Sv_c;
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, DOF> _pi_Sv_s;
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, DOF> _pi_Sv_c;
        Eigen::Matrix<float, DOF, FORCE_CONTROL_SUBSPACE_DIM> _Sf_s;
        Eigen::Matrix<float, DOF, FORCE_CONTROL_SUBSPACE_DIM> _Sf_c;
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, DOF> _pi_Sf_s;
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, DOF> _pi_Sf_c;  
        Eigen::Matrix<float, DOF, 1> _q_meas;
        Eigen::Matrix<float, DOF, 1> _dq_meas;
        Eigen::Matrix<float, DOF, 1> _f_meas;
        Eigen::Matrix<float, DOF, 1> _alpha;      // control input vector
        Eigen::Matrix<float, DOF, 1> _torque_cmd; // control output vector: gamma_e
        Eigen::Matrix3f _Rsc;
        Eigen::Isometry3f _gsc;    
        Eigen::Matrix3f _iJop;
        Eigen::Matrix3f _itJop;
        Eigen::Matrix3f _dtJop;
        Eigen::Matrix3f _Be;
        Eigen::Matrix<float, DOF, 1> _Ne;
        Eigen::Matrix<float, DOF, 1> _pe_s;
        Eigen::Matrix<float, DOF, 1> _ve_s;
        Eigen::Matrix<float, DOF, 1> _fe_s;
        Eigen::Matrix<float, DOF, 1> _pe_c;
        Eigen::Matrix<float, DOF, 1> _ve_c;
        Eigen::Matrix<float, DOF, 1> _fe_c;
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _lamda_d;       // desired control subspace R(Sf) vector
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _lamda_c;       // control subspace R(Sf) vector
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _lamda_s;       // control subspace R(Sf) vector
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _f_lamda;       // force control input
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _alpha_v;    // motion control input        
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _velocity_c; // control subspace R(Sv) vector
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _position_c;
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _f_c;
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _Integral_lamda_c;
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _Integral_lamda_s;
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _vd_c; // desired velocity in {C}
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _pd_c; // desired position in {C}
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _fd_c;    // desired force in {C}
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _vd_s; // desired velocity in {C}
        Eigen::Matrix<float, VELOCITY_CONTROL_SUBSPACE_DIM, 1> _pd_s; // desired position in {C}
        Eigen::Matrix<float, FORCE_CONTROL_SUBSPACE_DIM, 1> _fd_s;  
        // Update functions
        void update_inverse_operational_jacob();
        void update_inverse_transpose_operational_jacob();
        void update_derivative_operational_jacob();

};

} // OperationalSpaceControllers namespace

#endif // OPERATIONAL_SPACE_CONTROLLERS_H
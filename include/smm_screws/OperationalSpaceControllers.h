#ifndef OPERATIONAL_SPACE_CONTROLLERS_H
#define OPERATIONAL_SPACE_CONTROLLERS_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>
#include "smm_screws/robot_shared.h"
#include <ros/ros.h>

#define IDOSC_STATE_DIM     6
#define IMPEDANCE_STATE_DIM 6

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

        void initialize_gain_matrices();
        void set_desired_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> desired_state_received);
        void set_error_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> current_state_received);
        void set_error_state(Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> current_state_received, Eigen::Matrix<float, IMPEDANCE_STATE_DIM, 1> & error_state);
        void set_error_state(float *q_new);
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

} // OperationalSpaceControllers namespace

#endif // OPERATIONAL_SPACE_CONTROLLERS_H
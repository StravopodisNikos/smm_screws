#ifndef SCREWS_DYNAMICS_NDOF_H
#define SCREWS_DYNAMICS_NDOF_H

#include <stdexcept>
#include <type_traits>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "smm_screws/robot_parameters.h"
#include "smm_screws/core/ScrewsKinematicsNdof.h"
#include "smm_screws/core/RobotAbstractBaseNdof.h"

/*
 *  C++ library for computation of dynamic matrices for
 *  serial manipulators of fixed/reconfigurable structure,
 *  implementing screw theory tools.
 *
 *  This N-DOF ROS2 version follows the same architecture as
 *  ScrewsKinematicsNdof:
 *    - runtime DOF via _dof
 *    - RobotAbstractBaseNdof robot model
 *    - MAX_DOF storage, loop only to _dof
 *
 *  Special Thanks and Citations:
 *  [1] Murray, R. M., Li, Z., Sastry, S. S., & Sastry, S. S. (1994).
 *      A mathematical introduction to robotic manipulation. CRC press.
 *  [2] Müller, A. (2018). Screw and Lie group theory in multibody kinematics:
 *      Motion representation and recursive kinematics of tree-topology systems.
 *      Multibody System Dynamics, 43(1), 37-70.
 *  [3] Müller, A. (2018). Screw and Lie group theory in multibody dynamics:
 *      recursive algorithms and equations of motion of tree-topology systems.
 *      Multibody System Dynamics, 42(2), 219-248.
 */

class ScrewsDynamicsNdof : public ScrewsKinematicsNdof
{
public:
    static constexpr int MAX_DOF       = robot_params::MAX_DOF;
    static constexpr int MAX_METALINKS = robot_params::MAX_METALINKS;

    ScrewsDynamicsNdof();
    explicit ScrewsDynamicsNdof(RobotAbstractBaseNdof* ptr2abstract_ndof);

    int dof() const noexcept { return _dof; }

    // ============================================================
    // 1) Joint state API (templated: accepts float* or double*)
    // Dynamics version reuses inherited kinematics joint state and
    // additionally stores previous q and delta q for dynamics.
    // ============================================================

    template<typename df_number>
    void updateJointPos(const df_number* q_new)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateJointPos requires floating-point df_number");

        if (!q_new) {
            std::cerr << "[ScrewsDynamicsNdof::updateJointPos] Null q_new pointer\n";
            return;
        }

        if (_dof <= 0 || _dof > MAX_DOF) {
            std::cerr << "[ScrewsDynamicsNdof::updateJointPos] Invalid DOF = "
                      << _dof << "\n";
            return;
        }

        for (int i = 0; i < _dof; ++i) {
            _joint_pos_prev[i]  = _joint_pos[i];
            _joint_pos[i]       = static_cast<float>(q_new[i]);
            _delta_joint_pos[i] = _joint_pos[i] - _joint_pos_prev[i];
        }
    }

    template<typename df_number>
    void updateJointVel(const df_number* dq_new)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateJointVel requires floating-point df_number");

        if (!dq_new) {
            std::cerr << "[ScrewsDynamicsNdof::updateJointVel] Null dq_new pointer\n";
            return;
        }

        if (_dof <= 0 || _dof > MAX_DOF) {
            std::cerr << "[ScrewsDynamicsNdof::updateJointVel] Invalid DOF = "
                      << _dof << "\n";
            return;
        }

        for (int i = 0; i < _dof; ++i) {
            _joint_vel[i] = static_cast<float>(dq_new[i]);
        }
    }

    template<typename df_number>
    void updateJointState(const df_number* q_new,
                          const df_number* dq_new)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateJointState(q,dq) requires floating-point df_number");

        if (!q_new || !dq_new) {
            std::cerr << "[ScrewsDynamicsNdof::updateJointState(q,dq)] Null pointer input\n";
            return;
        }

        if (_dof <= 0 || _dof > MAX_DOF) {
            std::cerr << "[ScrewsDynamicsNdof::updateJointState(q,dq)] Invalid DOF = "
                      << _dof << "\n";
            return;
        }

        for (int i = 0; i < _dof; ++i) {
            _joint_pos_prev[i]  = _joint_pos[i];
            _joint_pos[i]       = static_cast<float>(q_new[i]);
            _delta_joint_pos[i] = _joint_pos[i] - _joint_pos_prev[i];
            _joint_vel[i]       = static_cast<float>(dq_new[i]);
        }
    }

    template<typename df_number>
    void updateJointState(const df_number* q_new,
                          const df_number* dq_new,
                          const df_number* ddq_new)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateJointState(q,dq,ddq) requires floating-point df_number");

        if (!q_new || !dq_new || !ddq_new) {
            std::cerr << "[ScrewsDynamicsNdof::updateJointState(q,dq,ddq)] Null pointer input\n";
            return;
        }

        if (_dof <= 0 || _dof > MAX_DOF) {
            std::cerr << "[ScrewsDynamicsNdof::updateJointState(q,dq,ddq)] Invalid DOF = "
                      << _dof << "\n";
            return;
        }

        for (int i = 0; i < _dof; ++i) {
            _joint_pos_prev[i]  = _joint_pos[i];
            _joint_pos[i]       = static_cast<float>(q_new[i]);
            _delta_joint_pos[i] = _joint_pos[i] - _joint_pos_prev[i];
            _joint_vel[i]       = static_cast<float>(dq_new[i]);
            _joint_accel[i]     = static_cast<float>(ddq_new[i]);
        }
    }

    // ----------------------------------------------------------------
    // 2) Dynamic matrices exported for usage in ROS nodes
    // Stored with MAX_DOF capacity; only top-left/top entries up to _dof
    // are valid for the current robot.
    // ----------------------------------------------------------------
    Eigen::Matrix<float, MAX_DOF, MAX_DOF> MM;   // Mass matrix
    Eigen::Matrix<float, MAX_DOF, MAX_DOF> CM;   // Coriolis matrix
    Eigen::Matrix<float, MAX_DOF, 1> GV;         // Gravity vector
    Eigen::Matrix<float, MAX_DOF, 1> FV;         // Friction vector

    Eigen::Matrix<float, MAX_DOF, MAX_DOF>* ptr2MM;


    // ----------------------------------------------------------------
    // 5. Dynamic matrix API. These are main fns for dynamic modeling - Not ready yet!
    // ----------------------------------------------------------------
    // ============================================================
    // 5.1) Mass matrix computation modes
    // ============================================================
    // Canonical API:
    //   MassMatrix(...)
    //
    // The user selects:
    //   1) representation:
    //        - spatial
    //        - body
    //
    //   2) if body representation is selected, the body-frame choice:
    //        - active joint frame
    //        - local CoM frame
    //
    // Default behavior:
    //   - spatial formulation
    //   - joint-frame flag is ignored in spatial mode
    //
    // This keeps one clean user-facing API while still allowing multiple
    // mathematically distinct internal implementations.
    enum class MassMatrixRepresentation
    {
        SPATIAL,
        BODY
    };

    enum class BodyFrameSelection
    {
        JOINT,
        COM
    };

    // ----------------------------------------------------------------
    // Canonical API for mass matrix
    // ----------------------------------------------------------------
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
    MassMatrix(
        MassMatrixRepresentation representation = MassMatrixRepresentation::SPATIAL,
        BodyFrameSelection body_frame = BodyFrameSelection::JOINT
    );
    //Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> CoriolisMatrix();
    //Eigen::Matrix<float, Eigen::Dynamic, 1> GravityVector();
    //Eigen::Matrix<float, Eigen::Dynamic, 1> GravityVectorAnalytical();
    //Eigen::Matrix<float, Eigen::Dynamic, 1> GravityVectorAnalyticalBody();
    //Eigen::Matrix<float, Eigen::Dynamic, 1> FrictionVector();

    // Fn that returns total potential energy - removed for now
    // float computePotentialEnergy();

protected:
    static constexpr float _g_z = -9.80665f;

    // Joint state storage used by dynamics
    float _delta_joint_pos[MAX_DOF];
    float _joint_pos_prev[MAX_DOF];

    // Link mass matrices
    Eigen::Matrix<float, 6, 6> _Mib[MAX_DOF];   // link mass matrices in body frames
    Eigen::Matrix<float, 6, 6> _Mis[MAX_DOF];   // link mass matrices in spatial/base frame

    // Internal scratch matrices
    Eigen::Matrix<float, 6, 6> _Ml_temp;
    
    Eigen::Matrix<float, MAX_DOF, MAX_DOF> parDerMass;
    Eigen::Matrix<float, MAX_DOF, MAX_DOF> ChristoffelSymbols[MAX_DOF];

    Eigen::Matrix<float, 6, 1> _LieBracketParDer[2]; // fixed size since 2 variables needed for internal computations
    Eigen::Matrix<float, 1, 6> _xi_traspose;

    float _PotEnergy_prev {0.0f};
    float _PotEnergy      {0.0f};

    // Meta / pseudo bookkeeping for later NDOF dynamics migration
    Eigen::Isometry3f _last_expo;
    int _last_twist_cnt {0};

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
    MassMatrix_s(
        BodyFrameSelection body_frame = BodyFrameSelection::JOINT
    );

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
    MassMatrix_b(
        BodyFrameSelection body_frame = BodyFrameSelection::JOINT
    );

    // ----------------------------------------------------------------
    // 2.2 Link Geometric Jacobians
    void computeLinkGeometricJacobians();
    // ----------------------------------------------------------------

private:
    bool _debug_verbosity {true};

    Eigen::Matrix<float, 6, 6> _alpha_temp; // used for adjoint tf for Alpha matrix

    Eigen::Matrix<float, 6, 6> _alpha[2]; // fixed size used for Mass Matrix calculations
    Eigen::Matrix<float, 6, 6> _alphaParDer[5]; // fixed size since 5 variables needed for internal computations
    Eigen::Matrix<float, 1, 1> _parDer_MassIJ_ThetaK;
    Eigen::Matrix<float, 6, 6> _alpha_transpose;

    // COM transformations
    Eigen::Isometry3f* ptr2links_com_tfs[MAX_DOF];
    Eigen::Isometry3f _gsli[MAX_DOF];

    // Link geometric Jacobian columns:
    // Jgl[link_index][joint_index] is a 6x1 column
    Eigen::Matrix<float, 6, 1>* ptr2Jgl[MAX_DOF][MAX_DOF];
    Eigen::Matrix<float, 6, 1>  _Jgl[MAX_DOF][MAX_DOF];

    // Body-frame inertias of real links, expressed either in selected
    // joint/body frames or COM frames depending on the latest call to
    // computeBodyInertiaFromSpatial(...)
    Eigen::Matrix<float, 6, 6> _BodyInertiaFrames[MAX_DOF];
    Eigen::Matrix<float, 6, 6>* _ptr2BodyInertiaFrames[MAX_DOF];

    // ----------------------------------------------------------------
    // 1. Initialization functions
    // ----------------------------------------------------------------
    void initializeLinkMassMatrices();

    // ----------------------------------------------------------------
    // 2. Update fns that extract data from kinematics
    // ----------------------------------------------------------------    
    void updateCOMTfs();

    // ----------------------------------------------------------------
    // 3. Computations for dynamics, implementing screw theory
    // ----------------------------------------------------------------
    // 3.1. Internal functions to set auxiliary matrix elements
    // Alpha matrices of Murray Eq. (4.27)
    // Anatomy/current version:
    //   uses _active_expos_anat[]
    //   no passive transforms needed
    Eigen::Matrix<float, 6, 6> computeAlphaMatrixAnat(size_t i, size_t j);

    Eigen::Matrix<float, 1, 1> computeParDerMassElement(size_t i, size_t j, size_t k);
    
    // 3.2 Tfs Spatial to selected body inertia
    void computeBodyInertiaFromSpatial(BodyFrameSelection body_frame = BodyFrameSelection::JOINT);

    // ----------------------------------------------------------------
    // >> Simple auxiliary functions (for debugging)
    // ----------------------------------------------------------------
    void print66Matrix(Eigen::Matrix<float, 6, 6> matrix);
    void print61Matrix(Eigen::Matrix<float, 6, 1> matrix);
    void print16Matrix(Eigen::Matrix<float, 1, 6> matrix);

};

#endif // SCREWS_DYNAMICS_NDOF_H
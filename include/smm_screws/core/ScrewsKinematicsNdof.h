#ifndef SCREWS_KINEMATICS_NDOF_H
#define SCREWS_KINEMATICS_NDOF_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
#include <type_traits>

#include "smm_screws/robot_parameters.h"
#include "smm_screws/core/ScrewsMain.h"
#include "smm_screws/core/RobotAbstractBaseNdof.h"

// KEY UPGRADES FROM 3DOF ONLY VERSION:
// 1. _dof instead of robot_params::DOF
// 2. RobotAbstractBaseNdof instead of RobotAbstractBase
// 3. arrays sized to MAX_DOF, but loops only to _dof

class ScrewsKinematicsNdof : public ScrewsMain {
public:
    // Alias to global robot parameters so everything stays consistent.
    static constexpr int MAX_DOF       = robot_params::MAX_DOF;
    static constexpr int MAX_METALINKS = robot_params::MAX_METALINKS;          

    explicit ScrewsKinematicsNdof(RobotAbstractBaseNdof* ptr2abstract_ndof);

    // Ndof pseudo transforms
    void initializePseudoTfs();

    int dof() const noexcept { return _dof; }

    // ============================================================
    // 1) Joint state API (templated: accepts float* or double*)
    // ============================================================

    template<typename df_number>
    void updateJointState(const df_number* q_new,
                          const df_number* dq_new,
                          const df_number* ddq_new)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateJointState requires floating-point df_number");

        if (!q_new || !dq_new || !ddq_new) {
            std::cerr << "[ScrewsKinematicsNdof::updateJointState] Null pointer input\n";
            return;
        }

        for (int i = 0; i < _dof; ++i) {
            _joint_pos[i]   = static_cast<float>(q_new[i]);
            _joint_vel[i]   = static_cast<float>(dq_new[i]);
            _joint_accel[i] = static_cast<float>(ddq_new[i]);
        }
    }

    template<typename df_number>
    void updateJointState(const df_number* q_new,
                          const df_number* dq_new)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateJointState requires floating-point df_number");

        if (!q_new || !dq_new) {
            std::cerr << "[ScrewsKinematicsNdof::updateJointState(q,dq)] Null pointer input\n";
            return;
        }

        for (int i = 0; i < _dof; ++i) {
            _joint_pos[i] = static_cast<float>(q_new[i]);
            _joint_vel[i] = static_cast<float>(dq_new[i]);
            // accelerations left as-is
        }
    }

    // ============================================================
    // 2) Exponentials (templated on df_number*, still using float math)
    // ============================================================

    template<typename df_number>
    void setExponentials(const df_number* q)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "setExponentials requires floating-point df_number");

        if (!q) {
            std::cerr << "[ScrewsKinematicsNdof::setExponentials] WARNING: q is null\n";
            return;
        }

        for (int i = 0; i < _dof; ++i) {
            float qi = static_cast<float>(q[i]);
            _active_expos[i] =
                twistExp(_ptr2abstract_ndof->active_twists[i], qi);
        }
    }

    template<typename df_number>
    void setExponentialsAnat(const df_number* q)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "setExponentialsAnat requires floating-point df_number");

        if (!q) {
            std::cerr << "[ScrewsKinematicsNdof::setExponentialsAnat] WARNING: q is null\n";
            return;
        }

        for (int i = 0; i < _dof; ++i) {
            float qi = static_cast<float>(q[i]);
            _active_expos_anat[i] =
                twistExp(_ptr2abstract_ndof->active_twists_anat[i], qi);
        }
    }

    // ============================================================
    // 3) Relative TFs and local screws (Ndof versions of Bi / iXi)
    // ============================================================

    void initializeRelativeTfs();
    void initializeLocalScrewCoordVectors();
    void initializeReferenceAnatomyActiveTwists();
    void initializeReferenceAnatomyActiveTfs();

    // ============================================================
    // 4) Forward Kinematics (templated; float/double → float)
    // ============================================================

    // Generic FK for {T} frame (TCP) given q[0..dof-1] (float or double)
    template<typename df_number>
    void ScrewsKinematicsNdof::ForwardKinematicsTCP(const df_number* q)
    {
        static_assert(std::is_floating_point<df_number>::value,
                    "ForwardKinematicsTCP requires floating-point df_number");

        if (!q) {
            std::cerr << "[ScrewsKinematicsNdof::ForwardKinematicsTCP(q)] WARNING: q is null\n";
            return;
        }

        if (!_ptr2abstract_ndof) {
            std::cerr << "[ScrewsKinematicsNdof::ForwardKinematicsTCP(q)] "
                        "RobotAbstractBaseNdof pointer is null\n";
            return;
        }

        _debug_verbosity = false;

        // IMPORTANT:
        // Use CURRENT / ANATOMY twists (active_twists_anat),
        // not reference twists. This already includes the effect of pseudos.
        setExponentialsAnat(q);

        Eigen::Isometry3f prefix = Eigen::Isometry3f::Identity();

        // Joint frames (current anatomy)
        for (int i = 0; i < _dof; ++i) {
            prefix    = prefix * _active_expos[i];
            _g[i]     = prefix * *(_ptr2abstract_ndof->gsai_test_ptr[i]);
        }

        // TCP frame at index _dof (current anatomy)
        _g[_dof] = prefix * *(_ptr2abstract_ndof->gsai_test_ptr[_dof]);
        _gst     = _g[_dof];

        if (_debug_verbosity) {
            std::cout << "[ForwardKinematicsTCPNdof] gst =\n" << _gst.matrix() << '\n';
        }
    }

    // FK using internally stored _joint_pos
    void ForwardKinematicsTCP();

    // ============================================================
    // 5) Position-only utility (templated)
    // ============================================================

    // q as raw pointer (float* or double*)
    template<typename df_number>
    Eigen::Vector3f updatePositionTCP(const df_number* q)
    {
        ForwardKinematicsTCP(q);
        Eigen::Vector3f p;
        p << _gst(0, 3), _gst(1, 3), _gst(2, 3);
        return p;
    }

    // q as Eigen vector (float or double)
    template<typename Derived>
    Eigen::Vector3f updatePositionTCP(const Eigen::MatrixBase<Derived>& q)
    {
        using df_number = typename Derived::df_number;
        static_assert(std::is_floating_point<df_number>::value,
                      "updatePositionTCP(Eigen) requires floating-point df_number");

        if (q.size() < _dof) {
            std::cerr << "[ScrewsKinematicsNdof::updatePositionTCP(Eigen)] "
                         "Input vector too small (size=" << q.size()
                      << ", dof=" << _dof << ")\n";
            return Eigen::Vector3f::Zero();
        }

        float qf[MAX_DOF];
        for (int i = 0; i < _dof; ++i) {
            qf[i] = static_cast<float>(q(i));
        }

        return updatePositionTCP(qf);
    }

    // Getter for TCP pose
    const Eigen::Isometry3f& getTcpPose() const { return _gst; }

    // ============================================================
    // 6) Jacobians (tool frame, spatial & body)
    // ============================================================

    // Assumes:
    //   - initializeLocalScrewCoordVectors() was called once
    //   - ForwardKinematicsTCP(...) was run for current q
    void computeSpatialJacobianTCP1();  // Ad(g_i) * iXi[i] --> fills _Jsp_tool (6 x _dof)
    void computeSpatialJacobianTCP2();  // Ad(g_i * g_ref_i^{-1}) * active_twists[i] --> fills _Jsp_tool (6 x _dof)
    void computeBodyJacobianTCP1();  // Tool 1: Ad(g_T⁻¹ g_i) * iXi[i]  --> fills _Jbd_tool (6 x _dof)
    void computeBodyJacobianTCP2();  // Tool 2: Ad(g_T⁻¹ g_i g_ref_i⁻¹) * active_twists[i] --> fills _Jbd_tool (6 x _dof)   
    
    // Convenience getters (return 6 x dof blocks)
    Eigen::Matrix<float, 6, Eigen::Dynamic> getSpatialJacobianTCP() const;
    Eigen::Matrix<float, 6, Eigen::Dynamic> getBodyJacobianTCP() const;

private:
    RobotAbstractBaseNdof* _ptr2abstract_ndof {nullptr};
    int _dof {0};

    // --- Ndof metamorphic link data ---
    int _total_pseudojoints {0};
    int _meta_pseudojoints[MAX_METALINKS] {0, 0, 0}; // per meta-link

    Eigen::Isometry3f _Pi[MAX_METALINKS];  // anatomy transforms per meta-link
    Eigen::Isometry3f _last_expo;          // running product within meta-link
    int _last_twist_cnt {0};               // global index into passive twists

    // Exponentials - Ndof equivalents of the 3-DOF private members
    Eigen::Isometry3f _active_expos[MAX_DOF];       // exp(ξ_ref_i * q_i)
    Eigen::Isometry3f _active_expos_anat[MAX_DOF];  // exp(ξ_anat_i * q_i)

    // Joint State    
    float _joint_pos[MAX_DOF];
    float _joint_vel[MAX_DOF];
    float _joint_accel[MAX_DOF];

    // Transformations
    Eigen::Isometry3f _gst;                         // TCP pose
    Eigen::Isometry3f _g[MAX_DOF + 1];              // joint frames + TCP
    Eigen::Isometry3f _Bi[MAX_DOF + 1];             // relative frames C_i,i-1(0)
    
    // R6 twists
    Eigen::Matrix<float, 6, 1> _iXi[MAX_DOF + 1];   // local screw coords

    // Jacobians (tool)
    Eigen::Matrix<float, 6, MAX_DOF> _Jsp_tool;     // spatial TCP Jacobian (cols 0.._dof-1)
    Eigen::Matrix<float, 6, MAX_DOF> _Jbd_tool;     // body TCP Jacobian    (cols 0.._dof-1)

    // Aux for adjoint operations
    Eigen::Matrix<float, 6, 6> _ad;

    bool _debug_verbosity {false};
};

#endif // SCREWS_KINEMATICS_NDOF_H

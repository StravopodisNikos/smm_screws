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
	enum class JacobianSelection { SPATIAL, BODY};
	typedef JacobianSelection typ_jacobian;

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
    void initializeSpatialJointScrewCoordVectors();
    void initializeReferenceAnatomyActiveTwists();
    void initializeReferenceAnatomyActiveTfs();
    void initializeHomeAnatomyActiveTfs();
    void initializeHomeAnatomyCOMTfs();

    const Eigen::Matrix<float, 6, 1>& getSpatialJointScrewCoordVector(int i) const;

    // ============================================================
    // 4a) Forward Kinematics (templated; float/double → float)
    // ============================================================

    // Generic FK for {T} frame (TCP) given q[0..dof-1] (float or double)
    template<typename df_number>
    void ForwardKinematicsTCP(const df_number* q)
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
            prefix    = prefix * _active_expos_anat[i];
            _g[i]     = prefix * *(_ptr2abstract_ndof->gsai_test_ptr[i]);
        }

        // TCP frame at index _dof (current anatomy)
        _g[_dof] = prefix * *(_ptr2abstract_ndof->gsai_test_ptr[_dof]);
        _gst     = _g[_dof];

        if (_debug_verbosity) {
            std::cout << "[ForwardKinematicsTCPNdof] gst =\n" << _gst.matrix() << '\n';
        }

        _is_operational_jacobian_valid = false; // to ensure warning is on if someone asks for Jop after joint state is updated
    }

    // FK using internally stored _joint_pos
    void ForwardKinematicsTCP();

    // ============================================================
    // 4b) Forward Kinematics for link COM frames
    // ============================================================

    // Generic FK for link COM frames given q[0..dof-1] (float or double)
    template<typename df_number>
    void ForwardKinematicsCOM(const df_number* q)
    {
        static_assert(std::is_floating_point<df_number>::value,
                    "ForwardKinematicsCOM requires floating-point df_number");

        if (!q) {
            std::cerr << "[ScrewsKinematicsNdof::ForwardKinematicsCOM(q)] WARNING: q is null\n";
            return;
        }

        if (!_ptr2abstract_ndof) {
            std::cerr << "[ScrewsKinematicsNdof::ForwardKinematicsCOM(q)] "
                        "RobotAbstractBaseNdof pointer is null\n";
            return;
        }

        _debug_verbosity = false;

        // IMPORTANT:
        // Use CURRENT / ANATOMY twists (active_twists_anat),
        // not reference twists. This already includes the effect of pseudos.
        setExponentialsAnat(q);

        Eigen::Isometry3f prefix = Eigen::Isometry3f::Identity();

        // Link COM frames (current anatomy)
        // One COM frame per real link, so only 0.._dof-1
        for (int i = 0; i < _dof; ++i) {
            prefix = prefix * _active_expos_anat[i];
            _gl[i] = prefix * *(_ptr2abstract_ndof->gsli_test_ptr[i]);
        }

        if (_debug_verbosity) {
            for (int i = 0; i < _dof; ++i) {
                std::cout << "[ForwardKinematicsCOMNdof] gl[" << i << "] =\n"
                        << _gl[i].matrix() << '\n';
            }
        }
    }

    void ForwardKinematicsCOM();

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

    // =================================================================
    // 5.1) Velocity-only utility (templated). Uses position only internal
    // =================================================================
    // How to call:
    // auto & kin = robot_context_ndof_->get_kinematics();
    // 1) Update joint state (q & dq) – dq needed for velocity
    // kin.updateJointState(q.data(), dq.data());
    // 2) Cartesian velocity in base frame
    // Eigen::Vector3f v_tcp = kin.updateVelocityTCP(q.data());
    // or, if you have an Eigen vector q_eig:
    // Eigen::Vector3f v_tcp2 = kin.updateVelocityTCP(q_eig);

    template<typename df_number>
    Eigen::Vector3f updateVelocityTCP(const df_number* q)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateVelocityTCP(q) requires floating-point df_number");

        if (!q) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCP(q)] WARNING: q is null\n";
            return Eigen::Vector3f::Zero();
        }

        if (_dof <= 0) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCP(q)] DOF <= 0\n";
            return Eigen::Vector3f::Zero();
        }

        // 1) Forward kinematics at this configuration
        //    (updates _gst and _g[0.._dof])
        ForwardKinematicsTCP(q);

        // 2) Spatial Jacobian at TCP for current q
        //    (fills internal _Jsp_tool 6 x _dof)
        computeSpatialJacobianTCP2();   // same choice as your viz node

        // 3) Build spatial velocity twist Vsp_tool_twist from Jsp and _joint_vel
        //    NOTE: _joint_vel[] must have been updated earlier via updateJointState(...)
        Eigen::Matrix<float, Eigen::Dynamic, 1> dq_vector(_dof);
        for (int i = 0; i < _dof; ++i) {
            dq_vector(i) = _joint_vel[i];
        }

        Eigen::Matrix<float, 6, Eigen::Dynamic> Jsp = getSpatialJacobianTCP();
        _Vsp_twist_tcp = Jsp * dq_vector;      // 6x1

        formTwist(_X_se3, _Vsp_twist_tcp);

        Eigen::Vector4f p;
        p << _gst(0, 3), _gst(1, 3), _gst(2, 3), 1.0f;

        Eigen::Vector4f v4 = _X_se3 * p;

        Eigen::Vector3f v3;
        v3 << v4(0), v4(1), v4(2);     

        if (_debug_verbosity) {
            std::cout << "[ScrewsKinematicsNdof::updateVelocityTCP(q)] "
                      << "v = [" << v3(0) << ", " << v3(1) << ", " << v3(2) << "]\n";
        }

        return v3;
    }

    // q as Eigen vector (float or double)
    template<typename Derived>
    Eigen::Vector3f updateVelocityTCP(const Eigen::MatrixBase<Derived>& q)
    {
        using Scalar = typename Derived::Scalar;
        static_assert(std::is_floating_point<Scalar>::value,
                      "updateVelocityTCP(Eigen) requires floating-point Scalar");

        if (q.size() < _dof) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCP(Eigen)] "
                         "Input vector too small (size=" << q.size()
                      << ", dof=" << _dof << ")\n";
            return Eigen::Vector3f::Zero();
        }

        float qf[MAX_DOF];
        for (int i = 0; i < _dof; ++i) {
            qf[i] = static_cast<float>(q(i));
        }

        return updateVelocityTCP(qf);
    }

    // =================================================================
    // 5.1b) Velocity-only utility from HYBRID twist
    // =================================================================
    // Uses:
    //   - ForwardKinematicsTCP(q)
    //   - computeBodyJacobiansFrames1()   (or 2, depending on your pipeline)
    //   - computeHybridJacobianTCP()
    //   - computeHybridVelocityTwistTCP()
    //
    // Returns Cartesian TCP linear velocity directly from the first 3
    // elements of the hybrid velocity twist wrt the inertial/base frame.

    template<typename df_number>
    Eigen::Vector3f updateVelocityTCPHybrid(const df_number* q)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateVelocityTCPHybrid(q) requires floating-point df_number");

        _debug_verbosity = false;

        if (!q) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCPHybrid(q)] "
                      << "WARNING: q is null\n";
            return Eigen::Vector3f::Zero();
        }

        if (_dof <= 0) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCPHybrid(q)] "
                      << "DOF <= 0\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 1) Update FK for current configuration
        // -----------------------------------------------------------------
        ForwardKinematicsTCP(q);

        const Eigen::Matrix4f gst_matrix = _gst.matrix();
        if (gst_matrix.rows() != 4 || gst_matrix.cols() != 4) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCPHybrid(q)] "
                      << "Invalid TCP transform dimensions\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 2) Compute body Jacobians for actual frames + TCP
        //
        // IMPORTANT:
        // computeHybridJacobianTCP() assumes the frame body Jacobians already
        // exist. We use version 1 here; if your chosen pipeline is version 2,
        // replace with computeBodyJacobiansFrames2().
        // -----------------------------------------------------------------
        computeBodyJacobiansFrames1();

        // -----------------------------------------------------------------
        // 3) Compute TCP hybrid Jacobian
        // -----------------------------------------------------------------
        computeHybridJacobianTCP();

        Eigen::Matrix<float, 6, Eigen::Dynamic> Jh = getHybridJacobianTCP();
        if (Jh.rows() != 6 || Jh.cols() != _dof) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCPHybrid(q)] "
                      << "Hybrid Jacobian has invalid dimensions: "
                      << Jh.rows() << "x" << Jh.cols()
                      << " (expected 6x" << _dof << ")\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 4) Compute TCP hybrid velocity twist
        // -----------------------------------------------------------------
        computeHybridVelocityTwistTCP();

        const Eigen::Matrix<float, 6, 1>& Vh = getHybridVelocityTwistTCP();
        if (Vh.size() != 6) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCPHybrid(q)] "
                      << "Hybrid velocity twist has invalid size\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 5) Direct extraction:
        //    first 3 components of hybrid velocity twist are the TCP
        //    linear velocity wrt inertial/base frame
        // -----------------------------------------------------------------
        Eigen::Vector3f v_tcp = Vh.block<3,1>(0,0);

        if (_debug_verbosity) {
            std::cout << "[ScrewsKinematicsNdof::updateVelocityTCPHybrid(q)] "
                      << "v_hybrid = [" << v_tcp(0) << ", "
                      << v_tcp(1) << ", "
                      << v_tcp(2) << "]\n";
        }

        return v_tcp;
    }

    // q as Eigen vector (float or double)
    template<typename Derived>
    Eigen::Vector3f updateVelocityTCPHybrid(const Eigen::MatrixBase<Derived>& q)
    {
        using Scalar = typename Derived::Scalar;
        static_assert(std::is_floating_point<Scalar>::value,
                      "updateVelocityTCPHybrid(Eigen) requires floating-point Scalar");

        if (q.size() < _dof) {
            std::cerr << "[ScrewsKinematicsNdof::updateVelocityTCPHybrid(Eigen)] "
                      << "Input vector too small (size=" << q.size()
                      << ", dof=" << _dof << ")\n";
            return Eigen::Vector3f::Zero();
        }

        float qf[MAX_DOF];
        for (int i = 0; i < _dof; ++i) {
            qf[i] = static_cast<float>(q(i));
        }

        return updateVelocityTCPHybrid(qf);
    }

    // =================================================================
    // 5.2) Acceleration-only utility (templated)
    // =================================================================
    // N-DOF version of:
    //   ScrewsKinematics::CartesianAcceleration_twist(...)
    //
    // Uses the already split internal computations:
    //   - ForwardKinematicsTCP(q)
    //   - computeSpatialJacobianTCP2()
    //   - computeSpatialVelocityTwistTCP()
    //   - computeDtSpatialVelocityTwistTCP()
    //
    // Returns Cartesian acceleration of the TCP point in the base frame.

    template<typename df_number>
    Eigen::Vector3f updateAccelerationTCP(const df_number* q)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateAccelerationTCP(q) requires floating-point df_number");

        _debug_verbosity = false;

        if (!q) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "WARNING: q is null\n";
            return Eigen::Vector3f::Zero();
        }

        if (_dof <= 0) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "DOF <= 0\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 1) Update FK for current configuration
        // -----------------------------------------------------------------
        ForwardKinematicsTCP(q);

        // Strict check: TCP pose must exist as 4x4 isometry
        const Eigen::Matrix4f gst_matrix = _gst.matrix();
        if (gst_matrix.rows() != 4 || gst_matrix.cols() != 4) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "Invalid TCP transform dimensions\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 2) Compute spatial Jacobian and spatial velocity twist
        //    Keep same choice as updateVelocityTCP(): Jacobian version 2
        // -----------------------------------------------------------------
        computeSpatialJacobianTCP2();
        computeSpatialVelocityTwistTCP();

        // -----------------------------------------------------------------
        // 3) Compute time derivative of spatial velocity twist
        //    This uses current _joint_vel and _joint_accel
        // -----------------------------------------------------------------
        computeDtSpatialVelocityTwistTCP();

        // -----------------------------------------------------------------
        // 4) Convert twists to se(3) matrices
        // -----------------------------------------------------------------
        Eigen::Matrix4f dV_twist_matrix = Eigen::Matrix4f::Zero();
        Eigen::Matrix4f V_twist_matrix  = Eigen::Matrix4f::Zero();

        formTwist(dV_twist_matrix, _dVsp_twist_tcp);
        formTwist(V_twist_matrix,  _Vsp_twist_tcp);

        if (dV_twist_matrix.rows() != 4 || dV_twist_matrix.cols() != 4) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "dV twist matrix has invalid dimensions\n";
            return Eigen::Vector3f::Zero();
        }

        if (V_twist_matrix.rows() != 4 || V_twist_matrix.cols() != 4) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "V twist matrix has invalid dimensions\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 5) Build TCP point in homogeneous form
        // -----------------------------------------------------------------
        Eigen::Vector4f p_h;
        p_h << _gst(0, 3), _gst(1, 3), _gst(2, 3), 1.0f;

        if (p_h.size() != 4) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "Homogeneous TCP point has invalid size\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 6) Compute TCP Cartesian velocity from current spatial twist
        //    v_qs = V_twist_matrix * p_h
        // -----------------------------------------------------------------
        Eigen::Vector4f v_qs = V_twist_matrix * p_h;

        if (v_qs.size() != 4) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "Computed Cartesian velocity has invalid size\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 7) Cartesian acceleration:
        //    a_qs = dV_twist_matrix * p_h + V_twist_matrix * v_qs
        // -----------------------------------------------------------------
        Eigen::Vector4f a_qs = dV_twist_matrix * p_h + V_twist_matrix * v_qs;

        if (a_qs.size() != 4) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "Computed Cartesian acceleration has invalid size\n";
            return Eigen::Vector3f::Zero();
        }

        Eigen::Vector3f a3;
        a3 << a_qs(0), a_qs(1), a_qs(2);

        if (_debug_verbosity) {
            std::cout << "[ScrewsKinematicsNdof::updateAccelerationTCP(q)] "
                      << "a = [" << a3(0) << ", " << a3(1) << ", " << a3(2) << "]\n";
        }

        return a3;
    }

    // q as Eigen vector (float or double)
    template<typename Derived>
    Eigen::Vector3f updateAccelerationTCP(const Eigen::MatrixBase<Derived>& q)
    {
        using Scalar = typename Derived::Scalar;
        static_assert(std::is_floating_point<Scalar>::value,
                      "updateAccelerationTCP(Eigen) requires floating-point Scalar");

        if (q.size() < _dof) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCP(Eigen)] "
                      << "Input vector too small (size=" << q.size()
                      << ", dof=" << _dof << ")\n";
            return Eigen::Vector3f::Zero();
        }

        float qf[MAX_DOF];
        for (int i = 0; i < _dof; ++i) {
            qf[i] = static_cast<float>(q(i));
        }

        return updateAccelerationTCP(qf);
    }

    // =================================================================
    // 5.2b) Acceleration-only utility from HYBRID twist derivative
    // =================================================================
    // Uses:
    //   - ForwardKinematicsTCP(q)
    //   - computeBodyJacobiansFrames1()   (or 2, depending on your chosen pipeline)
    //   - computeHybridJacobianTCP()
    //   - computeHybridVelocityTwistTCP()
    //   - computeDtHybridVelocityTwistTCP()
    //
    // Returns Cartesian TCP acceleration directly from the first 3 elements
    // of the hybrid acceleration twist wrt the inertial/base frame.

    template<typename df_number>
    Eigen::Vector3f updateAccelerationTCPHybrid(const df_number* q)
    {
        static_assert(std::is_floating_point<df_number>::value,
                      "updateAccelerationTCPHybrid(q) requires floating-point df_number");

        _debug_verbosity = false;

        if (!q) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCPHybrid(q)] "
                      << "WARNING: q is null\n";
            return Eigen::Vector3f::Zero();
        }

        if (_dof <= 0) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCPHybrid(q)] "
                      << "DOF <= 0\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 1) Update FK for current configuration
        // -----------------------------------------------------------------
        ForwardKinematicsTCP(q);

        const Eigen::Matrix4f gst_matrix = _gst.matrix();
        if (gst_matrix.rows() != 4 || gst_matrix.cols() != 4) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCPHybrid(q)] "
                      << "Invalid TCP transform dimensions\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 2) Compute body Jacobians for actual frames + TCP
        //
        // IMPORTANT:
        // computeHybridJacobianTCP() assumes the frame body Jacobians already
        // exist. We use version 1 here; if your chosen pipeline is version 2,
        // replace with computeBodyJacobiansFrames2().
        // -----------------------------------------------------------------
        computeBodyJacobiansFrames1();

        // -----------------------------------------------------------------
        // 3) Compute TCP hybrid Jacobian
        // -----------------------------------------------------------------
        computeHybridJacobianTCP();

        // Basic consistency check
        Eigen::Matrix<float, 6, Eigen::Dynamic> Jh = getHybridJacobianTCP();
        if (Jh.rows() != 6 || Jh.cols() != _dof) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCPHybrid(q)] "
                      << "Hybrid Jacobian has invalid dimensions: "
                      << Jh.rows() << "x" << Jh.cols()
                      << " (expected 6x" << _dof << ")\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 4) Compute TCP hybrid velocity twist
        // -----------------------------------------------------------------
        computeHybridVelocityTwistTCP();

        const Eigen::Matrix<float, 6, 1>& Vh = getHybridVelocityTwistTCP();
        if (Vh.size() != 6) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCPHybrid(q)] "
                      << "Hybrid velocity twist has invalid size\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 5) Compute TCP hybrid acceleration twist (Mueller Eq. 29 path)
        // -----------------------------------------------------------------
        computeDtHybridVelocityTwistTCP();

        const Eigen::Matrix<float, 6, 1>& dVh = getDtHybridVelocityTwistTCP();
        if (dVh.size() != 6) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCPHybrid(q)] "
                      << "Hybrid acceleration twist has invalid size\n";
            return Eigen::Vector3f::Zero();
        }

        // -----------------------------------------------------------------
        // 6) Direct extraction:
        //    first 3 components of hybrid acceleration twist are the TCP
        //    linear acceleration wrt inertial/base frame
        // -----------------------------------------------------------------
        Eigen::Vector3f a_tcp = dVh.block<3,1>(0,0);

        if (_debug_verbosity) {
            std::cout << "[ScrewsKinematicsNdof::updateAccelerationTCPHybrid(q)] "
                      << "a_hybrid = [" << a_tcp(0) << ", "
                      << a_tcp(1) << ", "
                      << a_tcp(2) << "]\n";
        }

        return a_tcp;
    }

    // q as Eigen vector (float or double)
    template<typename Derived>
    Eigen::Vector3f updateAccelerationTCPHybrid(const Eigen::MatrixBase<Derived>& q)
    {
        using Scalar = typename Derived::Scalar;
        static_assert(std::is_floating_point<Scalar>::value,
                      "updateAccelerationTCPHybrid(Eigen) requires floating-point Scalar");

        if (q.size() < _dof) {
            std::cerr << "[ScrewsKinematicsNdof::updateAccelerationTCPHybrid(Eigen)] "
                      << "Input vector too small (size=" << q.size()
                      << ", dof=" << _dof << ")\n";
            return Eigen::Vector3f::Zero();
        }

        float qf[MAX_DOF];
        for (int i = 0; i < _dof; ++i) {
            qf[i] = static_cast<float>(q(i));
        }

        return updateAccelerationTCPHybrid(qf);
    }

    // ============================================================
    // 6.1) Jacobians (tool frame, spatial & body)
    // ============================================================

    // Assumes:
    //   - initializeLocalScrewCoordVectors() was called once
    //   - ForwardKinematicsTCP(...) was run for current q
    void computeSpatialJacobianTCP1();  // Ad(g_i) * iXi[i] --> fills _Jsp_tool (6 x _dof)
    void computeSpatialJacobianTCP2();  // Ad(g_i * g_ref_i^{-1}) * active_twists[i] --> fills _Jsp_tool (6 x _dof)
    void computeSpatialJacobianTCP3();  // J^s_tool = [xi'_1  xi'_2 ... xi'_n], xi'_i = Ad_{ exp(xi_anat_1 * theta_1) * . . . * exp(xi_anat{i-1} * theta_{i-1}}) } * Xi_anat[i] --> fills _Jsp_tool (6 x _dof)
    void computeBodyJacobianTCP1();  // Tool 1: Ad(g_T⁻¹ g_i) * iXi[i]  --> fills _Jbd_tool (6 x _dof)
    void computeBodyJacobianTCP2();  // Tool 2: Ad(g_T⁻¹ g_i g_ref_i⁻¹) * active_twists[i] --> fills _Jbd_tool (6 x _dof)   
    void computeBodyJacobianTCP3();  // Tool 3: J^s_tool_b(q) = [ xi'_1  xi'_2  ...  xi'_n ], xi'_i = Ad_{ exp(xi_i * theta_i) * ... * exp(xi_n * theta_n) * gst_0 }^{-1} * xi_i
    // Convenience getters (return 6 x dof blocks)
    Eigen::Matrix<float, 6, Eigen::Dynamic> getSpatialJacobianTCP() const;
    Eigen::Matrix<float, 6, Eigen::Dynamic> getBodyJacobianTCP() const;

    const Eigen::Isometry3f& getJointFrame(int i) const
    {
        if (i < 0 || i > _dof) {
            throw std::out_of_range(
                "[ScrewsKinematicsNdof::getJointFrame] index out of range");
        }
        return _g[i];
    }

    // ============================================================
    // 6.2) Jacobians (body @ joint+tcp frames)
    // ============================================================

    // Assumes:
    // - initializeLocalScrewCoordVectors()         was called to generate iXi used in eq.4.16/Mueller paper, 1st ""=""
    // - initializeSpatialJointScrewCoordVectors(); was called to generate Yi used in eq.4.16/Mueller paper, 2nd ""=""
    // - initializeHomeAnatomyActiveTfs();          was called to generate Ai
    void computeBodyJacobiansFrames1();
    void computeBodyJacobiansFrames2();
    // Convenience getter for a single column J^b_(frame_index, joint_index)
    const Eigen::Matrix<float, 6, 1>& getBodyJacobianFrame(int frameIndex, int jointIndex) const;


    // Assumes:
    // - initializeLocalScrewCoordVectors()         was called to generate iXi used in eq.4.16/Mueller paper, 1st ""=""
    // - initializeSpatialJointScrewCoordVectors(); was called to generate Yi used in eq.4.16/Mueller paper, 2nd ""=""
    // - initializeHomeAnatomyCOMTfs();          was called to generate Ai for COM frames
    void computeBodyCOMJacobiansFrames();
    // Convenience getter for a single column J^b_(frame_index, joint_index)
    const Eigen::Matrix<float, 6, 1>& getBodyCOMJacobianFrame(int frameIndex, int jointIndex) const;

    // ============================================================
    // 6.3) TCP Jacobian (Operational Space jacobian) 
    //      - this is hybrid expression. 
    //      - measured @ body-tcp. 
    //      - expressed @ base (inertial/spatial)
    // ============================================================
    void computeHybridJacobianTCP();

    Eigen::Matrix<float, 6, Eigen::Dynamic> getHybridJacobianTCP() const;

    Eigen::Matrix<float, 6, Eigen::Dynamic> bodyToHybridJacobian(const Eigen::Isometry3f& C_frame, const Eigen::Matrix<float, 6, Eigen::Dynamic>& Jb) const;

    // ============================================================
    // 7.1) TCP velocity/acceleration twists (spatial/body) from Jacobian and dq
    // ============================================================
    void computeSpatialVelocityTwistTCP();
    void computeBodyVelocityTwistTCP();

    const Eigen::Matrix<float, 6, 1>& getSpatialVelocityTwistTCP() const
    {
        return _Vsp_twist_tcp;
    }

    const Eigen::Matrix<float, 6, 1>& getBodyVelocityTwistTCP() const
    {
        return _Vbd_twist_tcp;
    }

    void computeDtSpatialVelocityTwistTCP();
    void computeDtBodyVelocityTwistTCP();

    const Eigen::Matrix<float, 6, 1>& getDtSpatialVelocityTwistTCP() const
    {
        return _dVsp_twist_tcp;
    }

    const Eigen::Matrix<float, 6, 1>& getDtBodyVelocityTwistTCP() const
    {
        return _dVbd_twist_tcp;
    }

    // ============================================================
    // 7.2) TCP velocity/acceleration twists (hybrid) from Jacobian and dq
    // ============================================================
    void computeHybridVelocityTwistTCP();

    const Eigen::Matrix<float, 6, 1>& getHybridVelocityTwistTCP() const;
    Eigen::Vector3f getHybridLinearVelocityTCP() const;
    Eigen::Vector3f getHybridAngularVelocityTCP() const;    
    
    void computeDtHybridVelocityTwistTCP();

    const Eigen::Matrix<float, 6, 1>& getDtHybridVelocityTwistTCP() const;
    Eigen::Vector3f getHybridLinearAccelerationTCP() const;
    Eigen::Vector3f getHybridAngularAccelerationTCP() const;

    // ============================================================
    // 8) Jacobian Time Derivatives (tool frame, spatial & body)
    // ============================================================
    void computeDtSpatialJacobianTCP1();
    Eigen::Matrix<float, 6, Eigen::Dynamic> getDtSpatialJacobianTCP() const;

    // N-DOF equivalent of ScrewsKinematics::DtBodyJacobian_Tool_1()
    // Implements eq.(8)/p.223/[3] (first "=")
    void computeDtBodyJacobianTCP1();
    // N-DOF equivalent of ScrewsKinematics::DtBodyJacobian_Tool_1(),
    // but using the body Jacobian data generated by computeBodyJacobiansFrames2()
    // and stored in _BodyJacobiansFrames[_dof][j].
    void computeDtBodyJacobianTCP2();
    // N-DOF equivalent of ScrewsKinematics::DtBodyJacobian_Tool_2()
    // Implements the second "=" in eq.(8)/p.223/[3]
    void computeDtBodyJacobianTCP3();

    Eigen::Matrix<float, 6, Eigen::Dynamic> getDtBodyJacobianTCP() const;

    void computeDtHybridJacobianTCP();
    Eigen::Matrix<float, 6, Eigen::Dynamic> getDtHybridJacobianTCP() const;

    // ============================================================
    // 9) Operational Jacobian, This is the TCP Hybrid Jacobian, 
    //    exposed with operational-space naming
    // ============================================================
    Eigen::Matrix<float, 6, MAX_DOF> Jop;

    Eigen::Matrix<float, 6, Eigen::Dynamic> getOperationalJacobianTCP() const;

    bool hasOperationalJacobianTCP() const noexcept { return _is_operational_jacobian_valid; }

    Eigen::Matrix<float, 6, MAX_DOF> dJop;

    Eigen::Matrix<float, 6, Eigen::Dynamic> getDtOperationalJacobianTCP() const;

    bool hasDtOperationalJacobianTCP() const noexcept { return _is_dt_operational_jacobian_valid; }  

protected:
    RobotAbstractBaseNdof* _ptr2abstract_ndof {nullptr};
    int _dof {0};

    // Auxiliary arrays
    Eigen::Matrix<float, 6, 6> _ad;  // adjoint(screw product) result

    // --- Ndof metamorphic link data ---
    int _total_pseudojoints {0};
    int _meta_pseudojoints[MAX_METALINKS] {0, 0, 0}; // per meta-link

    // Joint State    
    float _joint_pos[MAX_DOF];
    float _joint_vel[MAX_DOF];
    float _joint_accel[MAX_DOF];

    // Exponentials - Ndof equivalents of the 3-DOF private members
    Eigen::Isometry3f _active_expos[MAX_DOF];       // exp(ξ_ref_i * q_i)
    Eigen::Isometry3f _active_expos_anat[MAX_DOF];  // exp(ξ_anat_i * q_i)
    Eigen::Isometry3f _Pi[MAX_METALINKS];  // anatomy transforms per meta-link

    // Transformations
    Eigen::Isometry3f _gst;                         // TCP pose
    Eigen::Isometry3f _g[MAX_DOF + 1];              // joint frames + TCP
    Eigen::Isometry3f _gl[MAX_DOF];                 // link COM frames  

    // Body Jacobians w.r.t. each joint frame:
    // index k = 0.._dof  → frame {k} (joint 0.._dof-1, TCP at index _dof)
    // index i = 0.._dof-1 → column for joint i
    Eigen::Matrix<float, 6, 1> _BodyJacobiansFrames[MAX_DOF+1][MAX_DOF];
    Eigen::Matrix<float, 6, 1>* _ptr2BodyJacobiansFrames[MAX_DOF+1][MAX_DOF];
    
    // Body Jacobians w.r.t. each link COM frame:
    Eigen::Matrix<float, 6, 1> _BodyCOMJacobiansFrames[MAX_DOF][MAX_DOF];
    Eigen::Matrix<float, 6, 1>* _ptr2BodyCOMJacobiansFrames[MAX_DOF][MAX_DOF]; 

    // ------------------------------------------------------------
    // Mueller body-form kinematic blocks used by Coriolis matrix
    // ------------------------------------------------------------

    // Strict lower-triangular block matrix A^b with dimensions (6*dof) x (6*dof)
    // Block (i,j), 0-based:
    //   A^b_ij = Ad_{g_i^{-1} g_j}   for j < i
    //   A^b_ij = 0                   otherwise
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> computeAbMatrix() const;

    // Block diagonal matrix a^b = diag( qdot_i * ad(X_i^b) )
    // Dimensions: (6*dof) x (6*dof)
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> computeabMatrix() const;

    // Block diagonal matrix b^b = diag( ad(V_i^b) )
    // Dimensions: (6*dof) x (6*dof)
    // Uses current body twists of the real joint/body frames.
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> computebbMatrix() const;

    // Convenience: stacked body Jacobian J^b with one 6xdof block per body frame
    // Dimensions: (6*dof) x dof
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> stackBodyJacobiansFrames() const;

    // Convenience: stacked body twists V^b = J^b * qdot
    // Dimensions: (6*dof) x 1
    Eigen::Matrix<float, Eigen::Dynamic, 1> stackBodyTwistsFrames() const;

private:
    // Auxiliary arrays
    Eigen::Matrix<float, 6, 6> _scp; // spatial cross profuct result

    Eigen::Isometry3f _last_expo;          // running product within meta-link
    int _last_twist_cnt {0};               // global index into passive twists

    Eigen::Isometry3f _g0[MAX_DOF + 1];             // joint frames + TCP @ zero configuration, Ai tfs of Mueller
    Eigen::Isometry3f _Bi[MAX_DOF + 1];             // relative frames C_i,i-1(0)
    Eigen::Isometry3f _gl0[MAX_DOF];                // link COM frames @ zero configuration

    // R6 twists
    Eigen::Matrix<float, 6, 1> _iXi[MAX_DOF + 1];   // local screw coords
    Eigen::Matrix<float, 6, 1> _Yi[MAX_DOF];        // spatial joint screw coords

    // SE(3) twists
    Eigen::Matrix4f _X_se3;

    // Jacobians (tool)
    Eigen::Matrix<float, 6, MAX_DOF> _Jsp_tool;     // spatial TCP Jacobian (cols 0.._dof-1)
    Eigen::Matrix<float, 6, MAX_DOF> _Jbd_tool;     // body TCP Jacobian    (cols 0.._dof-1)

    // Time Derivatives of Jacobians (tool)
    Eigen::Matrix<float, 6, MAX_DOF> _dJsp_tool;     
    Eigen::Matrix<float, 6, MAX_DOF> _dJbd_tool;
    Eigen::Matrix<float, 6, MAX_DOF> _dJh_tcp;     // time derivative of hybrid Jacobian of TCP

    // Hybrid Jacobian of TCP (linear velocity in base frame, angular velocity in tcp/body convention)
    Eigen::Matrix<float, 6, MAX_DOF> _Jh_tcp;

    // Operational Jacobian boolean for checking expose of private member
    bool _is_operational_jacobian_valid {false};
    bool _is_dt_operational_jacobian_valid{false};
    
    // Transform from last actual joint frame to tcp frame
    Eigen::Isometry3f _g_last_tcp;

    // TCP velocity twists
    Eigen::Matrix<float, 6, 1> _Vsp_twist_tcp;  // spatial twist of TCP
    Eigen::Matrix<float, 6, 1> _Vbd_twist_tcp;  // body twist of TCP
    Eigen::Matrix<float, 6, 1> _Vh_twist_tcp;   // hybrid velocity twist of TCP [v; w]

    // Fist time derivative of TCP velocity twists
    Eigen::Matrix<float, 6, 1> _dVsp_twist_tcp;   // spatial acceleration twist of TCP
    Eigen::Matrix<float, 6, 1> _dVbd_twist_tcp;   // body acceleration twist of TCP
    Eigen::Matrix<float, 6, 1> _dVh_twist_tcp;   // hybrid acceleration twist of TCP [a; alpha] 
       
    bool _debug_verbosity {true};

    void printTwist(Eigen::Matrix<float, 6, 1> twist);

};

#endif // SCREWS_KINEMATICS_NDOF_H

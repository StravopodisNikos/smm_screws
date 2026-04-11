#include "smm_screws/core/ScrewsDynamicsNdof.h"

ScrewsDynamicsNdof::ScrewsDynamicsNdof()
: ScrewsKinematicsNdof(nullptr)
{
    _debug_verbosity = true;

    MM.setZero();
    CM.setZero();
    GV.setZero();
    FV.setZero();

    _alpha_temp.setZero();
    _Ml_temp.setZero();
    _alpha_transpose.setZero();
    _parDer_MassIJ_ThetaK.setZero();
    _xi_traspose.setZero();

    _PotEnergy_prev = 0.0f;
    _PotEnergy      = 0.0f;

    _last_expo = Eigen::Isometry3f::Identity();
    _last_twist_cnt = 0;

    for (int i = 0; i < MAX_DOF; ++i) {
        _joint_pos[i]       = 0.0f;
        _joint_vel[i]       = 0.0f;
        _joint_accel[i]     = 0.0f;
        _joint_pos_prev[i]  = 0.0f;
        _delta_joint_pos[i] = 0.0f;

        _gsli[i] = Eigen::Isometry3f::Identity();
        ptr2links_com_tfs[i] = &_gsli[i];

        _Mib[i].setIdentity();
        _Mis[i].setIdentity();

        parDerMass.row(i).setZero();
        ChristoffelSymbols[i].setZero();

        for (int j = 0; j < MAX_DOF; ++j) {
            _Jgl[i][j].setZero();
            ptr2Jgl[i][j] = &_Jgl[i][j];
        }
    }

    // need to be expanded for N dof case! must check theory!
    for (int i = 0; i < 2; ++i) {
        _alpha[i].setZero();
    }

    for (int i = 0; i < 5; ++i) {
        _alphaParDer[i].setZero();
    }

    for (int i = 0; i < 2; ++i) {
        _LieBracketParDer[i].setZero();
    }

    ptr2MM = nullptr;
}

ScrewsDynamicsNdof::ScrewsDynamicsNdof(RobotAbstractBaseNdof* ptr2abstract_ndof)
: ScrewsKinematicsNdof(ptr2abstract_ndof)
{
    if (!_ptr2abstract_ndof) {
        throw std::invalid_argument(
            "[ScrewsDynamicsNdof] RobotAbstractBaseNdof pointer is null");
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        throw std::out_of_range(
            "[ScrewsDynamicsNdof] Invalid DOF loaded from RobotAbstractBaseNdof");
    }

    _debug_verbosity = true;

    MM.setZero();
    CM.setZero();
    GV.setZero();
    FV.setZero();

    _alpha_temp.setZero();
    _Ml_temp.setZero();
    _alpha_transpose.setZero();
    _parDer_MassIJ_ThetaK.setZero();
    _xi_traspose.setZero();

    _PotEnergy_prev = 0.0f;
    _PotEnergy      = 0.0f;

    _last_expo = Eigen::Isometry3f::Identity();
    _last_twist_cnt = 0;

    for (int i = 0; i < MAX_DOF; ++i) {
        _joint_pos[i]       = 0.0f;
        _joint_vel[i]       = 0.0f;
        _joint_accel[i]     = 0.0f;
        _joint_pos_prev[i]  = 0.0f;
        _delta_joint_pos[i] = 0.0f;

        _gsli[i] = Eigen::Isometry3f::Identity();
        ptr2links_com_tfs[i] = &_gsli[i];

        _Mib[i].setIdentity();
        _Mis[i].setIdentity();

        parDerMass.row(i).setZero();
        ChristoffelSymbols[i].setZero();

        for (int j = 0; j < MAX_DOF; ++j) {
            _Jgl[i][j].setZero();
            ptr2Jgl[i][j] = &_Jgl[i][j];
        }
    }

    for (int i = 0; i < 2; ++i) {
        _alpha[i].setZero();
    }

    for (int i = 0; i < 5; ++i) {
        _alphaParDer[i].setZero();
    }

    for (int i = 0; i < 2; ++i) {
        _LieBracketParDer[i].setZero();
    }

    ptr2MM = nullptr;
}

void ScrewsDynamicsNdof::initializeLinkMassMatrices()
{
    // =========================================================================
    // N-DOF ROS2 upgrade of the old 3-DOF function:
    //
    //   ScrewsDynamics::initalizeLinkMassMatrices()
    //
    // New N-DOF behavior:
    //   Copies the spatial link inertia matrices from the N-DOF abstract base:
    //
    //     RobotAbstractBaseNdof::Mi_s[i]
    //     RobotAbstractBaseNdof::Mi_s_ptr[i]
    //
    //   which are already loaded from YAML during:
    //     RobotAbstractBaseNdof::initializeFromYaml(...)
    //
    // Source in RobotAbstractBaseNdof:
    //   yaml_loader.M_s_com_0[i] -> Mi_s[i] -> Mi_s_ptr[i]
    //
    // Meaning of _Mis:
    //   Internal storage of link spatial inertia / mass matrices expressed in
    //   the spatial/base frame {S}.
    //
    // Compatibility / architecture notes:
    //   - This function is fully aligned with the ROS2 N-DOF architecture.
    //   - Storage is fixed to MAX_DOF, but only the first _dof entries are
    //     meaningful for the current robot.
    //   - Remaining slots [_dof .. MAX_DOF-1] are explicitly zeroed for safety.
    //
    // Preconditions:
    //   1) _ptr2abstract_ndof is valid
    //   2) _dof has already been loaded from RobotAbstractBaseNdof
    //   3) RobotAbstractBaseNdof::initializeFromYaml(...) has already succeeded
    //   4) Mi_s_ptr[i] entries of the abstract base are valid for i < _dof
    //
    // Postcondition:
    //   _Mis[i] contains the spatial inertia of link i, for i = 0.._dof-1
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsDynamicsNdof::initializeLinkMassMatrices] "
                  << "RobotAbstractBaseNdof pointer is null.\n";
        return;
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        std::cerr << "[ScrewsDynamicsNdof::initializeLinkMassMatrices] "
                  << "Invalid DOF = " << _dof
                  << " (expected 1.." << MAX_DOF << ").\n";
        return;
    }

    // Copy valid spatial inertia matrices from abstract base to local storage
    for (int i = 0; i < _dof; ++i) {
        if (!_ptr2abstract_ndof->Mi_s_ptr[i]) {
            std::cerr << "[ScrewsDynamicsNdof::initializeLinkMassMatrices] "
                      << "Mi_s_ptr[" << i << "] is null.\n";
            _Mis[i].setZero();
            continue;
        }

        _Mis[i] = *(_ptr2abstract_ndof->Mi_s_ptr[i]);

        if (_debug_verbosity) {
            std::cout << "[ScrewsDynamicsNdof::initializeLinkMassMatrices] "
                      << "_Mis[" << i << "] =\n"
                      << _Mis[i] << std::endl;
        }
    }

    // Zero unused entries for safety / deterministic behavior
    for (int i = _dof; i < MAX_DOF; ++i) {
        _Mis[i].setZero();
    }
}

void ScrewsDynamicsNdof::updateCOMTfs()
{
    // =========================================================================
    // N-DOF ROS2 function:
    //   ScrewsDynamicsNdof::updateCOMTfs()
    //
    // Purpose
    // -------
    // Updates the current link center-of-mass transforms using the current joint
    // positions and the already available kinematics exponentials:
    //
    //   _active_expos_anat[i] = exp(ξ_anat_i * q_i)
    //
    // The CoM transform of link i is computed as:
    //
    //   g_sl_i(q) = (exp_1 * exp_2 * ... * exp_i) * g_sl_i(0)
    //
    // where:
    //   - exp_k = current active joint exponential for joint k
    //   - g_sl_i(0) = home CoM frame of link i, loaded from YAML in the
    //                 RobotAbstractBaseNdof storage:
    //                     gl_test_0[i]
    //                 or equivalently via:
    //                     gsli_test_ptr[i]
    //
    // Compatibility / architecture
    // ----------------------------
    // This function is fully aligned with the ROS2 N-DOF architecture:
    //   - runtime DOF via _dof
    //   - inherited kinematics exponentials from ScrewsKinematicsNdof
    //   - CoM home frames loaded by RobotAbstractBaseNdof from YAML
    //
    // It also preserves the old dynamics-side compatibility storage:
    //   gsli[i]
    //   ptr2links_com_tfs[i]
    //
    // Preconditions
    // -------------
    // 1) _ptr2abstract_ndof is valid
    // 2) _dof is valid
    // 3) joint positions _joint_pos[] are already updated
    // 4) _active_expos_anat[] has already been updated
    //    (typically by updateActiveExpos())
    // 5) the abstract base has already loaded valid home CoM frames:
    //      gsli_test_ptr[i] or gl_test_0[i]
    //
    // Postcondition
    // -------------
    // For i = 0 .. _dof-1:
    //   gsli[i] = current world/base -> link_i_CoM transform
    //
    // Unused slots [ _dof .. MAX_DOF-1 ] are reset to identity for safety.
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsDynamicsNdof::updateCOMTfs] "
                  << "RobotAbstractBaseNdof pointer is null.\n";
        return;
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        std::cerr << "[ScrewsDynamicsNdof::updateCOMTfs] "
                  << "Invalid DOF = " << _dof
                  << " (expected 1.." << MAX_DOF << ").\n";
        return;
    }

    // Running forward product of active exponentials
    Eigen::Isometry3f prefix = Eigen::Isometry3f::Identity();

    for (int i = 0; i < _dof; ++i) {
        // Update forward product up to joint i
        prefix = prefix * _active_expos_anat[i];

        // Validate home CoM frame pointer from abstract base
        if (!_ptr2abstract_ndof->gsli_test_ptr[i]) {
            std::cerr << "[ScrewsDynamicsNdof::updateCOMTfs] "
                      << "gsli_test_ptr[" << i << "] is null.\n";
            _gsli[i] = Eigen::Isometry3f::Identity();
            ptr2links_com_tfs[i] = &_gsli[i];
            continue;
        }

        // Current CoM frame of link i:
        // world/base -> current link-i-CoM
        _gsli[i] = prefix * (*_ptr2abstract_ndof->gsli_test_ptr[i]);
        ptr2links_com_tfs[i] = &_gsli[i];

        if (_debug_verbosity) {
            std::cout << "[ScrewsDynamicsNdof::updateCOMTfs] "
                      << "gsli[" << i << "] =\n"
                      << _gsli[i].matrix() << std::endl;
        }
    }

    // Reset unused entries for deterministic behavior
    for (int i = _dof; i < MAX_DOF; ++i) {
        _gsli[i] = Eigen::Isometry3f::Identity();
        ptr2links_com_tfs[i] = &_gsli[i];
    }
}

Eigen::Matrix<float, 6, 6> ScrewsDynamicsNdof::computeAlphaMatrixAnat(size_t i, size_t j)
{
    // =========================================================================
    // N-DOF anatomy/current version of the Alpha matrix
    //
    // Theory:
    //   Implements Müller Eq. (4.27), using twist ordering [v; w].
    //
    //   For i > j:
    //     A_ij = Ad^{-1}( exp(xi_{j+1} * q_{j+1}) * ... * exp(xi_i * q_i) )
    //
    //   For i == j:
    //     A_ij = I
    //
    //   For i < j:
    //     A_ij = 0
    //
    // In this anatomy/current version:
    //   - we use the already updated current anatomy exponentials
    //       _active_expos_anat[k]
    //   - no passive transforms are needed
    //     because the anatomy twists already encode the current structure
    //
    // Compatibility / architecture:
    //   - fully aligned with ScrewsKinematicsNdof protected members
    //   - runtime DOF via _dof
    //   - valid for N-DOF (currently 3..6 due to xacro generation limits)
    //
    // Preconditions:
    //   1) _dof is valid
    //   2) _active_expos_anat[] has already been updated
    //      (typically by updateActiveExpos())
    //
    // Notes:
    //   This is the clean N-DOF extension of the old 3DOF hardcoded Alpha logic
    //   when the structure is absorbed in the anatomy screw coordinates.
    // =========================================================================

    if (_dof <= 0 || _dof > MAX_DOF) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::computeAlphaMatrixAnat] Invalid DOF.");
    }

    if (i >= static_cast<size_t>(_dof) || j >= static_cast<size_t>(_dof)) {
        throw std::out_of_range(
            "[ScrewsDynamicsNdof::computeAlphaMatrixAnat] Index out of range.");
    }

    if (i < j) {
        _alpha_temp.setZero();
        return _alpha_temp;
    }

    if (i == j) {
        _alpha_temp.setIdentity();
        return _alpha_temp;
    }

    // Build the transform chain:
    //   g = exp_{j+1} * exp_{j+2} * ... * exp_i
    Eigen::Isometry3f g_chain = Eigen::Isometry3f::Identity();

    for (size_t k = j + 1; k <= i; ++k) {
        g_chain = g_chain * _active_expos_anat[k];
    }

    // A_ij = Ad^{-1}(g_chain)
    iad(_alpha_temp, g_chain);
    return _alpha_temp;
}

Eigen::Matrix<float, 1, 1>
ScrewsDynamicsNdof::computeParDerMassElement(size_t i, size_t j, size_t k)
{
    // =========================================================================
    // N-DOF ROS2 upgrade of the old 3-DOF function:
    //
    //   ScrewsDynamics::computeParDerMassElement(size_t i, size_t j, size_t k)
    //
    // Purpose
    // -------
    // Computes one scalar element of the partial derivative of the mass matrix:
    //
    //   dM(i,j) / dtheta_k
    //
    // using the screw-theoretic formulation already validated in the old 3DOF
    // code, but generalized to runtime N-DOF.
    //
    // Theory / structure
    // ------------------
    // The implementation follows the same algebraic structure as the original
    // code:
    //
    //   sum_{l=max(i,j)}^{n-1} [ term_1(l,i,j,k) + term_2(l,i,j,k) ]
    //
    // where the terms are built from:
    //   - Alpha matrices A_ab
    //   - Lie brackets of transformed active anatomy twists
    //   - spatial link inertia matrices _Mis[l]
    //
    // In this upgraded version:
    //   - only the anatomy/current Alpha matrix is used:
    //       computeAlphaMatrixAnat(...)
    //   - only active_twists_anat[] are used
    //   - no passive transforms are required explicitly
    //
    // Compatibility
    // -------------
    // This function is compatible with:
    //   - ScrewsKinematicsNdof protected joint / twist / transform state
    //   - ScrewsMain::lb(...)
    //   - RobotAbstractBaseNdof::active_twists_anat[]
    //   - RobotAbstractBaseNdof::Mi_s_ptr[] loaded through YAML
    //
    // Preconditions
    // -------------
    // 1) _ptr2abstract_ndof is valid
    // 2) _dof is valid
    // 3) initializeLinkMassMatrices() has already been called
    // 4) updateActiveExpos() has already been called
    // 5) indices i, j, k are all < _dof
    //
    // Output
    // ------
    // Returns a 1x1 matrix, to preserve compatibility with the original code.
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::computeParDerMassElement] "
            "RobotAbstractBaseNdof pointer is null.");
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::computeParDerMassElement] Invalid DOF.");
    }

    if (i >= static_cast<size_t>(_dof) ||
        j >= static_cast<size_t>(_dof) ||
        k >= static_cast<size_t>(_dof))
    {
        throw std::out_of_range(
            "[ScrewsDynamicsNdof::computeParDerMassElement] Index out of range.");
    }

    _parDer_MassIJ_ThetaK.setZero();

    // l starts at max(i,j), exactly as in the old 3DOF formulation
    const size_t max_ij = (i > j) ? i : j;

    for (size_t l = max_ij; l < static_cast<size_t>(_dof); ++l)
    {
        // ---------------------------------------------------------------------
        // Alpha matrices (anatomy/current version)
        // ---------------------------------------------------------------------
        _alphaParDer[0] = computeAlphaMatrixAnat(k, i); // A_ki
        _alphaParDer[1] = computeAlphaMatrixAnat(l, k); // A_lk
        _alphaParDer[2] = computeAlphaMatrixAnat(l, j); // A_lj

        _alphaParDer[3] = computeAlphaMatrixAnat(l, i); // A_li
        _alphaParDer[4] = computeAlphaMatrixAnat(k, j); // A_kj

        // ---------------------------------------------------------------------
        // Lie brackets
        // ---------------------------------------------------------------------
        _LieBracketParDer[0] =
            lb(_alphaParDer[0] * _ptr2abstract_ndof->active_twists_anat[i],
               _ptr2abstract_ndof->active_twists_anat[k]);

        _LieBracketParDer[1] =
            lb(_alphaParDer[4] * _ptr2abstract_ndof->active_twists_anat[j],
               _ptr2abstract_ndof->active_twists_anat[k]);

        // ---------------------------------------------------------------------
        // Link spatial inertia matrix
        // In the old code:
        //   _Ml_temp = _Mis[l];
        // which means spatial/base-frame inertia is already available
        // ---------------------------------------------------------------------
        _Ml_temp = _Mis[l];

        // ---------------------------------------------------------------------
        // Accumulate scalar contribution
        // ---------------------------------------------------------------------
        _parDer_MassIJ_ThetaK =
            _parDer_MassIJ_ThetaK
            + (
                _LieBracketParDer[0].transpose()
                * _alphaParDer[1].transpose()
                * _Ml_temp
                * _alphaParDer[2]
                * _ptr2abstract_ndof->active_twists_anat[j]
              )
            + (
                _ptr2abstract_ndof->active_twists_anat[i].transpose()
                * _alphaParDer[3].transpose()
                * _Ml_temp
                * _alphaParDer[1]
                * _LieBracketParDer[1]
              );

        if (_debug_verbosity) {
            std::cout << "[ScrewsDynamicsNdof::computeParDerMassElement] "
                      << "l=" << l
                      << ", partial sum = " << _parDer_MassIJ_ThetaK(0,0)
                      << std::endl;
        }
    }

    return _parDer_MassIJ_ThetaK;
}

void ScrewsDynamicsNdof::computeLinkGeometricJacobians()
{
    // =========================================================================
    // N-DOF ROS2 upgrade of the old 3-DOF function:
    //
    //   ScrewsDynamics::LinkGeometricJacobians()
    //
    // Purpose
    // -------
    // Computes the full 6xN geometric Jacobian for the CoM frame of each link.
    //
    // For each link l and joint j:
    //
    //   if j <= l:
    //
    //       Jg_l(:,j) =
    //       [ a_j x (p_com_l - p_joint_j) ]
    //       [            a_j             ]
    //
    //   if j > l:
    //
    //       Jg_l(:,j) = 0
    //
    // where:
    //   - a_j       : joint axis expressed in base frame
    //   - p_joint_j : current origin of joint j
    //   - p_com_l   : current origin of CoM frame of link l
    //
    // Compatibility / architecture
    // ----------------------------
    // This function is aligned with:
    //   - ScrewsKinematicsNdof inherited joint state and exponentials
    //   - updateActiveTfs()
    //   - updateCOMTfs()
    //   - runtime DOF via _dof
    //   - MAX_DOF internal storage
    //
    // Axis convention
    // ---------------
    // Current SMM convention:
    //   joint 1      -> Z axis
    //   joints 2..3  -> X axis
    //   joints 4..6  -> Y axis
    //
    // Using 0-based indexing:
    //   j == 0       -> column 2
    //   j == 1,2     -> column 0
    //   j >= 3       -> column 1
    //
    // Important update
    // ----------------
    // This upgraded version now stores the full 6x1 geometric Jacobian column
    // for each link/joint pair:
    //
    //   top 3 entries    = translational part
    //   bottom 3 entries = rotational part
    //
    // So each link now has a full 6xN geometric Jacobian.
    //
    // Preconditions
    // -------------
    // 1) _dof is valid
    // 2) current joint positions are already updated
    // 3) updateActiveExpos() has already been called
    // 4) updateActiveTfs() can build current active joint transforms
    // 5) updateCOMTfs() can build current CoM transforms
    //
    // Postcondition
    // -------------
    // For each link l and joint j:
    //   - if j <= l : Jgl[l][j] contains the full 6x1 geometric Jacobian column
    //   - if j > l  : Jgl[l][j] = zero
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsDynamicsNdof::computeLinkGeometricJacobians] "
                  << "RobotAbstractBaseNdof pointer is null.\n";
        return;
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        std::cerr << "[ScrewsDynamicsNdof::computeLinkGeometricJacobians] "
                  << "Invalid DOF = " << _dof
                  << " (expected 1.." << MAX_DOF << ").\n";
        return;
    }

    _debug_verbosity = false;

    // -------------------------------------------------------------------------
    // 1) Update current active joint transforms and current CoM transforms
    // -------------------------------------------------------------------------
    updateCOMTfs();

    // -------------------------------------------------------------------------
    // 2) Reset all Jacobian columns to zero first
    // -------------------------------------------------------------------------
    for (int l = 0; l < MAX_DOF; ++l) {
        for (int j = 0; j < MAX_DOF; ++j) {
            _Jgl[l][j].setZero();
            ptr2Jgl[l][j] = &_Jgl[l][j];
        }
    }

    // -------------------------------------------------------------------------
    // 3) Helper lambda: extract the current joint rotation axis according to
    //    the SMM axis convention
    // -------------------------------------------------------------------------
    auto getJointAxisInBase = [&](int joint_index) -> Eigen::Vector3f
    {
        if (joint_index < 0 || joint_index >= _dof) {
            throw std::out_of_range(
                "[ScrewsDynamicsNdof::computeLinkGeometricJacobians] "
                "Joint index out of range while extracting joint axis.");
        }

        const Eigen::Matrix4f g_mat = _g[joint_index].matrix();

        // 0-based indexing:
        // joint 0      -> Z axis  (column 2)
        // joint 1,2    -> X axis  (column 0)
        // joint >= 3   -> Y axis  (column 1)
        if (joint_index == 0) {
            return g_mat.block<3,1>(0,2);
        } else if (joint_index == 1 || joint_index == 2) {
            return g_mat.block<3,1>(0,0);
        } else {
            return g_mat.block<3,1>(0,1);
        }
    };

    // -------------------------------------------------------------------------
    // 4) Build the full 6xN geometric Jacobians for each link CoM
    // -------------------------------------------------------------------------
    for (int l = 0; l < _dof; ++l)
    {
        const Eigen::Vector3f p_com_l = _gsli[l].translation();

        for (int j = 0; j < _dof; ++j)
        {
            if (j > l) {
                _Jgl[l][j].setZero();
                ptr2Jgl[l][j] = &_Jgl[l][j];
                continue;
            }

            const Eigen::Vector3f axis_j = getJointAxisInBase(j);
            const Eigen::Vector3f p_joint_j = _g[j].translation();

            const Eigen::Vector3f Jp_jl = axis_j.cross(p_com_l - p_joint_j);
            const Eigen::Vector3f Jo_jl = axis_j;

            _Jgl[l][j].setZero();
            _Jgl[l][j].block<3,1>(0,0) = Jp_jl;
            _Jgl[l][j].block<3,1>(3,0) = Jo_jl;

            ptr2Jgl[l][j] = &_Jgl[l][j];
        }
    }

    // -------------------------------------------------------------------------
    // 5) Reset unused link rows for deterministic behavior
    // -------------------------------------------------------------------------
    for (int l = _dof; l < MAX_DOF; ++l) {
        for (int j = 0; j < MAX_DOF; ++j) {
            _Jgl[l][j].setZero();
            ptr2Jgl[l][j] = &_Jgl[l][j];
        }
    }

    if (_debug_verbosity) {
        for (int l = 0; l < _dof; ++l) {
            std::cout << "[ScrewsDynamicsNdof::computeLinkGeometricJacobians] "
                      << "Full 6xN geometric Jacobian columns for link " << l << ":\n";
            for (int j = 0; j < _dof; ++j) {
                std::cout << "  Jgl[" << l << "][" << j << "] = "
                          << _Jgl[l][j].transpose() << std::endl;
            }
        }
    }
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsDynamicsNdof::MassMatrix()
{
    // =========================================================================
    // N-DOF ROS2 upgrade of the old 3-DOF function:
    //
    //   ScrewsDynamics::MassMatrix()
    //
    // Purpose
    // -------
    // Computes the joint-space mass matrix M(q) of the current SMM anatomy,
    // using the same screw-theoretic summation structure validated in the
    // old 3DOF code, but generalized to runtime N-DOF.
    //
    // Mathematical structure
    // ----------------------
    // For each (i,j), the mass matrix element is computed as:
    //
    //   M(i,j) = sum_{l=max(i,j)}^{n-1}
    //            xi_i^T * A_li^T * M_l * A_lj * xi_j
    //
    // where:
    //   - xi_i, xi_j : active anatomy twists
    //   - A_li, A_lj : Alpha matrices (anatomy/current version)
    //   - M_l        : spatial link inertia matrix of link l in frame {S}
    //
    // This preserves the exact structure of the old validated 3DOF code, with:
    //   setAlphamatrix(...)        -> computeAlphaMatrixAnat(...)
    //   _ptr2abstract->...         -> _ptr2abstract_ndof->...
    //   robot_params::DOF          -> runtime _dof
    //
    // Compatibility / architecture
    // ----------------------------
    // This implementation is fully aligned with:
    //   - ScrewsKinematicsNdof inherited joint state
    //   - RobotAbstractBaseNdof YAML-loaded active anatomy twists
    //   - N-DOF spatial link inertias _Mis[]
    //   - runtime DOF via _dof
    //
    // IMPORTANT DESIGN CHOICE
    // -----------------------
    // This function does NOT:
    //   - recompute active exponentials
    //   - reinitialize link mass matrices
    //
    // It assumes:
    //   - _active_expos_anat[] is already up to date for the current q
    //   - _Mis[] has already been initialized once
    //
    // This avoids repeated work and keeps dynamics aligned with the inherited
    // kinematics state.
    //
    // Preconditions
    // -------------
    // 1) _ptr2abstract_ndof is valid
    // 2) _dof is valid
    // 3) current joint positions _joint_pos[] are already updated
    // 4) inherited kinematics exponentials _active_expos_anat[] are already valid
    // 5) _Mis[] has already been initialized
    //
    // Output
    // ------
    // Returns the top-left _dof x _dof block of the internal fixed-capacity
    // matrix MM.
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::MassMatrix] RobotAbstractBaseNdof pointer is null.");
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::MassMatrix] Invalid DOF.");
    }

    _debug_verbosity = false;

    MM.setZero();

    if (_debug_verbosity) {
        std::cout << "[ScrewsDynamicsNdof::MassMatrix] Computing M(q):\n";
    }

    for (size_t i = 0; i < static_cast<size_t>(_dof); ++i)
    {
        for (size_t j = 0; j < static_cast<size_t>(_dof); ++j)
        {
            const size_t max_ij = (i > j) ? i : j;

            for (size_t l = max_ij; l < static_cast<size_t>(_dof); ++l)
            {
                // Alpha matrices (anatomy/current version)
                _alpha[0] = computeAlphaMatrixAnat(l, i); // A_li
                _alpha[1] = computeAlphaMatrixAnat(l, j); // A_lj

                // Spatial link inertia of link l
                // Already expressed in frame {S}, so no extra adjoint transform
                _Ml_temp = _Mis[l];

                MM(i, j) +=
                    _ptr2abstract_ndof->active_twists_anat[i].transpose()
                    * _alpha[0].transpose()
                    * _Ml_temp
                    * _alpha[1]
                    * _ptr2abstract_ndof->active_twists_anat[j];
            }

            if (_debug_verbosity) {
                std::cout << MM(i, j) << "\t";
            }
        }

        if (_debug_verbosity) {
            std::cout << std::endl;
        }
    }

    ptr2MM = &MM;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> M_out(_dof, _dof);
    M_out = MM.block(0, 0, _dof, _dof);

    return M_out;
}

/*
 *  PRINTING FUNCTIONS-USED FOR DEBUGGING
 */

void ScrewsDynamicsNdof::print66Matrix(Eigen::Matrix<float, 6, 6> matrix) {
    for (size_t i = 0; i < 6; i++) {
        for (size_t j = 0; j < 6; j++) {
            std::cout << matrix(i, j) << "\t";
        }
        std::cout << std::endl;
    }
    return;
}

void ScrewsDynamicsNdof::print61Matrix(Eigen::Matrix<float, 6, 1> matrix) {
    for (size_t i = 0; i < 6; i++) {
        std::cout << matrix[i] << std::endl;
    }
    return;
}

void ScrewsDynamicsNdof::print16Matrix(Eigen::Matrix<float, 1, 6> matrix) {
    for (size_t i = 0; i < 6; i++) {
        std::cout << matrix[i] << "\t";
    }
    std::cout <<  std::endl;
    return;
}
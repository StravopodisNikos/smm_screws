#include "smm_screws/core/ScrewsDynamicsNdof.h"

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

        _BodyInertiaFrames[i].setZero();
        _ptr2BodyInertiaFrames[i] = &_BodyInertiaFrames[i];
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

void ScrewsDynamicsNdof::computeBodyInertiaFromSpatial(BodyFrameSelection body_frame)
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsDynamicsNdof::computeBodyInertiaFromSpatial] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsDynamicsNdof::computeBodyInertiaFromSpatial] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    _debug_verbosity = false;

    // ============================================================
    // Inverse of Mueller Eq. (35):
    //
    //   M_i^s = Ad_{C_i}^{-T} M_i^b Ad_{C_i}^{-1}
    //
    // therefore
    //
    //   M_i^b = Ad_{C_i}^{T} M_i^s Ad_{C_i}
    //
    // body_frame == JOINT:
    //   use current joint/body frame _g[i]
    //
    // body_frame == COM:
    //   use current COM frame _gl[i]
    //
    // Spatial link inertias are assumed stored in _Mis[i].
    // ============================================================
    
    for (int i = 0; i < _dof; ++i) {
        std::cout << "[DEBUG] _ptr2BodyInertiaFrames[" << i << "] = "
                << _ptr2BodyInertiaFrames[i] << "\n";
    }

    for (int i = 0; i < _dof; ++i) {

        const Eigen::Isometry3f& C_i =
            (body_frame == BodyFrameSelection::JOINT) ? _g[i] : _gl[i];

        const Eigen::Matrix<float, 6, 6>& M_s_i = _Mis[i];

        ad(_ad, C_i);  // _ad = Ad_{C_i} in your [v; w] convention

        Eigen::Matrix<float, 6, 6> M_b_i = _ad.transpose() * M_s_i * _ad;

        _BodyInertiaFrames[i] = M_b_i;

        if (_ptr2BodyInertiaFrames[i]) {
            *(_ptr2BodyInertiaFrames[i]) = M_b_i;
        }

        if (_debug_verbosity) {
            std::cout << "[ScrewsDynamicsNdof::computeBodyInertiaFromSpatial] "
                      << "link " << i
                      << " body_frame = "
                      << ((body_frame == BodyFrameSelection::JOINT) ? "JOINT" : "COM")
                      << "\nM_b[" << i << "] =\n"
                      << M_b_i << "\n";
        }
    }
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
ScrewsDynamicsNdof::MassMatrix(
    DynamicsRepresentation representation,
    BodyFrameSelection body_frame)
{
    // =====================================================================
    // Dispatcher for Mass Matrix computation
    //
    // Routes the request to the appropriate internal implementation based on:
    //   - representation (SPATIAL / BODY)
    //   - body frame selection (JOINT / COM)
    //
    // This is the ONLY public entry point for users.
    // =====================================================================

    switch (representation)
    {
        case DynamicsRepresentation::SPATIAL:
        {
            // Body frame selection is irrelevant here
            return MassMatrix_s(body_frame);
        }

        case DynamicsRepresentation::BODY:
        {
            return MassMatrix_b(body_frame);
        }

        default:
        {
            throw std::runtime_error(
                "[ScrewsDynamicsNdof::MassMatrix] Invalid representation type.");
        }
    }
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsDynamicsNdof::MassMatrix_s(BodyFrameSelection /*body_frame*/)
{
    // =========================================================================
    // Spatial-formulation Mass Matrix M(q)
    // =========================================================================
    //
    // This function computes the joint-space mass matrix using the
    // **spatial (base-frame) representation** of screw theory.
    //
    // THEORY (Mueller / Murray consistent)
    // -----------------------------------
    // Each element is computed as:
    //
    //   M(i,j) = sum_{l=max(i,j)}^{n-1}
    //            xi_i^T * A_li^T * M_l * A_lj * xi_j
    //
    // where:
    //   xi_i, xi_j : active joint twists (expressed in spatial frame)
    //   A_li       : Alpha matrix mapping joint i → link l
    //   M_l        : spatial inertia matrix of link l (expressed in {S})
    //
    // IMPORTANT
    // ---------
    // - This implementation uses **spatial inertias (_Mis)** directly.
    // - No adjoint transformation is required for inertia.
    // - Alpha matrices are computed using:
    //       computeAlphaMatrixAnat(...)
    //
    // DESIGN ASSUMPTIONS
    // ------------------
    // This function assumes:
    //
    //   1) Joint state (_joint_pos, _joint_vel) is already updated
    //   2) Kinematics has already computed:
    //        _active_expos_anat[]
    //   3) Link inertias _Mis[] are initialized once (NOT here)
    //
    // This avoids recomputation and keeps dynamics aligned with
    // ScrewsKinematicsNdof.
    //
    // OUTPUT
    // ------
    // Returns an (_dof x _dof) dynamic matrix containing the valid
    // mass matrix entries.
    //
    // INTERNAL STORAGE
    // ----------------
    // MM is stored as fixed-size (MAX_DOF x MAX_DOF).
    // Only the top-left (_dof x _dof) block is valid.
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::MassMatrix_s] RobotAbstractBaseNdof pointer is null.");
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::MassMatrix_s] Invalid DOF.");
    }

    _debug_verbosity = true;

    MM.setZero();

    if (_debug_verbosity) {
        std::cout << "[MassMatrix_s] Computing spatial mass matrix M(q)\n";
    }

    for (size_t i = 0; i < static_cast<size_t>(_dof); ++i)
    {
        for (size_t j = 0; j < static_cast<size_t>(_dof); ++j)
        {
            const size_t max_ij = (i > j) ? i : j;

            for (size_t l = max_ij; l < static_cast<size_t>(_dof); ++l)
            {
                // --- Alpha matrices (anatomy-based)
                const Eigen::Matrix<float, 6, 6>& A_li =
                    (_alpha[0] = computeAlphaMatrixAnat(l, i));

                const Eigen::Matrix<float, 6, 6>& A_lj =
                    (_alpha[1] = computeAlphaMatrixAnat(l, j));

                // --- Spatial inertia of link l
                _Ml_temp = _Mis[l];

                // --- Contribution to M(i,j)
                MM(i, j) +=
                    _ptr2abstract_ndof->active_twists_anat[i].transpose()
                    * A_li.transpose()
                    * _Ml_temp
                    * A_lj
                    * _ptr2abstract_ndof->active_twists_anat[j];
            }

            if (_debug_verbosity) {
                std::cout << MM(i, j) << "\t";
            }
        }

        if (_debug_verbosity) {
            std::cout << "\n";
        }
    }

    ptr2MM = &MM;

    // -------------------------------------------------------------------------
    // Return only the valid submatrix (dynamic-sized)
    // -------------------------------------------------------------------------
    return MM.topLeftCorner(_dof, _dof);
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsDynamicsNdof::MassMatrix_b(BodyFrameSelection body_frame)
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsDynamicsNdof::MassMatrix_b] DOF <= 0\n";
        return Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>();
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsDynamicsNdof::MassMatrix_b] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>();
    }

    _debug_verbosity = true;

    // ============================================================
    // Preconditions
    // ------------------------------------------------------------
    // body_frame == BodyFrameSelection::JOINT:
    //   1) ForwardKinematicsTCP(q) has been called
    //      -> _g[0.._dof-1] valid
    //   2) computeBodyJacobiansFrames2() has been called
    //      -> _BodyJacobiansFrames valid for real links
    //   3) computeBodyInertiaFromSpatial(BodyFrameSelection::JOINT)
    //      can transform _Mis[i] into _BodyInertiaFrames[i]
    //
    // body_frame == BodyFrameSelection::COM:
    //   1) ForwardKinematicsCOM(q) has been called
    //      -> _gl[k] valid
    //   2) computeBodyCOMJacobiansFrames() has been called
    //      -> _BodyCOMJacobiansFrames valid
    //   3) computeBodyInertiaFromSpatial(BodyFrameSelection::COM)
    //      can transform _Mis[i] into _BodyInertiaFrames[i]
    //
    // Implements the same computation as MATLAB
    // calculateMassMatrix_Mueller():
    //
    //   M(q) = sum_{i=0}^{_dof-1} J_i^T M_i^b J_i
    //
    // where J_i is selected according to body_frame.
    //
    // Example usage:
    // ------------------------------------------------------------
    // ForwardKinematicsTCP(q);
    // computeBodyJacobiansFrames2();
    // Eigen::MatrixXf M_joint = MassMatrix_b(BodyFrameSelection::JOINT);
    // OR
    // ForwardKinematicsCOM(q);
    // computeBodyCOMJacobiansFrames();
    // Eigen::MatrixXf M_com = MassMatrix_b(BodyFrameSelection::COM);
    // 
    // Test to do:
    // ------------------------------------------------------------
    // Eigen::MatrixXf M_joint = MassMatrix_b(BodyFrameSelection::Joint);
    // Eigen::MatrixXf M_com   = MassMatrix_b(BodyFrameSelection::Com);
    // std::cout << "||M_joint - M_com|| = " << (M_joint - M_com).norm() << "\n";
    // ============================================================

    computeBodyInertiaFromSpatial(body_frame);

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> M;
    M.resize(_dof, _dof);
    M.setZero();

    Eigen::Matrix<float, 6, Eigen::Dynamic> J_i(6, _dof);
    J_i.setZero();

    for (int i = 0; i < _dof; ++i) {

        J_i.setZero();

        for (int j = 0; j < _dof; ++j) {
            if (body_frame == BodyFrameSelection::JOINT) {
                J_i.col(j) = _BodyJacobiansFrames[i][j];
            } else {
                J_i.col(j) = _BodyCOMJacobiansFrames[i][j];
            }
        }

        const Eigen::Matrix<float, 6, 6>& M_i_b = _BodyInertiaFrames[i];

        M.noalias() += J_i.transpose() * M_i_b * J_i;

        if (_debug_verbosity) {
            std::cout << "[ScrewsDynamicsNdof::MassMatrix_b] Link " << i
                      << " contribution in "
                      << ((body_frame == BodyFrameSelection::JOINT) ? "JOINT" : "COM")
                      << " frame:\n"
                      << (J_i.transpose() * M_i_b * J_i) << "\n";
        }
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsDynamicsNdof::MassMatrix_b] Final M(q):\n"
                  << M << "\n";
    }

    return M;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsDynamicsNdof::CoriolisMatrix(
    DynamicsRepresentation representation)
{
    switch (representation)
    {
        case DynamicsRepresentation::SPATIAL:
        {
            // Fixed internal choice:
            //   SPATIAL Coriolis -> JOINT-frame formulation
            return CoriolisMatrix_s();
        }

        case DynamicsRepresentation::BODY:
        {
            // Fixed internal choice:
            //   BODY Coriolis -> JOINT-frame formulation
            return CoriolisMatrix_b();
        }

        default:
        {
            throw std::runtime_error(
                "[ScrewsDynamicsNdof::CoriolisMatrix] Invalid representation.");
        }
    }
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsDynamicsNdof::CoriolisMatrix_b()
{
    if (_dof <= 0) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_b] DOF <= 0.");
    }

    if (!_ptr2abstract_ndof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_b] RobotAbstractBaseNdof pointer is null.");
    }

    // Fixed formulation choice:
    //   BODY Coriolis is implemented only in JOINT body frames.
    //
    // Preconditions:
    // 1) joint state updated
    // 2) ForwardKinematicsTCP(q) already called
    // 3) computeBodyJacobiansFrames2() already called
    // 4) initializeLinkMassMatrices() already done
    // 5) computeBodyInertiaFromSpatial(JOINT) done here

    computeBodyInertiaFromSpatial(BodyFrameSelection::JOINT);

    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jb = stackBodyJacobiansFrames();
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Ab = computeAbMatrix();
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> ab = computeabMatrix();
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> bb = computebbMatrix();

    if (Jb.rows() != 6 * _dof || Jb.cols() != _dof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_b] Invalid Jb dimensions.");
    }
    if (Ab.rows() != 6 * _dof || Ab.cols() != 6 * _dof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_b] Invalid Ab dimensions.");
    }
    if (ab.rows() != 6 * _dof || ab.cols() != 6 * _dof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_b] Invalid ab dimensions.");
    }
    if (bb.rows() != 6 * _dof || bb.cols() != 6 * _dof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_b] Invalid bb dimensions.");
    }

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Mb(6 * _dof, 6 * _dof);
    Mb.setZero();

    for (int i = 0; i < _dof; ++i) {
        Mb.block<6,6>(6 * i, 6 * i) = _BodyInertiaFrames[i];
    }

    // Müller Eq. (85):
    //   C(q,qdot) = - Jb^T * ( Mb*Ab*ab + bb^T*Mb ) * Jb
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C =
        -Jb.transpose() * (Mb * Ab * ab + bb.transpose() * Mb) * Jb;

    if (C.rows() != _dof || C.cols() != _dof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_b] Output C has invalid dimensions.");
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsDynamicsNdof::CoriolisMatrix_b] Jb =\n" << Jb << "\n";
        std::cout << "[ScrewsDynamicsNdof::CoriolisMatrix_b] Ab =\n" << Ab << "\n";
        std::cout << "[ScrewsDynamicsNdof::CoriolisMatrix_b] ab =\n" << ab << "\n";
        std::cout << "[ScrewsDynamicsNdof::CoriolisMatrix_b] bb =\n" << bb << "\n";
        std::cout << "[ScrewsDynamicsNdof::CoriolisMatrix_b] Mb =\n" << Mb << "\n";
        std::cout << "[ScrewsDynamicsNdof::CoriolisMatrix_b] C =\n" << C << "\n";
    }

    return C;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsDynamicsNdof::CoriolisMatrix_s()
{
    // =========================================================================
    // N-DOF spatial Coriolis matrix C(q, qdot)
    //
    // Purpose
    // -------
    // Computes the Coriolis / centrifugal matrix using the Christoffel-symbol
    // construction based on partial derivatives of the SPATIAL mass matrix.
    //
    // This is the direct N-DOF generalization of the old 3-DOF function:
    //
    //   ScrewsDynamics::CoriolisMatrix_loc()
    //
    // Theory
    // ------
    // Christoffel symbols of the first kind:
    //
    //   c_ijk = 0.5 * ( dM_ij/dq_k + dM_ik/dq_j - dM_kj/dq_i )
    //
    // and the Coriolis matrix entries are:
    //
    //   C_ij = sum_k c_ijk * qdot_k
    //
    // Here:
    //   - M(q) is the SPATIAL joint-space mass matrix
    //   - dM_ab/dq_c is computed through computeParDerMassElement(a,b,c)
    //
    // Fixed formulation scope
    // -----------------------
    // This implementation is always the SPATIAL / JOINT formulation.
    // There is no runtime body-frame selection.
    //
    // Compatibility
    // -------------
    // This function relies on:
    //   - initializeLinkMassMatrices() already executed
    //   - updateJointState(...) already executed
    //   - updateActiveExpos() already consistent with current state
    //   - computeParDerMassElement(...) already upgraded to N-DOF
    //
    // Output
    // ------
    // Returns an (_dof x _dof) dynamic-sized Coriolis matrix.
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_s] "
            "RobotAbstractBaseNdof pointer is null.");
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::CoriolisMatrix_s] Invalid DOF.");
    }

    CM.setZero();

    const bool debug_print = _debug_verbosity;

    for (size_t i = 0; i < static_cast<size_t>(_dof); ++i)
    {
        for (size_t j = 0; j < static_cast<size_t>(_dof); ++j)
        {
            float cij_sum = 0.0f;

            for (size_t k = 0; k < static_cast<size_t>(_dof); ++k)
            {
                const float dM_ij_dq_k =
                    computeParDerMassElement(i, j, k)(0, 0);

                const float dM_ik_dq_j =
                    computeParDerMassElement(i, k, j)(0, 0);

                const float dM_kj_dq_i =
                    computeParDerMassElement(k, j, i)(0, 0);

                const float christoffel =
                    0.5f * (dM_ij_dq_k + dM_ik_dq_j - dM_kj_dq_i);

                ChristoffelSymbols[k](i, j) = christoffel;
                cij_sum += christoffel * _joint_vel[k];
            }

            CM(i, j) = cij_sum;

            if (debug_print) {
                std::cout << CM(i, j) << "\t";
            }
        }

        if (debug_print) {
            std::cout << "\n";
        }
    }

    return CM.topLeftCorner(_dof, _dof);
}

Eigen::Matrix<float, Eigen::Dynamic, 1>
ScrewsDynamicsNdof::GravityVector(
    DynamicsRepresentation representation)
{
    switch (representation)
    {
        case DynamicsRepresentation::SPATIAL:
        {
            // Fixed formulation:
            // spatial gravity is computed through CoM geometric Jacobians.
            return GravityVector_s();
        }

        case DynamicsRepresentation::BODY:
        {
            // Fixed formulation:
            // body gravity is computed through body CoM Jacobians.
            return GravityVector_b();
        }

        default:
        {
            throw std::runtime_error(
                "[ScrewsDynamicsNdof::GravityVector] Invalid representation.");
        }
    }
}

float ScrewsDynamicsNdof::extractLinkMassFromSpatialInertia(size_t link_index) const
{
    if (link_index >= static_cast<size_t>(_dof)) {
        throw std::out_of_range(
            "[ScrewsDynamicsNdof::extractLinkMassFromSpatialInertia] Index out of range.");
    }

    const Eigen::Matrix<float, 6, 6>& M_s_i = _Mis[link_index];

    // Standard spatial inertia structure in [v; w] convention:
    // top-left block = m * I_3
    const float m00 = M_s_i(0,0);
    const float m11 = M_s_i(1,1);
    const float m22 = M_s_i(2,2);

    const float mass = (m00 + m11 + m22) / 3.0f;

    if (mass <= 0.0f) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::extractLinkMassFromSpatialInertia] Non-positive extracted mass.");
    }

    return mass;
}

Eigen::Matrix<float, Eigen::Dynamic, 1>
ScrewsDynamicsNdof::GravityVector_s()
{
    // =========================================================================
    // N-DOF spatial gravity vector g(q)
    //
    // Implements:
    //   g(q) = - sum_i Jv_i(q)^T * (m_i * g0)
    //
    // where:
    //   - Jv_i is the top 3 rows of the full geometric Jacobian of CoM i
    //   - g0 is gravity expressed in the base/spatial frame
    //
    // This matches the MATLAB functions:
    //   calculateGravityVectorAnalytical(...)
    //   calculateGravityVectorAnalyticalSpatialCOM(...)
    //
    // Fixed formulation scope
    // -----------------------
    // This is always the SPATIAL gravity formulation, evaluated through
    // CoM-based geometric Jacobians. There is no runtime body-frame selection.
    //
    // Preconditions:
    //   1) joint state already updated
    //   2) initializeLinkMassMatrices() already called
    //   3) current active transforms / exponentials already available
    //
    // Notes:
    //   - This function internally refreshes the CoM geometric Jacobians.
    //   - Gravity is expressed in the base/spatial frame.
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::GravityVector_s] "
            "RobotAbstractBaseNdof pointer is null.");
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::GravityVector_s] Invalid DOF.");
    }

    GV.setZero();

    const bool debug_print = _debug_verbosity;

    // -------------------------------------------------------------------------
    // Base-frame gravity vector
    // -------------------------------------------------------------------------
    Eigen::Matrix<float, 3, 1> g0;
    g0 << 0.0f, 0.0f, -9.80665f;

    // -------------------------------------------------------------------------
    // Ensure CoM geometric Jacobians are current
    // -------------------------------------------------------------------------
    computeLinkGeometricJacobians();

    for (int i = 0; i < _dof; ++i)
    {
        const float mi = extractLinkMassFromSpatialInertia(static_cast<size_t>(i));

        Eigen::Matrix<float, 3, Eigen::Dynamic> Jv_i(3, _dof);
        Jv_i.setZero();

        for (int j = 0; j < _dof; ++j) {
            Jv_i.col(j) = _Jgl[i][j].template block<3,1>(0,0);
        }

        GV.topRows(_dof).noalias() -= Jv_i.transpose() * (mi * g0);

        if (debug_print) {
            std::cout << "[ScrewsDynamicsNdof::GravityVector_s] link " << i
                      << ", mass = " << mi << "\n";
            std::cout << "Jv_i =\n" << Jv_i << "\n";
            std::cout << "partial GV =\n" << GV.topRows(_dof) << "\n";
        }
    }

    return GV.topRows(_dof);
}

Eigen::Matrix<float, Eigen::Dynamic, 1>
ScrewsDynamicsNdof::GravityVector_b()
{
    // =========================================================================
    // N-DOF body gravity vector g(q)
    //
    // Implements:
    //   g(q) = - sum_i Jb_com_i(q)^T * Wg_i^b
    //
    // where:
    //   Wg_i^b = [ R_i^T * (m_i * g0) ;
    //              0 ]
    //
    // with:
    //   - Jb_com_i : 6xn body Jacobian of CoM frame i
    //   - R_i      : current CoM frame orientation wrt base
    //   - g0       : gravity in base frame
    //
    // This matches the MATLAB function:
    //   calculateGravityVectorAnalyticalBodyCOM(...)
    //
    // Fixed formulation scope
    // -----------------------
    // This is always the BODY gravity formulation, evaluated in CoM body
    // frames. There is no runtime body-frame selection.
    //
    // Preconditions:
    //   1) joint state already updated
    //   2) initializeLinkMassMatrices() already called
    //
    // Notes:
    //   - This function internally refreshes the CoM forward kinematics.
    //   - It also refreshes the body CoM Jacobians before assembling g(q).
    // =========================================================================

    if (!_ptr2abstract_ndof) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::GravityVector_b] "
            "RobotAbstractBaseNdof pointer is null.");
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        throw std::runtime_error(
            "[ScrewsDynamicsNdof::GravityVector_b] Invalid DOF.");
    }

    GV.setZero();

    const bool debug_print = _debug_verbosity;

    Eigen::Matrix<float, 3, 1> g0;
    g0 << 0.0f, 0.0f, -9.80665f;

    // -------------------------------------------------------------------------
    // Ensure current CoM frames and body CoM Jacobians are valid
    // -------------------------------------------------------------------------
    ForwardKinematicsCOM(_joint_pos);
    computeBodyCOMJacobiansFrames();

    for (int i = 0; i < _dof; ++i)
    {
        const float mi = extractLinkMassFromSpatialInertia(static_cast<size_t>(i));

        // Current CoM frame orientation wrt base
        const Eigen::Matrix3f R_i = _gl[i].rotation();

        // Gravity force expressed in CoM body frame
        const Eigen::Matrix<float, 3, 1> f_g_b = R_i.transpose() * (mi * g0);

        // 6D wrench in [force; moment] order
        Eigen::Matrix<float, 6, 1> W_g_b;
        W_g_b.setZero();
        W_g_b.template block<3,1>(0,0) = f_g_b;

        Eigen::Matrix<float, 6, Eigen::Dynamic> Jb_com_i(6, _dof);
        Jb_com_i.setZero();

        for (int j = 0; j < _dof; ++j) {
            Jb_com_i.col(j) = _BodyCOMJacobiansFrames[i][j];
        }

        GV.topRows(_dof).noalias() -= Jb_com_i.transpose() * W_g_b;

        if (debug_print) {
            std::cout << "[ScrewsDynamicsNdof::GravityVector_b] link " << i
                      << ", mass = " << mi << "\n";
            std::cout << "R_i =\n" << R_i << "\n";
            std::cout << "f_g_b =\n" << f_g_b << "\n";
            std::cout << "Jb_com_i =\n" << Jb_com_i << "\n";
            std::cout << "partial GV =\n" << GV.topRows(_dof) << "\n";
        }
    }

    return GV.topRows(_dof);
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
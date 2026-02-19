#include "smm_screws/core/ScrewsKinematicsNdof.h"

#include <string>  // for std::to_string

// ==================== ctor ====================

ScrewsKinematicsNdof::ScrewsKinematicsNdof(RobotAbstractBaseNdof* ptr2abstract_ndof)
: _ptr2abstract_ndof(ptr2abstract_ndof)
{
    if (!_ptr2abstract_ndof) {
        throw std::invalid_argument("[ScrewsKinematicsNdof] ptr2abstract_ndof is null");
    }

    _dof = _ptr2abstract_ndof->get_DOF();

    if (_dof < robot_params::MIN_DOF || _dof > robot_params::MAX_DOF) {
        throw std::runtime_error(
            "[ScrewsKinematicsNdof] Invalid DOF in RobotAbstractBaseNdof: " +
            std::to_string(_dof) +
            " (expected " + std::to_string(robot_params::MIN_DOF) +
            ".." + std::to_string(robot_params::MAX_DOF) + ")"
        );
    }

    // Initialize pseudojoints per meta-link from RobotAbstractBaseNdof
    _meta_pseudojoints[0] = _ptr2abstract_ndof->get_PSEUDOS_METALINK1();
    _meta_pseudojoints[1] = _ptr2abstract_ndof->get_PSEUDOS_METALINK2();
    _meta_pseudojoints[2] = _ptr2abstract_ndof->get_PSEUDOS_METALINK3();

    _total_pseudojoints =
        _meta_pseudojoints[0] +
        _meta_pseudojoints[1] +
        _meta_pseudojoints[2];

    _last_twist_cnt = 0;
    _last_expo.setIdentity();

    for (int i = 0; i < MAX_METALINKS; ++i) {
        _Pi[i].setIdentity();
    }

    // Initialize joint state arrays and exponentials
    for (int i = 0; i < MAX_DOF; ++i) {
        _joint_pos[i]   = 0.0f;
        _joint_vel[i]   = 0.0f;
        _joint_accel[i] = 0.0f;
        _active_expos[i].setIdentity();
        _active_expos_anat[i].setIdentity();
    }

    _gst.setIdentity();
    for (int i = 0; i < MAX_DOF + 1; ++i) {
        _g[i].setIdentity();
        _Bi[i].setIdentity();
        _iXi[i].setZero();
    }
}

void ScrewsKinematicsNdof::initializePseudoTfs()
{
    std::cerr
        << "[ScrewsKinematicsNdof::initializePseudoTfs] "
        << "DEPRECATED ANATOMY TFS API - FOR BACKWARD COMPATIBILITY.\n";

    _last_twist_cnt = 0;

    // No pseudojoints at all → all Pi = Identity
    if (_total_pseudojoints == 0) {
        for (int i = 0; i < MAX_METALINKS; ++i) {
            _Pi[i].setIdentity();
            if (_debug_verbosity) {
                std::cout << "[initializePseudoTfsNdof] _Pi[" << i
                          << "] = Identity (no pseudojoints)\n";
            }
        }
        return;
    }

    // Build Pi[meta] for each of the 3 meta-links
    for (int meta = 0; meta < MAX_METALINKS; ++meta) {
        _last_expo.setIdentity();

        const int num_pseudos = _meta_pseudojoints[meta];

        if (num_pseudos == 0) {
            _Pi[meta].setIdentity();
            if (_debug_verbosity) {
                std::cout << "[initializePseudoTfsNdof] meta-link " << meta
                          << " has 0 pseudojoints → _Pi[" << meta
                          << "] = Identity\n";
            }
            continue;
        }

        if (_debug_verbosity) {
            std::cout << "[initializePseudoTfsNdof] meta-link " << meta
                      << " has " << num_pseudos << " pseudojoints\n";
        }

        for (int j = 0; j < num_pseudos; ++j) {
            const int index = _last_twist_cnt;

            // Passive twist and pseudo angle from YAML-backed model
            Eigen::Matrix<float, 6, 1> xi =
                _ptr2abstract_ndof->get_PASSIVE_TWIST(index);
            float theta =
                _ptr2abstract_ndof->get_PSEUDO_ANGLE(index);

            if (_debug_verbosity) {
                std::cout << "\n[initializePseudoTfsNdof] "
                          << "meta-link " << meta
                          << ", pseudo #" << index << "\n";
                std::cout << "Twist (xi):    " << xi.transpose() << "\n";
                std::cout << "Angle (theta): " << theta << "\n";
            }

            Eigen::Isometry3f exp_xi_theta = twistExp(xi, theta);

            if (_debug_verbosity) {
                std::cout << "twistExp(xi, theta):\n"
                          << exp_xi_theta.matrix() << "\n";
            }

            // Compound within this meta-link
            _Pi[meta] = _last_expo * exp_xi_theta;

            if (_debug_verbosity) {
                std::cout << "_Pi[" << meta << "] after compounding:\n"
                          << _Pi[meta].matrix() << "\n";
            }

            _last_expo = _Pi[meta];
            ++_last_twist_cnt;
        }
    }

    if (_debug_verbosity) {
        std::cout << "\n[initializePseudoTfsNdof] Final _Pi matrices:\n";
        for (int i = 0; i < MAX_METALINKS; ++i) {
            std::cout << "_Pi[" << i << "]:\n"
                      << _Pi[i].matrix() << "\n";
        }
    }
}

// ==================== 3) relative TFs & local screws ====================

void ScrewsKinematicsNdof::initializeRelativeTfs()
{
    _debug_verbosity = false;

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::initializeRelativeTfs] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Bi[0] = g_sa1(0)
    _Bi[0] = *(_ptr2abstract_ndof->gsai_ptr[0]);
    if (_debug_verbosity) {
        std::cout << "[initializeRelativeTfs] B0 =\n" << _Bi[0].matrix() << '\n';
    }

    // Bi[i] = g_sai(i) * g_sai(i-1)^{-1}
    for (int i = 1; i < _dof + 1; ++i) {
        _Bi[i] = extractRelativeTf(
            *(_ptr2abstract_ndof->gsai_ptr[i]),
            *(_ptr2abstract_ndof->gsai_ptr[i - 1]));

        if (_debug_verbosity) {
            std::cout << "[initializeRelativeTfs] B" << i << " =\n"
                      << _Bi[i].matrix() << '\n';
        }
    }
}

void ScrewsKinematicsNdof::initializeLocalScrewCoordVectors()
{
    _debug_verbosity = false;

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::initializeLocalScrewCoordVectors] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Joint frames
    for (int i = 0; i < _dof; ++i) {
        _iXi[i] = extractLocalScrewCoordVector(
            *(_ptr2abstract_ndof->gsai_ptr[i]),
            _ptr2abstract_ndof->active_twists[i]);
    }

    // Tool frame: local screw coords from relative tool->last-joint TF
    Eigen::Isometry3f Bi_tool = extractRelativeTf(
        *(_ptr2abstract_ndof->gsai_ptr[_dof]),
        *(_ptr2abstract_ndof->gsai_ptr[_dof - 1]));

    vee(_iXi[_dof], Bi_tool.matrix());

    if (_debug_verbosity) {
        std::cout << "[initializeLocalScrewCoordVectors] iXi (0.."
                  << _dof << ") initialized\n";
    }
}

void ScrewsKinematicsNdof::initializeReferenceAnatomyActiveTwists()
{
    if (!_ptr2abstract_ndof) {
        std::cerr << "[initializeReferenceAnatomyActiveTwistsNdof] "
                  << "RobotAbstractBaseNdof pointer is null.\n";
        return;
    }

    const int dof = _dof;

    if (dof < 3 || dof > robot_params::MAX_DOF) {
        std::cerr << "[initializeReferenceAnatomyActiveTwistsNdof] "
                  << "Invalid DOF = " << dof
                  << " (expected 3.." << robot_params::MAX_DOF << ").\n";
        return;
    }

    // --------------------------------------------------------------------
    // Joint 1: no meta-link before it → identity transform
    // --------------------------------------------------------------------
    _ptr2abstract_ndof->active_twists[0] =
        _ptr2abstract_ndof->active_twists_anat[0];

    // --------------------------------------------------------------------
    // Joint 2: after meta1 → use Ad(P1)^(-1)
    // --------------------------------------------------------------------
    if (dof >= 2) {
        ad(_ad, _Pi[0]);  // Ad(P1)
        Eigen::Matrix<float, 6, 6> Ad_P1_inv = _ad.inverse();

        _ptr2abstract_ndof->active_twists[1] =
            Ad_P1_inv * _ptr2abstract_ndof->active_twists_anat[1];
    }

    // --------------------------------------------------------------------
    // Joint 3: after meta1 + meta2 → Ad(P1 P2)^(-1)
    // (this reproduces exactly the 3DOF behaviour you posted)
    // --------------------------------------------------------------------
    if (dof >= 3) {
        ad(_ad, _Pi[0] * _Pi[1]);  // Ad(P1 P2)
        Eigen::Matrix<float, 6, 6> Ad_P12_inv = _ad.inverse();

        _ptr2abstract_ndof->active_twists[2] =
            Ad_P12_inv * _ptr2abstract_ndof->active_twists_anat[2];
    }

    // --------------------------------------------------------------------
    // Joints 4–6: after meta1 + meta2 + meta3 → Ad(P1 P2 P3)^(-1)
    // Pattern: active1 - meta1 - active2 - meta2 - active3 - meta3 - active4 - active5 - active6
    // All joints downstream of meta3 see the same P1 P2 P3 transform.
    // --------------------------------------------------------------------
    if (dof >= 4) {
        ad(_ad, _Pi[0] * _Pi[1] * _Pi[2]);  // Ad(P1 P2 P3)
        Eigen::Matrix<float, 6, 6> Ad_P123_inv = _ad.inverse();

        // Joint 4
        _ptr2abstract_ndof->active_twists[3] =
            Ad_P123_inv * _ptr2abstract_ndof->active_twists_anat[3];

        // Joint 5 (if exists)
        if (dof >= 5) {
            _ptr2abstract_ndof->active_twists[4] =
                Ad_P123_inv * _ptr2abstract_ndof->active_twists_anat[4];
        }

        // Joint 6 (if exists)
        if (dof >= 6) {
            _ptr2abstract_ndof->active_twists[5] =
                Ad_P123_inv * _ptr2abstract_ndof->active_twists_anat[5];
        }
    }

    // --------------------------------------------------------------------
    // Update pointers to reference twists
    // --------------------------------------------------------------------
    for (int i = 0; i < dof; ++i) {
        _ptr2abstract_ndof->ptr2_active_twists[i] =
            &_ptr2abstract_ndof->active_twists[i];
    }

    std::cout << "[initializeReferenceAnatomyActiveTwistsNdof] Reference twists initialized (DOF = "
              << dof << "):\n";
    for (int i = 0; i < dof; ++i) {
        std::cout << "active_twists[" << i << "] = "
                  << _ptr2abstract_ndof->active_twists[i].transpose()
                  << std::endl;
    }
}

void ScrewsKinematicsNdof::initializeReferenceAnatomyActiveTfs()
{
    if (!_ptr2abstract_ndof) {
        std::cerr << "[initializeReferenceAnatomyActiveTfsNdof] "
                  << "RobotAbstractBaseNdof pointer is null.\n";
        return;
    }

    const int dof = _dof;

    if (dof < 3 || dof > robot_params::MAX_DOF) {
        std::cerr << "[initializeReferenceAnatomyActiveTfsNdof] "
                  << "Invalid DOF = " << dof
                  << " (expected 3.." << robot_params::MAX_DOF << ").\n";
        return;
    }

    // ------------------------------------------------------------
    // Mapping (same philosophy as the 3DOF case you posted):
    //
    // active1 - meta1 - active2 - meta2 - active3 - meta3 - active4 - active5 - active6
    //
    // J1 (idx 0): no pseudos before it                -> I
    // J2 (idx 1): after meta1                         -> P1
    // J3 (idx 2): after meta1 + meta2                 -> P1 P2
    // J4–J6 (idx 3..5): after meta1 + meta2 + meta3   -> P1 P2 P3
    //
    // TCP (idx = dof):
    //   - DOF=3:  after P1 P2        (keep EXACT 3DOF behaviour)
    //   - DOF>=4: after P1 P2 P3
    // ------------------------------------------------------------

    // Joint 1: no pseudo before it
    _ptr2abstract_ndof->g_ref_0[0] = _ptr2abstract_ndof->g_test_0[0];

    // Joint 2: undo meta1
    if (dof >= 2) {
        const Eigen::Isometry3f P1_inv = _Pi[0].inverse();
        _ptr2abstract_ndof->g_ref_0[1] = P1_inv * _ptr2abstract_ndof->g_test_0[1];
    }

    // Joint 3: undo meta1 + meta2
    if (dof >= 3) {
        const Eigen::Isometry3f P12_inv = (_Pi[0] * _Pi[1]).inverse();
        _ptr2abstract_ndof->g_ref_0[2] = P12_inv * _ptr2abstract_ndof->g_test_0[2];

        // 3DOF case: TCP is also after meta2 -> use P1 P2 (exact old behaviour)
        if (dof == 3) {
            _ptr2abstract_ndof->g_ref_0[3] = P12_inv * _ptr2abstract_ndof->g_test_0[3];
        }
    }

    // DOF >= 4: joints 4..6 and TCP are after meta3 -> use P1 P2 P3
    if (dof >= 4) {
        const Eigen::Isometry3f P123_inv = (_Pi[0] * _Pi[1] * _Pi[2]).inverse();

        // Joint 4 (idx 3)
        _ptr2abstract_ndof->g_ref_0[3] = P123_inv * _ptr2abstract_ndof->g_test_0[3];

        // Joint 5 (idx 4), if present
        if (dof >= 5) {
            _ptr2abstract_ndof->g_ref_0[4] = P123_inv * _ptr2abstract_ndof->g_test_0[4];
        }

        // Joint 6 (idx 5), if present
        if (dof >= 6) {
            _ptr2abstract_ndof->g_ref_0[5] = P123_inv * _ptr2abstract_ndof->g_test_0[5];
        }

        // TCP at index dof
        _ptr2abstract_ndof->g_ref_0[dof] = P123_inv * _ptr2abstract_ndof->g_test_0[dof];
    }

    // Update pointers (even though they were set to g_ref_0 in initializeFromYaml)
    for (int i = 0; i <= dof; ++i) {
        _ptr2abstract_ndof->gsai_ptr[i] = &_ptr2abstract_ndof->g_ref_0[i];
    }

    std::cout << "[initializeReferenceAnatomyActiveTfsNdof] Reference transformations initialized (DOF = "
              << dof << "):\n";
    for (int i = 0; i <= dof; ++i) {
        std::cout << "g_ref_0[" << i << "] =\n"
                  << _ptr2abstract_ndof->g_ref_0[i].matrix() << std::endl;
    }
}

// ==================== 4) FK using stored joint state ====================

void ScrewsKinematicsNdof::ForwardKinematicsTCP()
{
    // Use internally stored joint positions (_joint_pos)
    ForwardKinematicsTCP(_joint_pos);
}

// ==================== 5) Jacobians (tool frame) ====================

// ==================== Spatial Jacobian – Option 1 (Tool_1) ====================
// Matches: SpatialJacobian_Tool_1() from 3-DOF case
//   J^s_i = Ad_{g_i} * iXi[i]
void ScrewsKinematicsNdof::computeSpatialJacobianTCP1()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialJacobianTCP1] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialJacobianTCP1] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Assumptions (same as 3-DOF):
    //  - ForwardKinematicsTCP(q) was called for current q -> _g[i] are up to date
    //  - initializeLocalScrewCoordVectorsNdof() was called -> _iXi[i] are set

    _debug_verbosity = false;  // same behavior as original SpatialJacobian_Tool_1

    for (int i = 0; i < _dof; ++i) {
        // Ad_{g_i}
        ad(_ad, _g[i]);

        // J^s_i = Ad_{g_i} * iXi[i]
        _Jsp_tool.col(i) = _ad * _iXi[i];
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof] Spatial Jacobian 1 (Ad(g_i) * iXi[i]):\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jsp_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

// ==================== Spatial Jacobian – Option 2 (Tool_2) ====================
// Matches: SpatialJacobian_Tool_2() from 3-DOF case
//   J^s_i = Ad_{ g_i * (g_ref_i)^{-1} } * active_twists[i]
void ScrewsKinematicsNdof::computeSpatialJacobianTCP2()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialJacobianTCP2] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialJacobianTCP2] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Assumptions:
    //  - initializePseudoTfsNdof()
    //  - initializeReferenceAnatomyActiveTwistsNdof()
    //  - initializeReferenceAnatomyActiveTfsNdof()
    //  - ForwardKinematicsTCP(q)
    // have been called appropriately, so:
    //   _g[i]                          -> current joint frames
    //   _ptr2abstract_ndof->gsai_ptr[i] -> reference anatomy joint frames
    //   _ptr2abstract_ndof->active_twists[i] -> reference anatomy twists

    _debug_verbosity = false;

    for (int i = 0; i < _dof; ++i) {

        // Ad_{ g_i * (g_ref_i)^{-1} }
        Eigen::Isometry3f g_current   = _g[i];
        Eigen::Isometry3f g_reference = *(_ptr2abstract_ndof->gsai_ptr[i]);
        ad(_ad, g_current * g_reference.inverse());

        // J^s_i = Ad_{..} * active_twists[i]
        _Jsp_tool.col(i) = _ad * _ptr2abstract_ndof->active_twists[i];
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof] Spatial Jacobian 2 (Ad(g_i g_ref_i^{-1}) * active_twists[i]):\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jsp_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

// ==================== Spatial Jacobian – Option 3 (Tool_3) ====================
// Matches: SpatialJacobian_Tool() from Murray book.
//   Uses anatomy twists and their exponentials (PoE form).
//
//   J^s = [ xi'_1  xi'_2  ...  xi'_n ]
//   xi'_i = Ad_{ exp(xi_1 * theta_1) * ... * exp(xi_{i-1} * theta_{i-1}) } * xi_i
//
// Preconditions:
//   1. _ptr2abstract_ndof is valid
//   2. _dof in {3,4,5,6}
//   3. active_twists_anat[i] have been loaded from YAML
//   4. setExponentialsAnat(q) has been called for current q, so that
//      _active_expos_anat[i] = exp( xi_anat[i] * q[i] )
void ScrewsKinematicsNdof::computeSpatialJacobianTCP3()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialJacobianTCP3] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialJacobianTCP3] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    _debug_verbosity = false;

    // Prefix product: exp(xi_1 theta_1) * ... * exp(xi_{i-1} theta_{i-1})
    Eigen::Isometry3f prefix = Eigen::Isometry3f::Identity();

    for (int i = 0; i < _dof; ++i) {
        if (i > 0) {
            // Multiply by exp(xi_{i-1} * theta_{i-1}) from setExponentialsAnat()
            prefix = prefix * _active_expos_anat[i - 1];
        }

        // Ad_{prefix}
        ad(_ad, prefix);

        // xi'_i = Ad_{prefix} * xi_anat[i]
        const Eigen::Matrix<float, 6, 1>& xi_anat_i =
            _ptr2abstract_ndof->active_twists_anat[i];

        _Jsp_tool.col(i) = _ad * xi_anat_i;
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof] Spatial Jacobian Tool 3 "
                     "(Murray PoE with anat twists):\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jsp_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

// ===================================================
// How to use Spatial Jacobian options:
// ===================================================
// After FK:
// kin_ndof.ForwardKinematicsTCP(q);
//
// Option 1: directly from Ad(g_i) * iXi[i]
//kin_ndof.computeSpatialJacobianTCP1();
//Eigen::MatrixXf J1 = kin_ndof.getSpatialJacobianTCP();
//
// Option 2: via reference anatomy + pseudos
//kin_ndof.computeSpatialJacobianTCP2();
//Eigen::MatrixXf J2 = kin_ndof.getSpatialJacobianTCP();
//
// Option 3: via test anatomy only
// 1. Ensure anat twists loaded (via RobotAbstractBaseNdof::initializeFromYaml)
// 2. For a given q (size = dof):
//kin_ndof.setExponentialsAnat(q);   // fills _active_expos_anat[i]
// 3. Compute Jacobian (Tool_3)
//kin_ndof.computeSpatialJacobianTCP3();
// 4. Get 6×dof matrix
//Eigen::MatrixXf Js = kin_ndof.getSpatialJacobianTCP();

// ==================== Body Jacobian – Option 1 (Tool_1) ====================
// Matches: BodyJacobian_Tool_1() from 3-DOF case
//   J^b_i = Ad_{ g_T^{-1} g_i } * iXi[i]

void ScrewsKinematicsNdof::computeBodyJacobianTCP1()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobianTCP1] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobianTCP1] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Assumptions (same as 3-DOF version):
    //  1. ForwardKinematicsTCP(q) has been called for current q
    //     => _g[0.._dof], _gst are up to date
    //  2. initializeLocalScrewCoordVectorsNdof() has been called
    //     => _iXi[i] are set

    _debug_verbosity = false;

    for (int i = 0; i < _dof; ++i) {
        // Ad_{ g_T^{-1} g_i }
        ad(_ad, _gst.inverse() * _g[i]);

        // J^b_i = Ad_{ g_T^{-1} g_i } * iXi[i]
        _Jbd_tool.col(i) = _ad * _iXi[i];
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof] Body Jacobian Tool 1 (Ad(g_T^{-1} g_i) * iXi[i]):\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jbd_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

// ==================== Body Jacobian – Option 2 (Tool_2) ====================
// Matches: BodyJacobian_Tool_2() from 3-DOF case
//   J^b_i = Ad_{ g_T^{-1} g_i g_ref_i^{-1} } * active_twists[i]
void ScrewsKinematicsNdof::computeBodyJacobianTCP2()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobianTCP2] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobianTCP2] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Assumptions:
    //  1. initializePseudoTfsNdof()
    //  2. initializeReferenceAnatomyActiveTwistsNdof()
    //  3. initializeReferenceAnatomyActiveTfsNdof()
    //  4. ForwardKinematicsTCP(q)
    // have been called appropriately, so:
    //   _g[i]                                  -> current joint frames
    //   _gst                                   -> current TCP frame
    //   _ptr2abstract_ndof->gsai_ptr[i]       -> reference joint frames
    //   _ptr2abstract_ndof->active_twists[i]  -> reference twists

    _debug_verbosity = false;

    for (int i = 0; i < _dof; ++i) {
        Eigen::Isometry3f g_ref_i = *(_ptr2abstract_ndof->gsai_ptr[i]);

        // Ad_{ g_T^{-1} g_i g_ref_i^{-1} }
        ad(_ad, _gst.inverse() * _g[i] * g_ref_i.inverse());

        // J^b_i = Ad_{..} * active_twists[i]
        _Jbd_tool.col(i) = _ad * _ptr2abstract_ndof->active_twists[i];
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof] Body Jacobian Tool 2 "
                     "(Ad(g_T^{-1} g_i g_ref_i^{-1}) * active_twists[i]):\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jbd_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

// ==================== Body Jacobian – Option 3 (Tool_3, TCP) ====================
// Murray-style PoE body Jacobian, expressed in the {T} frame, using ONLY
// anatomy twists and their exponentials.
//
//   J^s_tool_b(q) = [ xi'_1  xi'_2  ...  xi'_n ]
//   xi'_i = Ad_{ exp(xi_i * theta_i) * ... * exp(xi_n * theta_n) * gst_0 }^{-1} * xi_i
//
// Preconditions:
//   1. _ptr2abstract_ndof is valid
//   2. _dof in {3,4,5,6}
//   3. active_twists_anat[i] have been loaded from YAML
//   4. setExponentialsAnat(q) has been called for current q so that
//      _active_expos_anat[i] = exp( xi_anat[i] * q[i] )
void ScrewsKinematicsNdof::computeBodyJacobianTCP3()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobianTCP3] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobianTCP3] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    _debug_verbosity = false;

    // gst_0: TCP pose at q = 0 (loaded from YAML as gst_test_0)
    const Eigen::Isometry3f& gst0 = _ptr2abstract_ndof->g_test_0[_dof];

    for (int i = 0; i < _dof; ++i) {
        // suffix_i = exp(xi_i * theta_i) * ... * exp(xi_n * theta_n) * gst_0
        Eigen::Isometry3f suffix = Eigen::Isometry3f::Identity();

        for (int k = i; k < _dof; ++k) {
            suffix = suffix * _active_expos_anat[k];
        }
        suffix = suffix * gst0;

        // Ad_{suffix}
        ad(_ad, suffix);

        // xi'_i = Ad_{suffix}^{-1} * xi_i
        const Eigen::Matrix<float, 6, 1>& xi_anat_i =
            _ptr2abstract_ndof->active_twists_anat[i];

        _Jbd_tool.col(i) = _ad.inverse() * xi_anat_i;
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof] Body Jacobian Tool 3 "
                     "(Murray PoE with anat twists, TCP frame):\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jbd_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

// ===================================================
// How to use TCP Body Jacobian options:
// ===================================================
// After FK:
// kin_ndof.ForwardKinematicsTCP(q);
//
// Option 1: directly from Ad(g_i) * iXi[i]
// kin_ndof.computeBodyJacobianTCP1();
// Eigen::MatrixXf Jb1 = kin_ndof.getBodyJacobianTCP();
//
// Option 2: via reference anatomy + pseudos
// kin_ndof.computeBodyJacobianTCP2();
// Eigen::MatrixXf Jb2 = kin_ndof.getBodyJacobianTCP();
// Option 3: via test anatomy twists and tfs
// q: current joint vector (size = dof)
// kin_ndof.setExponentialsAnat(q);     // fills _active_expos_anat[i]
// Murray-style body Jacobian (option 3)
// kin_ndof.computeBodyJacobianTCP3();
// Eigen::MatrixXf Jb = kin_ndof.getBodyJacobianTCP();  // 6 x dof
// ==================== Jacobian getters ====================

Eigen::Matrix<float, 6, Eigen::Dynamic>
ScrewsKinematicsNdof::getSpatialJacobianTCP() const
{
    Eigen::Matrix<float, 6, Eigen::Dynamic> J(6, _dof);
    if (_dof > 0) {
        J = _Jsp_tool.block(0, 0, 6, _dof);
    } else {
        J.setZero();
    }
    return J;
}

Eigen::Matrix<float, 6, Eigen::Dynamic>
ScrewsKinematicsNdof::getBodyJacobianTCP() const
{
    Eigen::Matrix<float, 6, Eigen::Dynamic> J(6, _dof);
    if (_dof > 0) {
        J = _Jbd_tool.block(0, 0, 6, _dof);
    } else {
        J.setZero();
    }
    return J;
}

void ScrewsKinematicsNdof::VelocityTwistTCP(typ_jacobian jacob_selection)
{
    // Returns the spatial OR the body velocity twist @ current [q, dq]
    _debug_verbosity = false;

    if (_dof <= 0) {
        std::cerr
            << "[ScrewsKinematicsNdof::VelocityTwistTCP] DOF <= 0, aborting.\n";
        return;
    }

    // Build dq_vector from stored joint velocities (_joint_vel[0.._dof-1])
    Eigen::Matrix<float, Eigen::Dynamic, 1> dq_vector(_dof);
    for (int i = 0; i < _dof; ++i) {
        dq_vector(i) = _joint_vel[i];
    }

    switch (jacob_selection)
    {
      case typ_jacobian::SPATIAL:
      {
        // Use already-computed spatial Jacobian (6 x _dof)
        Eigen::Matrix<float, 6, Eigen::Dynamic> Jsp = getSpatialJacobianTCP();
        if (Jsp.cols() != _dof) {
            std::cerr
                << "[ScrewsKinematicsNdof::VelocityTwistTCP] "
                << "Jsp.cols() = " << Jsp.cols()
                << " but _dof = " << _dof << "\n";
        }

        _Vsp_twist_tcp = Jsp * dq_vector;

        if (_debug_verbosity) {
            std::cout
              << "[ScrewsKinematicsNdof::VelocityTwistTCP] Spatial Velocity Twist:\n";
            printTwist(_Vsp_twist_tcp);
        }
        break;
      }

      case typ_jacobian::BODY:
      {
        // Use already-computed body Jacobian (6 x _dof)
        Eigen::Matrix<float, 6, Eigen::Dynamic> Jbd = getBodyJacobianTCP();
        if (Jbd.cols() != _dof) {
            std::cerr
                << "[ScrewsKinematicsNdof::VelocityTwistTCP] "
                << "Jbd.cols() = " << Jbd.cols()
                << " but _dof = " << _dof << "\n";
        }

        _Vbd_twist_tcp = Jbd * dq_vector;

        if (_debug_verbosity) {
            std::cout
              << "[ScrewsKinematicsNdof::VelocityTwistTCP] Body Velocity Twist:\n";
            printTwist(_Vbd_twist_tcp);
        }
        break;
      }

      default:
        std::cerr
          << "[ScrewsKinematicsNdof::VelocityTwistTCP] "
          << "WRONG JACOBIAN SELECTION FOR VELOCITY TWIST\n";
        break;
    }
}

void ScrewsKinematicsNdof::printTwist(Eigen::Matrix<float, 6, 1> twist) {
    for (size_t i = 0; i < 6; i++) {
        std::cout << twist[i] << std::endl;
    }
    return;
}

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

    // Initialize per-frame body Jacobian storage and pointer table
    for (int k = 0; k < MAX_DOF + 1; ++k) {
        for (int i = 0; i < MAX_DOF; ++i) {
            _BodyJacobiansFrames[k][i].setZero();
            _ptr2BodyJacobiansFrames[k][i] = &_BodyJacobiansFrames[k][i];
        }
    }

    for (int k = 0; k < MAX_DOF; ++k) {
        for (int j = 0; j < MAX_DOF; ++j) {
            _BodyCOMJacobiansFrames[k][j].setZero();
            _ptr2BodyCOMJacobiansFrames[k][j] = &_BodyCOMJacobiansFrames[k][j];
    }
}

}

void ScrewsKinematicsNdof::initializePseudoTfs()
{
    _debug_verbosity = true;

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
    _debug_verbosity = true;

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::initializeRelativeTfs] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Bi[0] = g_sa1(0)
    _Bi[0] = *(_ptr2abstract_ndof->gsai_test_ptr[0]);
    if (_debug_verbosity) {
        std::cout << "[initializeRelativeTfs] B0 =\n" << _Bi[0].matrix() << '\n';
    }

    // Bi[i] = g_sai(i) * g_sai(i-1)^{-1}
    for (int i = 1; i < _dof + 1; ++i) {
        _Bi[i] = extractRelativeTf(
            *(_ptr2abstract_ndof->gsai_test_ptr[i]),
            *(_ptr2abstract_ndof->gsai_test_ptr[i - 1]));

        if (_debug_verbosity) {
            std::cout << "[initializeRelativeTfs] B" << i << " =\n"
                      << _Bi[i].matrix() << '\n';
        }
    }
}

/*
void ScrewsKinematicsNdof::initializeLocalScrewCoordVectors()
{
    _debug_verbosity = true;

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::initializeLocalScrewCoordVectors] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Joint frames
    for (int i = 0; i < _dof; ++i) {
        _iXi[i] = extractLocalScrewCoordVector(
        *(_ptr2abstract_ndof->gsai_test_ptr[i]),
        _ptr2abstract_ndof->active_twists_anat[i]);
    }

    // Tool frame: local screw coords from relative tool->last-joint TF
    Eigen::Isometry3f Bi_tool = extractRelativeTf(
    *(_ptr2abstract_ndof->gsai_test_ptr[_dof]),
    *(_ptr2abstract_ndof->gsai_test_ptr[_dof - 1]));

    vee(_iXi[_dof], Bi_tool.matrix());

    if (_debug_verbosity) {
        std::cout << "[initializeLocalScrewCoordVectors] iXi (0.."
                  << _dof << ") initialized\n";
    }
}
*/
void ScrewsKinematicsNdof::initializeLocalScrewCoordVectors()
{
    _debug_verbosity = true;

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::initializeLocalScrewCoordVectors] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    std::cout << "\n============================================================\n";
    std::cout << "[ScrewsKinematicsNdof::initializeLocalScrewCoordVectors] START\n";
    std::cout << "============================================================\n";

    // Joint frames
    for (int i = 0; i < _dof; ++i) {

        if (!_ptr2abstract_ndof->gsai_test_ptr[i]) {
            std::cerr << "[initializeLocalScrewCoordVectors] gsai_test_ptr[" << i
                      << "] is null\n";
            continue;
        }

        const Eigen::Isometry3f& g_i = *(_ptr2abstract_ndof->gsai_test_ptr[i]);
        const Eigen::Matrix<float, 6, 1>& xi_s_i =
            _ptr2abstract_ndof->active_twists_anat[i];

        Eigen::Matrix<float, 6, 1> xi_local =
            extractLocalScrewCoordVector(g_i, xi_s_i);

        _iXi[i] = xi_local;

        std::cout << "\n[initializeLocalScrewCoordVectors] joint i = " << i << "\n";
        std::cout << "gsai_test_ptr[" << i << "] =\n" << g_i.matrix() << "\n";
        std::cout << "active_twists_anat[" << i << "] = "
                  << xi_s_i.transpose() << "\n";
        std::cout << "extracted _iXi[" << i << "] = "
                  << _iXi[i].transpose() << "\n";

        if (!_iXi[i].allFinite()) {
            std::cerr << "[initializeLocalScrewCoordVectors] WARNING: _iXi[" << i
                      << "] contains non-finite values\n";
        }

        if (_iXi[i].cwiseAbs().maxCoeff() < 1e-12f) {
            std::cerr << "[initializeLocalScrewCoordVectors] WARNING: _iXi[" << i
                      << "] is suspiciously close to zero\n";
        }
    }

    // Tool frame: local screw coords from relative tool->last-joint TF
    if (!_ptr2abstract_ndof->gsai_test_ptr[_dof]) {
        std::cerr << "[initializeLocalScrewCoordVectors] gsai_test_ptr[" << _dof
                  << "] (tool) is null\n";
        return;
    }

    if (!_ptr2abstract_ndof->gsai_test_ptr[_dof - 1]) {
        std::cerr << "[initializeLocalScrewCoordVectors] gsai_test_ptr[" << (_dof - 1)
                  << "] (last joint) is null\n";
        return;
    }

    Eigen::Isometry3f Bi_tool = extractRelativeTf(
        *(_ptr2abstract_ndof->gsai_test_ptr[_dof]),
        *(_ptr2abstract_ndof->gsai_test_ptr[_dof - 1]));

    vee(_iXi[_dof], Bi_tool.matrix());

    std::cout << "\n[initializeLocalScrewCoordVectors] tool relative TF Bi_tool =\n"
              << Bi_tool.matrix() << "\n";
    std::cout << "[initializeLocalScrewCoordVectors] _iXi[" << _dof << "] (tool) = "
              << _iXi[_dof].transpose() << "\n";

    std::cout << "\n[initializeLocalScrewCoordVectors] FINAL iXi values:\n";
    for (int i = 0; i <= _dof; ++i) {
        std::cout << "_iXi[" << i << "] = " << _iXi[i].transpose() << "\n";
    }

    std::cout << "============================================================\n";
    std::cout << "[ScrewsKinematicsNdof::initializeLocalScrewCoordVectors] END\n";
    std::cout << "============================================================\n" << std::flush;
}

void ScrewsKinematicsNdof::initializeSpatialJointScrewCoordVectors()
{
    _debug_verbosity = false;

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::initializeSpatialJointScrewCoordVectors] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    if (_dof <= 0 || _dof > MAX_DOF) {
        std::cerr << "[ScrewsKinematicsNdof::initializeSpatialJointScrewCoordVectors] "
                     "Invalid DOF = " << _dof << "\n";
        return;
    }

    // Yi := spatial joint screw coordinates of the CURRENT anatomy
    // These are already loaded from YAML into active_twists_anat[i].
    for (int i = 0; i < _dof; ++i) {
        _Yi[i] = _ptr2abstract_ndof->active_twists_anat[i];
    }

    // Zero remaining slots for safety
    for (int i = _dof; i < MAX_DOF; ++i) {
        _Yi[i].setZero();
    }

    if (_debug_verbosity) {
        std::cout << "[initializeSpatialJointScrewCoordVectors] Yi initialized:\n";
        for (int i = 0; i < _dof; ++i) {
            std::cout << "_Yi[" << i << "] = "
                      << _Yi[i].transpose() << "\n";
        }
    }
}

const Eigen::Matrix<float, 6, 1>& ScrewsKinematicsNdof::getSpatialJointScrewCoordVector(int i) const
{
    static Eigen::Matrix<float, 6, 1> zero = Eigen::Matrix<float, 6, 1>::Zero();

    if (i < 0 || i >= _dof) {
        std::cerr << "[ScrewsKinematicsNdof::getSpatialJointScrewCoordVector] "
                     "Index out of bounds: " << i << "\n";
        return zero;
    }

    return _Yi[i];
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
    _debug_verbosity = true;

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

void ScrewsKinematicsNdof::initializeHomeAnatomyActiveTfs()
{
    for (int i = 0; i <= _dof; ++i) {
        _g0[i] = _ptr2abstract_ndof->g_test_0[i];

        std::cout << "[ScrewsKinematicsNdof::initializeHomeAnatomyActiveTfs] _g0[" << i << "] =\n"
                  << _g0[i].matrix() << std::endl;
    }

    for (int i = _dof + 1; i < robot_params::MAX_DOF + 1; ++i) {
        _g0[i] = Eigen::Isometry3f::Identity();
    }
}

void ScrewsKinematicsNdof::initializeHomeAnatomyCOMTfs()
{
    for (int i = 0; i < _dof; ++i) {
        _gl0[i] = _ptr2abstract_ndof->gl_test_0[i];

        std::cout << "[ScrewsKinematicsNdof::initializeHomeAnatomyCOMTfs] _gl0[" << i << "] =\n"
                  << _gl0[i].matrix() << std::endl;
    }

    for (int i = _dof + 1; i < robot_params::MAX_DOF; ++i) {
        _gl0[i] = Eigen::Isometry3f::Identity();
    }
}

// ==================== 4) FK using stored joint state ====================

void ScrewsKinematicsNdof::ForwardKinematicsTCP()
{
    // Use internally stored joint positions (_joint_pos)
    ForwardKinematicsTCP(_joint_pos);
}

// FK using internally stored _joint_pos
void ScrewsKinematicsNdof::ForwardKinematicsCOM()
{
    ForwardKinematicsCOM(_joint_pos);
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

void ScrewsKinematicsNdof::computeBodyJacobiansFrames1()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobiansFrames1] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobiansFrames1] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Preconditions:
    //  1) initializeRelativeTfs() has been called     -> _Bi
    //  2) initializeLocalScrewCoordVectors() called   -> _iXi[0.._dof-1]
    //  3) ForwardKinematicsTCP(q) called for current q
    //     -> _g[0.._dof] are up to date (joint frames + TCP)
    //
    // Implements the 1st "=" of Eq. 4.16:
    //   J^b_{k,i} = Ad_{ g_k^{-1} g_i } * iXi[i]
    //
    // Serial-chain sparsity:
    //   - real body frames k = 0.._dof-1 only depend on upstream joints i <= k
    //   - TCP frame k = _dof depends on all joints i = 0.._dof-1

    _debug_verbosity = true;

    std::cout << "\n[computeBodyJacobiansFrames1] local screw vectors _iXi:\n";
    for (int i = 0; i < _dof; ++i) {
        std::cout << "_iXi[" << i << "] = " << _iXi[i].transpose() << "\n";
    }
    std::cout << std::flush;

    for (int k = 0; k <= _dof; ++k) {

        const Eigen::Isometry3f& g_k = _g[k];

        for (int i = 0; i < _dof; ++i) {

            const bool allowed = ((k < _dof) && (i <= k)) || (k == _dof);

            Eigen::Matrix<float, 6, 1> col = Eigen::Matrix<float, 6, 1>::Zero();

            if (allowed) {
                const Eigen::Isometry3f& g_i = _g[i];

                // Ad_{ g_k^{-1} g_i }
                ad(_ad, g_k.inverse() * g_i);

                // J^b_(k),i = Ad_{ g_k^{-1} g_i } * iXi[i]
                col = _ad * _iXi[i];
            }

            _BodyJacobiansFrames[k][i] = col;

            if (_ptr2BodyJacobiansFrames[k][i]) {
                *(_ptr2BodyJacobiansFrames[k][i]) = col;
            }
        }

        if (_debug_verbosity) {
            std::cout << "[ScrewsKinematicsNdof::computeBodyJacobiansFrames1] "
                         "J^b for frame k=" << k << ":\n";
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < _dof; ++c) {
                    std::cout << _BodyJacobiansFrames[k][c](r) << "\t";
                }
                std::cout << "\n";
            }
        }
    }
}

void ScrewsKinematicsNdof::computeBodyJacobiansFrames2()
{
    _debug_verbosity = true;

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobiansFrames2] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobiansFrames2] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Preconditions:
    //  1) initializeHomeAnatomyActiveTfs() has been called -> _g0[0.._dof]
    //  2) initializeSpatialJointScrewCoordVectors() or equivalent called -> _Yi[0.._dof-1]
    //  3) ForwardKinematicsTCP(q) called for current q
    //     -> _g[0.._dof] are up to date (joint frames + TCP)
    //
    // Implements the 2nd "=" of Eq. 4.16:
    //   J^b_{k,j} = Ad_{ C_{k,j} A_j^{-1} } Y_j
    // with
    //   C_{k,j} = g_k^{-1} g_j
    //   A_j     = _g0[j]
    //
    // Serial-chain sparsity:
    //   - real body frames k = 0.._dof-1 only depend on upstream joints j <= k
    //   - TCP frame k = _dof depends on all joints j = 0.._dof-1

    _debug_verbosity = false;

    for (int k = 0; k <= _dof; ++k) {

        const Eigen::Isometry3f& g_k = _g[k];

        for (int j = 0; j < _dof; ++j) {

            const bool allowed = ((k < _dof) && (j <= k)) || (k == _dof);

            Eigen::Matrix<float, 6, 1> col = Eigen::Matrix<float, 6, 1>::Zero();

            if (allowed) {
                const Eigen::Isometry3f& g_j = _g[j];
                const Eigen::Isometry3f& A_j = _g0[j];
                const Eigen::Matrix<float, 6, 1>& Y_j = _Yi[j];

                // Ad_{ C_{k,j} A_j^{-1} } = Ad_{ g_k^{-1} g_j A_j^{-1} }
                ad(_ad, g_k.inverse() * g_j * A_j.inverse());
                col = _ad * Y_j;
            }

            _BodyJacobiansFrames[k][j] = col;

            if (_ptr2BodyJacobiansFrames[k][j]) {
                *(_ptr2BodyJacobiansFrames[k][j]) = col;
            }
        }

        if (_debug_verbosity) {
            std::cout << "[ScrewsKinematicsNdof::computeBodyJacobiansFrames2] "
                         "J^b for frame k=" << k << ":\n";
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < _dof; ++c) {
                    std::cout << _BodyJacobiansFrames[k][c](r) << "\t";
                }
                std::cout << "\n";
            }
        }
    }
}

const Eigen::Matrix<float, 6, 1>&
ScrewsKinematicsNdof::getBodyJacobianFrame(int frameIndex, int jointIndex) const
{
    static Eigen::Matrix<float, 6, 1> dummy_zero = Eigen::Matrix<float, 6, 1>::Zero();

    if (frameIndex < 0 || frameIndex > _dof ||
        jointIndex < 0 || jointIndex >= _dof)
    {
        std::cerr << "[ScrewsKinematicsNdof::getBodyJacobianFrame] "
                     "Index out of range: frameIndex=" << frameIndex
                  << ", jointIndex=" << jointIndex
                  << ", dof=" << _dof << "\n";
        return dummy_zero;
    }

    return _BodyJacobiansFrames[frameIndex][jointIndex];
}

void ScrewsKinematicsNdof::computeBodyCOMJacobiansFrames()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyCOMJacobiansFrames] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyCOMJacobiansFrames] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    // Preconditions:
    //  1) initializeHomeAnatomyCOMTfs() has been called
    //     -> _gl0[0.._dof-1]
    //  2) initializeSpatialJointScrewCoordVectors() called
    //     -> _Yi[0.._dof-1]
    //  3) ForwardKinematicsCOM(q) has been called for current q
    //     -> _gl[0.._dof-1] are up to date (link COM frames)
    //
    // Implements the 2nd "=" of Eq. 4.16 for LINK COM frames:
    //   J^{b,com}_{k,j} = Ad_{ C^{com}_{k,j} A_j^{-1} } Y_j
    // with
    //   C^{com}_{k,j} = g_{l,k}^{-1} g_j
    //   A_j           = _gl0[j]
    //
    // Here:
    //   - k = 0.._dof-1 indexes the link COM frames
    //   - j = 0.._dof-1 indexes the actuated joints
    //
    // Serial-chain sparsity:
    //   - COM frame k only depends on upstream joints j <= k

    _debug_verbosity = true;

    for (int k = 0; k < _dof; ++k) {

        const Eigen::Isometry3f& g_k = _gl[k];

        for (int j = 0; j < _dof; ++j) {

            std::cout << "[DEBUG] COM k=" << k << ", j=" << j << std::endl;
            
            const bool allowed = (j <= k);

            Eigen::Matrix<float, 6, 1> col = Eigen::Matrix<float, 6, 1>::Zero();

            if (allowed) {
                const Eigen::Isometry3f& g_j = _gl[j];
                const Eigen::Isometry3f& A_j = _gl0[j];
                const Eigen::Matrix<float, 6, 1>& Y_j = _Yi[j];

                // Ad_{ C^{com}_{k,j} A_j^{-1} } = Ad_{ g_{l,k}^{-1} g_j A_j^{-1} }
                ad(_ad, g_k.inverse() * g_j * A_j.inverse());
                col = _ad * Y_j;
            }

            _BodyCOMJacobiansFrames[k][j] = col;

            if (_ptr2BodyCOMJacobiansFrames[k][j]) {
                *(_ptr2BodyCOMJacobiansFrames[k][j]) = col;
            }
        }

        if (_debug_verbosity) {
            std::cout << "[ScrewsKinematicsNdof::computeBodyCOMJacobiansFrames] "
                         "J^{b,com} for COM frame k=" << k << ":\n";
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < _dof; ++c) {
                    std::cout << _BodyCOMJacobiansFrames[k][c](r) << "\t";
                }
                std::cout << "\n";
            }
        }
    }
}

void ScrewsKinematicsNdof::computeHybridJacobianTCP()
{
    // =========================================================================
    // N-DOF implementation of the MATLAB function:
    //   [Jh_tcp, Jb_tcp, g_last_tcp] = calculateHybridJacobianTCP(...)
    //
    // MATLAB source meaning:
    //   Computes the TCP hybrid Jacobian from the BODY Jacobian of the last
    //   actual joint frame.
    //
    // Inputs in MATLAB:
    //   C_last   : spatial transform of last actual joint axis frame
    //   C_tcp    : spatial transform of tcp frame
    //   Jb_last  : 6xn body Jacobian of last actual body frame
    //
    // Mapping in this N-DOF class:
    //   C_last   --> _g[_dof - 1]
    //   C_tcp    --> _g[_dof]
    //   Jb_last  --> _BodyJacobiansFrames[_dof - 1][j], j = 0.._dof-1
    //
    // Outputs stored in this class:
    //   Jb_tcp      --> _Jbd_tool
    //   Jh_tcp      --> _Jh_tcp
    //   g_last_tcp  --> _g_last_tcp
    //
    // Convention:
    //   Twists ordered as [v; w]
    //
    // Preconditions:
    //   1) ForwardKinematicsTCP(...) has been called for the current q so that
    //      _g[0.._dof] are up to date.
    //   2) computeBodyJacobiansFrames1() or computeBodyJacobiansFrames2() has
    //      been called so that _BodyJacobiansFrames are up to date.
    //
    // Note:
    //   This function does NOT recompute body Jacobians automatically.
    //   It assumes the frame-wise body Jacobians already exist, exactly as in
    //   the MATLAB workflow where Jb_last is given as input.
    // =========================================================================

    _debug_verbosity = true;
    _is_operational_jacobian_valid = false;

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeHybridJacobianTCP] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeHybridJacobianTCP] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    _debug_verbosity = true;

    const int last_frame_index = _dof - 1;
    const int tcp_frame_index  = _dof;

    // C_last and C_tcp
    const Eigen::Isometry3f& C_last = _g[last_frame_index];
    const Eigen::Isometry3f& C_tcp  = _g[tcp_frame_index];

    // Step 1: g_last_tcp = inv(C_last) * C_tcp
    _g_last_tcp = C_last.inverse() * C_tcp;

    // Step 2: Jb_tcp = ad(inv(g_last_tcp)) * Jb_last
    //
    // Here Jb_last(:,j) is stored as _BodyJacobiansFrames[last_frame_index][j]
    Eigen::Matrix<float, 6, 6> ad_inv_g_last_tcp;
    ad(ad_inv_g_last_tcp, _g_last_tcp.inverse());

    for (int j = 0; j < _dof; ++j) {
        _Jbd_tool.col(j) = ad_inv_g_last_tcp * _BodyJacobiansFrames[last_frame_index][j];
    }

    // Step 3: Jh_tcp = ad(g_rot) * Jb_tcp
    // where g_rot contains only the rotation part of C_tcp
    Eigen::Isometry3f g_rot = Eigen::Isometry3f::Identity();
    g_rot.linear() = C_tcp.linear();
    g_rot.translation().setZero();

    Eigen::Matrix<float, 6, 6> ad_g_rot;
    ad(ad_g_rot, g_rot);

    for (int j = 0; j < _dof; ++j) {
        _Jh_tcp.col(j) = ad_g_rot * _Jbd_tool.col(j); // here is where magic happens 
        Jop.col(j) = _Jh_tcp.col(j);  // populate the public memeber
    }

    // Zero unused columns of public member for safety
    for (int j = _dof; j < MAX_DOF; ++j) {
        Jop.col(j).setZero();
    }

    _is_operational_jacobian_valid = true; // now we set the boolean as true, since Jop is populated

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeHybridJacobianTCP] "
                     "g_last_tcp =\n"
                  << _g_last_tcp.matrix() << std::endl;

        std::cout << "[ScrewsKinematicsNdof::computeHybridJacobianTCP] "
                     "Jb_tcp =\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jbd_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }

        std::cout << "[ScrewsKinematicsNdof::computeHybridJacobianTCP] "
                     "Jh_tcp =\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jh_tcp(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

Eigen::Matrix<float, 6, Eigen::Dynamic> ScrewsKinematicsNdof::getHybridJacobianTCP() const
{
    Eigen::Matrix<float, 6, Eigen::Dynamic> out(6, _dof);

    for (int j = 0; j < _dof; ++j) {
        out.col(j) = _Jh_tcp.col(j);
    }

    return out;
}

Eigen::Matrix<float, 6, Eigen::Dynamic> ScrewsKinematicsNdof::bodyToHybridJacobian(
    const Eigen::Isometry3f& C_frame,
    const Eigen::Matrix<float, 6, Eigen::Dynamic>& Jb) const
{
    // =========================================================================
    // N-DOF implementation of the MATLAB function:
    //   Jh = bodyToHybridJacobian(C_frame, Jb, USE_SYM)
    //
    // Purpose:
    //   Converts the BODY Jacobian of a frame into the HYBRID Jacobian of the
    //   same frame.
    //
    // Convention:
    //   Twists ordered as [v; w]
    //
    // MATLAB form:
    //   R = C_frame(1:3,1:3)
    //   g_rot = eye(4); g_rot(1:3,1:3) = R
    //   Jh = ad(g_rot) * Jb
    //
    // Interpretation:
    //   The hybrid Jacobian expresses:
    //     - linear velocity in the base frame
    //     - angular velocity with the same rotational mapping induced by the
    //       frame orientation
    //
    // Notes on theory:
    //   This is the same body-to-hybrid conversion step used in:
    //     - calculateHybridJacobianTCP(...)
    //     - calculateDtHybridJacobian(...)
    //
    //   In your MATLAB pipeline, this conversion is the frame-level operation
    //   that precedes the use of Mueller Eq. (27) for the analytical time
    //   derivative of the hybrid Jacobian.
    //
    //   So while this helper is not itself the full Eq. (27), it is a direct
    //   building block for that derivation and for the TCP hybrid Jacobian.
    //
    // Preconditions:
    //   1) Jb must be 6 x n
    //   2) C_frame must be a valid SE(3) transform
    // =========================================================================

    if (Jb.rows() != 6) {
        std::cerr << "[ScrewsKinematicsNdof::bodyToHybridJacobian] "
                  << "Invalid body Jacobian row count: "
                  << Jb.rows() << " (expected 6)\n";
        return Eigen::Matrix<float, 6, Eigen::Dynamic>::Zero(6, Jb.cols());
    }

    // Extract only the rotation of the frame
    Eigen::Isometry3f g_rot = Eigen::Isometry3f::Identity();
    g_rot.linear() = C_frame.linear();
    g_rot.translation().setZero();

    // ad(g_rot)
    Eigen::Matrix<float, 6, 6> ad_g_rot;
    const_cast<ScrewsKinematicsNdof*>(this)->ad(ad_g_rot, g_rot);

    Eigen::Matrix<float, 6, Eigen::Dynamic> Jh(6, Jb.cols());
    Jh = ad_g_rot * Jb;

    return Jh;
}

void ScrewsKinematicsNdof::computeSpatialVelocityTwistTCP()
{
    // ========================================================================
    // N-DOF split version of the old combined:
    //   ScrewsKinematicsNdof::VelocityTwistTCP(typ_jacobian::SPATIAL)
    //
    // Computes the spatial velocity twist at the TCP:
    //   V^s_tcp = J^s_tcp(q) * dq
    //
    // Preconditions:
    //   1) _dof > 0
    //   2) joint velocities _joint_vel[0.._dof-1] are up to date
    //   3) spatial Jacobian has already been computed for the current q
    //      (e.g. via computeSpatialJacobianTCP1/2/3())
    // ========================================================================

    _debug_verbosity = false;

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialVelocityTwistTCP] "
                  << "DOF <= 0, aborting.\n";
        _Vsp_twist_tcp.setZero();
        return;
    }

    Eigen::Matrix<float, 6, Eigen::Dynamic> Jsp = getSpatialJacobianTCP();

    if (Jsp.rows() != 6) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialVelocityTwistTCP] "
                  << "Spatial Jacobian has invalid row count: "
                  << Jsp.rows() << " (expected 6)\n";
        _Vsp_twist_tcp.setZero();
        return;
    }

    if (Jsp.cols() != _dof) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialVelocityTwistTCP] "
                  << "Spatial Jacobian has invalid column count: "
                  << Jsp.cols() << " (expected " << _dof << ")\n";
        _Vsp_twist_tcp.setZero();
        return;
    }

    Eigen::Matrix<float, Eigen::Dynamic, 1> dq_vector(_dof);
    for (int i = 0; i < _dof; ++i) {
        dq_vector(i) = _joint_vel[i];
    }

    if (dq_vector.size() != Jsp.cols()) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialVelocityTwistTCP] "
                  << "Dimension mismatch: dq size = " << dq_vector.size()
                  << ", Jsp.cols() = " << Jsp.cols() << "\n";
        _Vsp_twist_tcp.setZero();
        return;
    }

    _Vsp_twist_tcp = Jsp * dq_vector;

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeSpatialVelocityTwistTCP] "
                  << "Spatial Velocity Twist:\n";
        printTwist(_Vsp_twist_tcp);
    }
}

void ScrewsKinematicsNdof::computeBodyVelocityTwistTCP()
{
    // ========================================================================
    // N-DOF split version of the old combined:
    //   ScrewsKinematicsNdof::VelocityTwistTCP(typ_jacobian::BODY)
    //
    // Computes the body velocity twist at the TCP:
    //   V^b_tcp = J^b_tcp(q) * dq
    //
    // Preconditions:
    //   1) _dof > 0
    //   2) joint velocities _joint_vel[0.._dof-1] are up to date
    //   3) body Jacobian has already been computed for the current q
    //      (e.g. via computeBodyJacobianTCP1/2/3())
    // ========================================================================

    _debug_verbosity = false;

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyVelocityTwistTCP] "
                  << "DOF <= 0, aborting.\n";
        _Vbd_twist_tcp.setZero();
        return;
    }

    Eigen::Matrix<float, 6, Eigen::Dynamic> Jbd = getBodyJacobianTCP();

    if (Jbd.rows() != 6) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyVelocityTwistTCP] "
                  << "Body Jacobian has invalid row count: "
                  << Jbd.rows() << " (expected 6)\n";
        _Vbd_twist_tcp.setZero();
        return;
    }

    if (Jbd.cols() != _dof) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyVelocityTwistTCP] "
                  << "Body Jacobian has invalid column count: "
                  << Jbd.cols() << " (expected " << _dof << ")\n";
        _Vbd_twist_tcp.setZero();
        return;
    }

    Eigen::Matrix<float, Eigen::Dynamic, 1> dq_vector(_dof);
    for (int i = 0; i < _dof; ++i) {
        dq_vector(i) = _joint_vel[i];
    }

    if (dq_vector.size() != Jbd.cols()) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyVelocityTwistTCP] "
                  << "Dimension mismatch: dq size = " << dq_vector.size()
                  << ", Jbd.cols() = " << Jbd.cols() << "\n";
        _Vbd_twist_tcp.setZero();
        return;
    }

    _Vbd_twist_tcp = Jbd * dq_vector;

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeBodyVelocityTwistTCP] "
                  << "Body Velocity Twist:\n";
        printTwist(_Vbd_twist_tcp);
    }
}

void ScrewsKinematicsNdof::computeDtSpatialVelocityTwistTCP()
{
    // =========================================================================
    // Implements eq.(18)/p.223 in Mueller-Dynamics paper.
    // N-DOF split version of:
    //   ScrewsKinematics::DtToolVelocityTwist(typ_jacobian::SPATIAL)
    //
    // Original 3-DOF source function:
    //   void ScrewsKinematics::DtToolVelocityTwist(typ_jacobian jacob_selection)
    //   --> case typ_jacobian::SPATIAL
    //
    // Purpose:
    //   Computes the first time derivative of the SPATIAL velocity twist of the
    //   TCP / {T} frame (i.e. the spatial acceleration twist).
    //
    // Mathematical form used in the original 3-DOF code:
    //   dV^s_T = J^s_T(q) * ddq + dJ^s_T(q,dq) * dq
    //
    // but evaluated explicitly as:
    //   Term 1: sum_j J^s_T(:,j) * ddq_j
    //   Term 2: sum_{k<j} [ J^s_T(:,k), J^s_T(:,j) ] * dq_k * dq_j
    //
    // Mapping from old 3-DOF storage to current N-DOF storage:
    //   Jsp_t_1[j]        --> _Jsp_tool.col(j)
    //   dVsp_tool_twist   --> _dVsp_twist_tcp
    //
    // Preconditions:
    //   1) _dof > 0
    //   2) _joint_vel[0.._dof-1] and _joint_accel[0.._dof-1] are up to date
    //   3) spatial Jacobian at TCP has been computed for current q
    //      (e.g. computeSpatialJacobianTCP1/2/3())
    // =========================================================================

    _debug_verbosity = false;
    _dVsp_twist_tcp.setZero();

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtSpatialVelocityTwistTCP] "
                  << "DOF <= 0, aborting.\n";
        return;
    }

    Eigen::Matrix<float, 6, Eigen::Dynamic> Jsp = getSpatialJacobianTCP();

    if (Jsp.rows() != 6) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtSpatialVelocityTwistTCP] "
                  << "Spatial Jacobian row count = " << Jsp.rows()
                  << ", expected 6.\n";
        return;
    }

    if (Jsp.cols() != _dof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtSpatialVelocityTwistTCP] "
                  << "Spatial Jacobian column count = " << Jsp.cols()
                  << ", expected " << _dof << ".\n";
        return;
    }

    Eigen::Matrix<float, 6, 1> dV1 = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 6, 1> dV2 = Eigen::Matrix<float, 6, 1>::Zero();

    // --- Term 1: Σ J_i * ddq_i ---
    for (int j = 0; j < _dof; ++j) {
        dV1 += Jsp.col(j) * _joint_accel[j];
    }

    // --- Term 2: Σ_{k<j} [J_k, J_j] * dq_k * dq_j ---
    for (int k = 0; k < _dof; ++k) {
        for (int j = k + 1; j < _dof; ++j) {
            dV2 += lb(Jsp.col(k), Jsp.col(j)) * _joint_vel[k] * _joint_vel[j];
        }
    }

    _dVsp_twist_tcp = dV1 + dV2;

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeDtSpatialVelocityTwistTCP] "
                  << "Spatial Acceleration Twist:\n";
        printTwist(_dVsp_twist_tcp);
    }
}

void ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP()
{
    // =========================================================================
    // N-DOF split version of:
    //   ScrewsKinematics::DtToolVelocityTwist(typ_jacobian::BODY)
    //
    // Original 3-DOF source function:
    //   void ScrewsKinematics::DtToolVelocityTwist(typ_jacobian jacob_selection)
    //   --> case typ_jacobian::BODY
    //
    // Purpose:
    //   Computes the first time derivative of the BODY velocity twist of the
    //   TCP / {T} frame (i.e. the body acceleration twist).
    //
    // Mathematical form used in the original 3-DOF code:
    //   dV^b_T = J^b_T(q) * ddq + dJ^b_T(q,dq) * dq
    //
    // Mapping from old 3-DOF storage to current N-DOF storage:
    //   Jbd63            --> getBodyJacobianTCP()
    //   dJbd63           --> getDtBodyJacobianTCP()
    //   dVbd_tool_twist  --> _dVbd_twist_tcp
    //
    // Preconditions:
    //   1) _dof > 0
    //   2) _joint_vel[0.._dof-1] and _joint_accel[0.._dof-1] are up to date
    //   3) body Jacobian at TCP has been computed for current q
    //   4) time derivative of body Jacobian at TCP has been computed
    //      (e.g. computeDtBodyJacobianTCP1/2/3())
    // =========================================================================

    _debug_verbosity = false;
    _dVbd_twist_tcp.setZero();

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "DOF <= 0, aborting.\n";
        return;
    }

    Eigen::Matrix<float, 6, Eigen::Dynamic> Jbd  = getBodyJacobianTCP();
    Eigen::Matrix<float, 6, Eigen::Dynamic> dJbd = getDtBodyJacobianTCP();

    if (Jbd.rows() != 6) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "Body Jacobian row count = " << Jbd.rows()
                  << ", expected 6.\n";
        return;
    }

    if (Jbd.cols() != _dof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "Body Jacobian column count = " << Jbd.cols()
                  << ", expected " << _dof << ".\n";
        return;
    }

    if (dJbd.rows() != 6) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "dBody Jacobian row count = " << dJbd.rows()
                  << ", expected 6.\n";
        return;
    }

    if (dJbd.cols() != _dof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "dBody Jacobian column count = " << dJbd.cols()
                  << ", expected " << _dof << ".\n";
        return;
    }

    Eigen::Matrix<float, Eigen::Dynamic, 1> dq_vector(_dof);
    Eigen::Matrix<float, Eigen::Dynamic, 1> ddq_vector(_dof);

    for (int i = 0; i < _dof; ++i) {
        dq_vector(i)  = _joint_vel[i];
        ddq_vector(i) = _joint_accel[i];
    }

    if (dq_vector.size() != Jbd.cols()) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "Dimension mismatch: dq size = " << dq_vector.size()
                  << ", Jbd.cols() = " << Jbd.cols() << ".\n";
        return;
    }

    if (ddq_vector.size() != Jbd.cols()) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "Dimension mismatch: ddq size = " << ddq_vector.size()
                  << ", Jbd.cols() = " << Jbd.cols() << ".\n";
        return;
    }

    if (dq_vector.size() != dJbd.cols()) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "Dimension mismatch: dq size = " << dq_vector.size()
                  << ", dJbd.cols() = " << dJbd.cols() << ".\n";
        return;
    }

    _dVbd_twist_tcp = Jbd * ddq_vector + dJbd * dq_vector;

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeDtBodyVelocityTwistTCP] "
                  << "Body Acceleration Twist:\n";
        printTwist(_dVbd_twist_tcp);
    }
}

void ScrewsKinematicsNdof::computeHybridVelocityTwistTCP()
{
    // =========================================================================
    // N-DOF implementation of the MATLAB function:
    //   [Vh_tcp, v_tcp, w_tcp] = calculateHybridVelTwistTCP(Jh_tcp, qdot)
    //
    // MATLAB meaning:
    //   Calculates TCP hybrid velocity twist with respect to the base frame.
    //
    // Mathematical form:
    //   Vh_tcp = Jh_tcp * qdot
    //   v_tcp  = Vh_tcp(1:3)
    //   w_tcp  = Vh_tcp(4:6)
    //
    // Convention:
    //   Twist ordering [v; w]
    //
    // Inputs in this N-DOF class:
    //   Jh_tcp  --> getHybridJacobianTCP()
    //   qdot    --> _joint_vel[0.._dof-1]
    //
    // Output stored in this class:
    //   Vh_tcp  --> _Vh_twist_tcp
    //
    // Preconditions:
    //   1) _dof > 0
    //   2) _joint_vel[0.._dof-1] are up to date
    //   3) Hybrid Jacobian TCP has already been computed for the current q
    //      (e.g. via computeHybridJacobianTCP())
    //
    // Function assumes code implementation: |-->-->-->-->|
    // ForwardKinematicsTCP(q);
    // computeBodyJacobiansFrames1();   // or 2, depending on your chosen pipeline
    // computeHybridJacobianTCP();
    // computeHybridVelocityTwistTCP();
    //
    // =========================================================================

    _debug_verbosity = true;

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeHybridVelocityTwistTCP] "
                  << "DOF <= 0, aborting.\n";
        _Vh_twist_tcp.setZero();
        return;
    }

    Eigen::Matrix<float, 6, Eigen::Dynamic> Jh = getHybridJacobianTCP();

    if (Jh.rows() != 6) {
        std::cerr << "[ScrewsKinematicsNdof::computeHybridVelocityTwistTCP] "
                  << "Hybrid Jacobian has invalid row count: "
                  << Jh.rows() << " (expected 6)\n";
        _Vh_twist_tcp.setZero();
        return;
    }

    if (Jh.cols() != _dof) {
        std::cerr << "[ScrewsKinematicsNdof::computeHybridVelocityTwistTCP] "
                  << "Hybrid Jacobian has invalid column count: "
                  << Jh.cols() << " (expected " << _dof << ")\n";
        _Vh_twist_tcp.setZero();
        return;
    }

    Eigen::Matrix<float, Eigen::Dynamic, 1> dq_vector(_dof);
    for (int i = 0; i < _dof; ++i) {
        dq_vector(i) = _joint_vel[i];
    }

    if (dq_vector.size() != Jh.cols()) {
        std::cerr << "[ScrewsKinematicsNdof::computeHybridVelocityTwistTCP] "
                  << "Dimension mismatch: dq size = " << dq_vector.size()
                  << ", Jh.cols() = " << Jh.cols() << "\n";
        _Vh_twist_tcp.setZero();
        return;
    }

    _Vh_twist_tcp = Jh * dq_vector;

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeHybridVelocityTwistTCP] "
                  << "Hybrid Velocity Twist:\n";
        printTwist(_Vh_twist_tcp);

        std::cout << "[ScrewsKinematicsNdof::computeHybridVelocityTwistTCP] "
                  << "v_tcp = ["
                  << _Vh_twist_tcp(0) << ", "
                  << _Vh_twist_tcp(1) << ", "
                  << _Vh_twist_tcp(2) << "], w_tcp = ["
                  << _Vh_twist_tcp(3) << ", "
                  << _Vh_twist_tcp(4) << ", "
                  << _Vh_twist_tcp(5) << "]\n";
    }
}

const Eigen::Matrix<float, 6, 1>& ScrewsKinematicsNdof::getHybridVelocityTwistTCP() const
{
    return _Vh_twist_tcp;
}

Eigen::Vector3f ScrewsKinematicsNdof::getHybridLinearVelocityTCP() const
{
    return _Vh_twist_tcp.block<3,1>(0,0);
}

Eigen::Vector3f ScrewsKinematicsNdof::getHybridAngularVelocityTCP() const
{
    return _Vh_twist_tcp.block<3,1>(3,0);
}

void ScrewsKinematicsNdof::computeDtHybridVelocityTwistTCP()
{
    // =========================================================================
    // N-DOF implementation of the MATLAB function:
    //   calculateDtHybridVelocityTwistEq29(...)
    //
    // This C++ version intentionally OMITS recalculation of quantities that are
    // assumed to be already available from previously called functions.
    //
    // Assumed already computed before calling this function:
    //   1) ForwardKinematicsTCP(...)
    //      -> _g[0.._dof] are up to date
    //   2) Body Jacobians for actual bodies / frames
    //      -> _BodyJacobiansFrames[frame][joint]
    //   3) TCP hybrid Jacobian
    //      -> _Jh_tcp
    //   4) TCP hybrid velocity twist
    //      -> _Vh_twist_tcp
    //
    // Theory:
    //   Mueller Eq. (29):
    //
    //     Vdot_i^h = sum_j ( J_{i,j}^h * qddot_j
    //                      + [ J_{i,j}^h , ^v(Vtilde_{j-1,i}^h) ] * qdot_j )
    //
    //   with
    //
    //     Vtilde_{j-1,i}^h = V_i^h - Ad_{r_{i,j-1}} * V_{j-1}^h
    //
    // Convention:
    //   twists ordered as [v; w]
    //
    // TCP-specific mapping:
    //   i -> tcp
    //   V_i^h       -> _Vh_twist_tcp
    //   J_{i,j}^h   -> _Jh_tcp.col(j)
    //
    // For body (j-1), we compute only what is still needed:
    //   - Jh_prev    = bodyToHybridJacobian( C_prev, Jb_prev )
    //   - Vh_prev    = Jh_prev * qdot
    //   - Vtilde     = Vh_tcp - Ad_{r_{tcp,j-1}} * Vh_prev
    //
    // Then Eq. (29) is applied directly column-by-column.
    //
    // Outputs stored in this class:
    //   _dVh_twist_tcp  : hybrid acceleration twist of TCP [a_tcp; alpha_tcp]
    //
    // Function assumes code implementation: |-->-->-->-->|
    // kin.ForwardKinematicsTCP(q.data());
    // kin.computeBodyJacobiansFrames1();      // or 2, as long as storage is updated
    // kin.computeHybridJacobianTCP();
    // kin.computeHybridVelocityTwistTCP();
    // kin.computeDtHybridVelocityTwistTCP();    
    // =========================================================================

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtHybridVelocityTwistTCP] "
                  << "DOF <= 0\n";
        _dVh_twist_tcp.setZero();
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtHybridVelocityTwistTCP] "
                  << "RobotAbstractBaseNdof pointer is null\n";
        _dVh_twist_tcp.setZero();
        return;
    }

    _debug_verbosity = true;
    _dVh_twist_tcp.setZero();

    // -------------------------------------------------------------------------
    // Build qdot and qddot vectors from stored joint state
    // -------------------------------------------------------------------------
    Eigen::Matrix<float, Eigen::Dynamic, 1> qdot(_dof);
    Eigen::Matrix<float, Eigen::Dynamic, 1> qddot(_dof);

    for (int i = 0; i < _dof; ++i) {
        qdot(i)  = _joint_vel[i];
        qddot(i) = _joint_accel[i];
    }

    if (qdot.size() != _dof || qddot.size() != _dof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtHybridVelocityTwistTCP] "
                  << "Invalid qdot/qddot size\n";
        _dVh_twist_tcp.setZero();
        return;
    }

    // -------------------------------------------------------------------------
    // Basic consistency checks for already-computed TCP hybrid Jacobian/twist
    // -------------------------------------------------------------------------
    if (_Jh_tcp.rows() != 6) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtHybridVelocityTwistTCP] "
                  << "Hybrid Jacobian row count invalid: "
                  << _Jh_tcp.rows() << " (expected 6)\n";
        _dVh_twist_tcp.setZero();
        return;
    }

    const Eigen::Isometry3f& C_tcp = _g[_dof];
    const Eigen::Vector3f r_tcp = C_tcp.translation();

    // -------------------------------------------------------------------------
    // Direct implementation of Mueller Eq. (29)
    // -------------------------------------------------------------------------
    for (int j = 0; j < _dof; ++j)
    {
        Eigen::Matrix<float, 6, 1> Vh_prev_at_tcp = Eigen::Matrix<float, 6, 1>::Zero();

        // ---------------------------------------------------------------------
        // For j > 0, compute the hybrid twist of actual body (j-1)
        // ---------------------------------------------------------------------
        if (j > 0)
        {
            const int prev_body_index = j - 1;

            // Build Jb_prev from stored frame-wise body Jacobian table
            Eigen::Matrix<float, 6, Eigen::Dynamic> Jb_prev(6, _dof);
            for (int col = 0; col < _dof; ++col) {
                Jb_prev.col(col) = _BodyJacobiansFrames[prev_body_index][col];
            }

            // Convert body Jacobian of body (j-1) to hybrid Jacobian
            Eigen::Matrix<float, 6, Eigen::Dynamic> Jh_prev =
                bodyToHybridJacobian(_g[prev_body_index], Jb_prev);

            if (Jh_prev.rows() != 6 || Jh_prev.cols() != _dof) {
                std::cerr << "[ScrewsKinematicsNdof::computeDtHybridVelocityTwistTCP] "
                          << "Invalid Jh_prev dimensions for body index "
                          << prev_body_index << "\n";
                _dVh_twist_tcp.setZero();
                return;
            }

            // Hybrid twist of body (j-1)
            Eigen::Matrix<float, 6, 1> Vh_prev = Jh_prev * qdot;

            // Vector from TCP origin to origin of body (j-1), base-resolved
            const Eigen::Vector3f r_prev = _g[prev_body_index].translation();
            const Eigen::Vector3f r_tcp_prev = r_prev - r_tcp;

            Eigen::Isometry3f g_r = Eigen::Isometry3f::Identity();
            g_r.translation() = r_tcp_prev;

            Eigen::Matrix<float, 6, 6> ad_g_r;
            ad(ad_g_r, g_r);

            // Ad_{r_{tcp,j-1}} * Vh_prev
            Vh_prev_at_tcp = ad_g_r * Vh_prev;
        }

        // ---------------------------------------------------------------------
        // Vtilde_{j-1,tcp}
        // ---------------------------------------------------------------------
        Eigen::Matrix<float, 6, 1> Vtilde = _Vh_twist_tcp - Vh_prev_at_tcp;

        // Keep only translational part: ^v Vtilde
        Eigen::Matrix<float, 6, 1> Vtilde_v = Eigen::Matrix<float, 6, 1>::Zero();
        Vtilde_v.block<3,1>(0,0) = Vtilde.block<3,1>(0,0);

        // Eq. (29) term for column j:
        //   Jh(:,j) * qddot(j) + ad_twist_vw(Jh(:,j)) * Vtilde_v * qdot(j)
        Eigen::Matrix<float, 6, 6> adJh;
        adTwistVW(adJh, _Jh_tcp.col(j));

        _dVh_twist_tcp += _Jh_tcp.col(j) * qddot(j)
                        + adJh * Vtilde_v * qdot(j);
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeDtHybridVelocityTwistTCP] "
                  << "Hybrid Acceleration Twist:\n";
        printTwist(_dVh_twist_tcp);

        std::cout << "[ScrewsKinematicsNdof::computeDtHybridVelocityTwistTCP] "
                  << "a_tcp = ["
                  << _dVh_twist_tcp(0) << ", "
                  << _dVh_twist_tcp(1) << ", "
                  << _dVh_twist_tcp(2) << "], alpha_tcp = ["
                  << _dVh_twist_tcp(3) << ", "
                  << _dVh_twist_tcp(4) << ", "
                  << _dVh_twist_tcp(5) << "]\n";
    }
}

const Eigen::Matrix<float, 6, 1>& ScrewsKinematicsNdof::getDtHybridVelocityTwistTCP() const
{
    return _dVh_twist_tcp;
}

Eigen::Vector3f ScrewsKinematicsNdof::getHybridLinearAccelerationTCP() const
{
    return _dVh_twist_tcp.block<3,1>(0,0);
}

Eigen::Vector3f ScrewsKinematicsNdof::getHybridAngularAccelerationTCP() const
{
    return _dVh_twist_tcp.block<3,1>(3,0);
}

void ScrewsKinematicsNdof::computeDtSpatialJacobianTCP1()
{
    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtSpatialJacobianTCP1] DOF <= 0\n";
        return;
    }

    _debug_verbosity = false;

    // Clear all columns first
    for (int j = 0; j < _dof; ++j) {
        _dJsp_tool.col(j).setZero();
    }

    // Implements the first "=" in eq.(17)
    for (int j = 0; j < _dof; ++j) {

        Eigen::Matrix<float, 6, 1> dJ = Eigen::Matrix<float, 6, 1>::Zero();

        for (int k = 0; k < j; ++k) {
            dJ += lb(_Jsp_tool.col(k), _Jsp_tool.col(j)) * _joint_vel[k];
        }

        _dJsp_tool.col(j) = dJ;
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeDtSpatialJacobianTCP1] dJsp_tool =\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _dJsp_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

Eigen::Matrix<float, 6, Eigen::Dynamic> ScrewsKinematicsNdof::getDtSpatialJacobianTCP() const
{
    Eigen::Matrix<float, 6, Eigen::Dynamic> out(6, _dof);

    for (int j = 0; j < _dof; ++j) {
        out.col(j) = _dJsp_tool.col(j);
    }

    return out;
}

void ScrewsKinematicsNdof::computeDtBodyJacobianTCP1()
{
    // ================================================================
    // N-DOF version of:
    //   ScrewsKinematics::DtBodyJacobian_Tool_1()
    //
    // Implements:
    //   First "=" in eq.(8)/p.223/[3]
    //   Time derivative of the BODY Jacobian at the TCP frame {T}
    //
    // Original 3-DOF logic:
    //   dJ_j = sum_{k=j+1}^{DOF-1} [J_j, J_k] * dq[k]
    //
    // Mapping:
    //   ptr2BodyJacobiansFrames[DOF][j]  -->  _Jbd_tool.col(j)
    //   ptr2dJbd_t_1[j]                  -->  _dJbd_tool.col(j)
    // ================================================================

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyJacobianTCP1] DOF <= 0\n";
        return;
    }

    _debug_verbosity = false;

    // Make sure the body Jacobian at TCP is up to date
    computeBodyJacobianTCP1();

    for (int j = 0; j < _dof; ++j) {

        Eigen::Matrix<float, 6, 1> dJ = Eigen::Matrix<float, 6, 1>::Zero();

        for (int k = j + 1; k < _dof; ++k) {
            dJ += lb(_Jbd_tool.col(j), _Jbd_tool.col(k)) * _joint_vel[k];
        }

        _dJbd_tool.col(j) = dJ;
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeDtBodyJacobianTCP1] dJbd_tool =\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _dJbd_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

void ScrewsKinematicsNdof::computeDtBodyJacobianTCP2()
{
    // ========================================================================
    // N-DOF version of:
    //   ScrewsKinematics::DtBodyJacobian_Tool_1()
    //
    // Original 3-DOF function:
    //   void ScrewsKinematics::DtBodyJacobian_Tool_1()
    //
    // Original 3-DOF purpose:
    //   Implements the first "=" in eq.(8)/p.223/[3]
    //   Outputs the time derivative of the BODY Jacobian at the tool / {T}
    //
    // Mathematical form implemented:
    //   dJ^b_T(:,j) = sum_{k=j+1}^{n-1} [ J^b_T(:,j), J^b_T(:,k) ] * dq_k
    //
    // IMPORTANT STORAGE CHOICE:
    //   This N-DOF version follows the SAME philosophy as the 3-DOF code:
    //   it uses the body Jacobian columns stored in:
    //       _BodyJacobiansFrames[_dof][j]
    //   where frame index _dof corresponds to the TCP / tool frame {T}.
    //
    // Mapping from 3-DOF code to N-DOF code:
    //   ptr2BodyJacobiansFrames[robot_params::DOF][j]
    //       --> _ptr2BodyJacobiansFrames[_dof][j]
    //
    //   *ptr2dJbd_t_1[j]
    //       --> _dJbd_tool.col(j)
    //
    // Preconditions:
    //   1) ForwardKinematicsTCP(...) has been called for the current q
    //      so that _g[0.._dof] are up to date
    //   2) computeBodyJacobiansFrames2() has been called, OR this function
    //      calls it directly to refresh _BodyJacobiansFrames
    //   3) _joint_vel[] contains the current dq values
    //
    // NOTE:
    //   We use computeBodyJacobiansFrames2() here so that this derivative
    //   is consistent with your "2" body Jacobian implementation.
    // ========================================================================

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyJacobianTCP2] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyJacobianTCP2] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    _debug_verbosity = false;

    // Refresh body Jacobians for all frames.
    // We specifically use version "2", but also "1" could be used, since both write to the same meomry address.
    computeBodyJacobiansFrames2();

    // Tool/TCP frame is stored at frame index _dof
    const int tcp_frame_index = _dof;

    for (int j = 0; j < _dof; ++j)
    {
        Eigen::Matrix<float, 6, 1> dJ = Eigen::Matrix<float, 6, 1>::Zero();

        for (int k = j + 1; k < _dof; ++k)
        {
            dJ += lb(*_ptr2BodyJacobiansFrames[tcp_frame_index][j],
                     *_ptr2BodyJacobiansFrames[tcp_frame_index][k]) * _joint_vel[k];
        }

        _dJbd_tool.col(j) = dJ;
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeDtBodyJacobianTCP2] "
                     "Time Derivative Body Jacobian Tool 2:\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _dJbd_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }

    return;
}

void ScrewsKinematicsNdof::computeDtBodyJacobianTCP3()
{
    // ========================================================================
    // N-DOF version of:
    //   ScrewsKinematics::DtBodyJacobian_Tool_2()
    //
    // Original 3-DOF function:
    //   void ScrewsKinematics::DtBodyJacobian_Tool_2()
    //
    // Original 3-DOF purpose:
    //   Implements the second "=" in eq.(8)/p.223/[3]
    //   Outputs the time derivative of the BODY Jacobian at the tool / {T}
    //
    // Original 3-DOF code logic:
    //   for each column j:
    //     dJ_j = sum_{k=j+1}^{DOF-1}
    //              ( - Ad_{ g_T^{-1} g_k } * ad(iXi_k) * J^b_k(:,j) ) * dq_k
    //
    // where:
    //   g_T = g[robot_params::DOF]
    //   J^b_k(:,j) = ptr2BodyJacobiansFrames[k][j]
    //
    // Mapping from 3-DOF code to N-DOF code:
    //   g[robot_params::DOF]                 --> _g[_dof]
    //   g[k]                                --> _g[k]
    //   iXi[k]                              --> _iXi[k]
    //   ptr2BodyJacobiansFrames[k][j]       --> _ptr2BodyJacobiansFrames[k][j]
    //   ptr2dJbd_t_2[j]                     --> _dJbd_tool.col(j)
    //
    //
    // IMPORTANT:
    //   This function uses the frame-wise body Jacobians stored in:
    //       _BodyJacobiansFrames[k][j]
    //   exactly like the original 3-DOF implementation.
    //
    // Preconditions:
    //   1) ForwardKinematicsTCP(...) has been called for the current q
    //      so that _g[0.._dof] are up to date
    //   2) computeBodyJacobiansFrames1() has been called, OR this function
    //      calls it directly to refresh _BodyJacobiansFrames
    //   3) initializeLocalScrewCoordVectors() has been called so that _iXi[] exist
    //   4) _joint_vel[] contains the current dq values
    // ========================================================================

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyJacobianTCP3] DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtBodyJacobianTCP3] "
                     "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    _debug_verbosity = false;

    // Refresh body Jacobians for all frames.
    // This follows the original 3-DOF formula, which uses iXi[k],
    // so we pair it with computeBodyJacobiansFrames1(). Again "2"
    // could be used with no problem
    computeBodyJacobiansFrames1();

    const int tcp_frame_index = _dof;
    Eigen::Matrix<float, 6, 1> Adad3;

    for (int j = 0; j < _dof; ++j)
    {
        Eigen::Matrix<float, 6, 1> dJ = Eigen::Matrix<float, 6, 1>::Zero();

        for (int k = j + 1; k < _dof; ++k)
        {
            // Ad_{ g_T^{-1} g_k }
            ad(_ad, _g[tcp_frame_index].inverse() * _g[k]);

            // ad(iXi_k)
            spatialCrossProduct(_scp, _iXi[k]);

            // - Ad_{ g_T^{-1} g_k } * ad(iXi_k) * J^b_k(:,j)
            Adad3 = -_ad * _scp * (*_ptr2BodyJacobiansFrames[k][j]);

            dJ += Adad3 * _joint_vel[k];
        }

        _dJbd_tool.col(j) = dJ;
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeDtBodyJacobianTCP3] "
                     "Time Derivative Body Jacobian Tool 3:\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _dJbd_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }

    return;
}

Eigen::Matrix<float, 6, Eigen::Dynamic> ScrewsKinematicsNdof::getDtBodyJacobianTCP() const
{
    Eigen::Matrix<float, 6, Eigen::Dynamic> out(6, _dof);

    for (int j = 0; j < _dof; ++j) {
        out.col(j) = _dJbd_tool.col(j);
    }

    return out;
}

void ScrewsKinematicsNdof::computeDtHybridJacobianTCP()
{
    // =========================================================================
    // N-DOF implementation of the MATLAB function:
    //   calculateDtHybridJacobian(...)
    //
    // Here we intentionally OMIT recalculation of quantities that are assumed
    // to be already available in the class.
    //
    // Assumed already computed before calling this function:
    //   1) ForwardKinematicsTCP(...) has been called
    //      -> _g[0.._dof] are up to date
    //   2) Body Jacobians for joint frames + TCP have been computed
    //      -> _BodyJacobiansFrames[frame][joint]
    //   3) TCP hybrid Jacobian has already been computed
    //      -> _Jh_tcp
    //   4) TCP hybrid velocity twist has already been computed
    //      -> _Vh_twist_tcp
    //
    // MATLAB quantities omitted here because they are assumed already known:
    //   Jh_tcp  -> _Jh_tcp
    //   Vh_tcp  -> _Vh_twist_tcp
    //   Jb_tcp  -> already available elsewhere if needed
    //
    // Theory:
    //   Mueller Eq. (27):
    //
    //     Jdot^h_{i,j} = [ J^h_{i,j},  ^v(Vtilde^h_{j-1,i}) ]
    //
    //   with
    //
    //     Vtilde^h_{j-1,i} = V^h_i - Ad_{r_{i,j-1}} V^h_{j-1}
    //
    // Convention:
    //   twists ordered as [v; w]
    //
    // In this TCP-specific implementation:
    //   i   -> tcp
    //   V^h_i -> _Vh_twist_tcp
    //   J^h_{i,j} -> _Jh_tcp.col(j)
    //
    // For body (j-1), we compute:
    //   Jh_body_(j-1) = bodyToHybridJacobian( C_(j-1), Jb_(j-1) )
    //   Vh_body_(j-1) = Jh_body_(j-1) * qdot
    //
    // Then:
    //   Vtilde = Vh_tcp - Ad_{r_{tcp,j-1}} * Vh_body_(j-1)
    //
    // and only the translational part enters:
    //   Vtilde_v = [ Vtilde(1:3) ; 0 ; 0 ; 0 ]
    //
    // Finally:
    //   dJh_tcp.col(j) = ad_twist_vw( Jh_tcp.col(j) ) * Vtilde_v
    //
    // Function assumes code implementation: |-->-->-->-->|
    // kin.ForwardKinematicsTCP(q.data());
    // kin.computeBodyJacobiansFrames1();      // or 2, as long as storage is updated
    // kin.computeHybridJacobianTCP();
    // kin.computeHybridVelocityTwistTCP();
    // kin.computeDtHybridJacobianTCP();
    //
    // IMPORTANT NOTE:
    // Right now it recomputes jh_prev and Vh_prev for each iteration step.
    // =========================================================================

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtHybridJacobianTCP] "
                  << "DOF <= 0\n";
        return;
    }

    if (!_ptr2abstract_ndof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtHybridJacobianTCP] "
                  << "RobotAbstractBaseNdof pointer is null\n";
        return;
    }

    _debug_verbosity = false;
    _dJh_tcp.setZero();

    // -------------------------------------------------------------------------
    // Build qdot vector from stored joint velocities
    // -------------------------------------------------------------------------
    Eigen::Matrix<float, Eigen::Dynamic, 1> qdot(_dof);
    for (int i = 0; i < _dof; ++i) {
        qdot(i) = _joint_vel[i];
    }

    if (qdot.size() != _dof) {
        std::cerr << "[ScrewsKinematicsNdof::computeDtHybridJacobianTCP] "
                  << "Invalid qdot size\n";
        return;
    }

    // -------------------------------------------------------------------------
    // TCP transform and origin in base frame
    // -------------------------------------------------------------------------
    const Eigen::Isometry3f& C_tcp = _g[_dof];
    const Eigen::Vector3f r_tcp = C_tcp.translation();

    // -------------------------------------------------------------------------
    // For each column j, apply Mueller Eq. (27)
    // -------------------------------------------------------------------------
    for (int j = 0; j < _dof; ++j)
    {
        Eigen::Matrix<float, 6, 1> Vh_prev_at_tcp = Eigen::Matrix<float, 6, 1>::Zero();

        // ---------------------------------------------------------------------
        // If j > 0, compute hybrid twist of actual body (j-1)
        // ---------------------------------------------------------------------
        if (j > 0)
        {
            const int prev_body_index = j - 1;

            // Build Jb_prev from stored frame-wise body Jacobian table
            Eigen::Matrix<float, 6, Eigen::Dynamic> Jb_prev(6, _dof);
            for (int col = 0; col < _dof; ++col) {
                Jb_prev.col(col) = _BodyJacobiansFrames[prev_body_index][col];
            }

            // Convert body Jacobian of body (j-1) to hybrid Jacobian
            Eigen::Matrix<float, 6, Eigen::Dynamic> Jh_prev =
                bodyToHybridJacobian(_g[prev_body_index], Jb_prev);

            if (Jh_prev.rows() != 6 || Jh_prev.cols() != _dof) {
                std::cerr << "[ScrewsKinematicsNdof::computeDtHybridJacobianTCP] "
                          << "Invalid Jh_prev dimensions for body index "
                          << prev_body_index << "\n";
                return;
            }

            // Hybrid twist of body (j-1)
            Eigen::Matrix<float, 6, 1> Vh_prev = Jh_prev * qdot;

            // Vector from TCP origin to origin of body (j-1), expressed in base frame
            const Eigen::Vector3f r_prev = _g[prev_body_index].translation();
            const Eigen::Vector3f r_tcp_prev = r_prev - r_tcp;

            Eigen::Isometry3f g_r = Eigen::Isometry3f::Identity();
            g_r.translation() = r_tcp_prev;

            Eigen::Matrix<float, 6, 6> ad_g_r;
            ad(ad_g_r, g_r);

            // Ad_{r_{tcp,j-1}} * Vh_{j-1}
            Vh_prev_at_tcp = ad_g_r * Vh_prev;
        }

        // ---------------------------------------------------------------------
        // Vtilde = Vh_tcp - Vh_prev_at_tcp
        // ---------------------------------------------------------------------
        Eigen::Matrix<float, 6, 1> Vtilde = _Vh_twist_tcp - Vh_prev_at_tcp;

        // Only translational part enters as ^v(Vtilde)
        Eigen::Matrix<float, 6, 1> Vtilde_v = Eigen::Matrix<float, 6, 1>::Zero();
        Vtilde_v.block<3,1>(0,0) = Vtilde.block<3,1>(0,0);

        // Eq. (27):
        //   dJh(:,j) = ad_twist_vw( Jh(:,j) ) * Vtilde_v
        Eigen::Matrix<float, 6, 6> adJh;
        adTwistVW(adJh, _Jh_tcp.col(j));

        _dJh_tcp.col(j) = adJh * Vtilde_v;
    }

    if (_debug_verbosity) {
        std::cout << "[ScrewsKinematicsNdof::computeDtHybridJacobianTCP] "
                  << "dJh_tcp =\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _dJh_tcp(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

Eigen::Matrix<float, 6, Eigen::Dynamic> ScrewsKinematicsNdof::getDtHybridJacobianTCP() const
{
    Eigen::Matrix<float, 6, Eigen::Dynamic> out(6, _dof);

    for (int j = 0; j < _dof; ++j) {
        out.col(j) = _dJh_tcp.col(j);
    }

    return out;
}

// Example usage in node:
//try {
//    Eigen::Matrix<float, 6, Eigen::Dynamic> Jop = kin.getOperationalJacobianTCP();
//    // use Jop
//}
//catch (const std::runtime_error& e) {
//    std::cerr << "Error: " << e.what() << std::endl;
//}
// and the respective msg shown if no jacobian is computed:
// >> Error: Operational Jacobian not computed
Eigen::Matrix<float, 6, Eigen::Dynamic> ScrewsKinematicsNdof::getOperationalJacobianTCP() const
{
    // With exception
    if (!_is_operational_jacobian_valid) {
        throw std::runtime_error("Operational Jacobian not computed");
    }
    // No exception. Remode on [4.4.26]
    //if (!_is_operational_jacobian_valid) {
    //    std::cerr << "[ScrewsKinematicsNdof::getOperationalJacobianTCP] "
    //              << "Operational Jacobian not computed. "
    //             << "Call computeHybridJacobianTCP() first.\n";
    //    return Eigen::Matrix<float, 6, Eigen::Dynamic>::Zero(6, _dof);
    //}

    if (_dof <= 0 || _dof > MAX_DOF) {
        std::cerr << "[ScrewsKinematicsNdof::getOperationalJacobianTCP] "
                  << "Invalid DOF = " << _dof << "\n";
        return Eigen::Matrix<float, 6, Eigen::Dynamic>::Zero(6, 0);
    }

    Eigen::Matrix<float, 6, Eigen::Dynamic> out(6, _dof);
    for (int j = 0; j < _dof; ++j) {
        out.col(j) = Jop.col(j);
    }

    return out;
}

// ============================================================
// Protected members
// ============================================================

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsKinematicsNdof::stackBodyJacobiansFrames() const
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jb(6 * _dof, _dof);
    Jb.setZero();

    for (int i = 0; i < _dof; ++i) {
        for (int j = 0; j < _dof; ++j) {
            Jb.block<6,1>(6 * i, j) = _BodyJacobiansFrames[i][j];
        }
    }

    return Jb;
}

Eigen::Matrix<float, Eigen::Dynamic, 1>
ScrewsKinematicsNdof::stackBodyTwistsFrames() const
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jb = stackBodyJacobiansFrames();

    Eigen::Matrix<float, Eigen::Dynamic, 1> qdot(_dof);
    for (int i = 0; i < _dof; ++i) {
        qdot(i) = _joint_vel[i];
    }

    return Jb * qdot;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsKinematicsNdof::computeAbMatrix() const
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Ab(6 * _dof, 6 * _dof);
    Ab.setZero();

    Eigen::Matrix<float, 6, 6> Ad_block;

    for (int i = 0; i < _dof; ++i) {
        for (int j = 0; j < i; ++j) {
            // A^b_ij = Ad_{ g_i^{-1} g_j }
            const Eigen::Isometry3f gij = _g[i].inverse() * _g[j];
            const_cast<ScrewsKinematicsNdof*>(this)->ad(Ad_block, gij);
            Ab.block<6,6>(6 * i, 6 * j) = Ad_block;
        }
    }

    return Ab;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsKinematicsNdof::computeabMatrix() const
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> ab(6 * _dof, 6 * _dof);
    ab.setZero();

    Eigen::Matrix<float, 6, 6> adXi;

    for (int i = 0; i < _dof; ++i) {
        const_cast<ScrewsKinematicsNdof*>(this)->spatialCrossProduct(adXi, _iXi[i]);
        ab.block<6,6>(6 * i, 6 * i) = _joint_vel[i] * adXi;
    }

    return ab;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
ScrewsKinematicsNdof::computebbMatrix() const
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> bb(6 * _dof, 6 * _dof);
    bb.setZero();

    Eigen::Matrix<float, Eigen::Dynamic, 1> Vb = stackBodyTwistsFrames();
    Eigen::Matrix<float, 6, 6> adV;

    for (int i = 0; i < _dof; ++i) {
        Eigen::Matrix<float, 6, 1> Vi = Vb.block<6,1>(6 * i, 0);
        const_cast<ScrewsKinematicsNdof*>(this)->spatialCrossProduct(adV, Vi);
        bb.block<6,6>(6 * i, 6 * i) = adV;
    }

    return bb;
}

// ============================================================
// Private members
// ============================================================

void ScrewsKinematicsNdof::printTwist(Eigen::Matrix<float, 6, 1> twist) {
    for (size_t i = 0; i < 6; i++) {
        std::cout << twist[i] << std::endl;
    }
    return;
}

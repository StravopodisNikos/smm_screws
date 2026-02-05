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

void ScrewsKinematicsNdof::computeSpatialJacobianTCP()
{
    // Precondition: _g[0.._dof-1], _iXi[0.._dof-1] are up-to-date
    // Typically:
    //   - initializeLocalScrewCoordVectors() once at startup
    //   - ForwardKinematicsTCP(...) for current q before calling this

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeSpatialJacobianTCP] DOF <= 0\n";
        return;
    }

    for (int i = 0; i < _dof; ++i) {
        ad(_ad, _g[i]);                       // Ad_{g_i}
        _Jsp_tool.col(i) = _ad * _iXi[i];     // J^s_i = Ad_{g_i} * iXi_i
    }

    if (_debug_verbosity) {
        std::cout << "[computeSpatialJacobianTCP] Jsp_tool (6 x " << _dof << "):\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jsp_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

void ScrewsKinematicsNdof::computeBodyJacobianTCP()
{
    // Precondition: ForwardKinematicsTCP was called (so _g[_dof] is TCP pose)
    // Body Jacobian for TCP: J^b_i = Ad_{g_T^{-1} g_i} * iXi_i

    if (_dof <= 0) {
        std::cerr << "[ScrewsKinematicsNdof::computeBodyJacobianTCP] DOF <= 0\n";
        return;
    }

    Eigen::Isometry3f gT_inv = _g[_dof].inverse();

    for (int i = 0; i < _dof; ++i) {
        ad(_ad, gT_inv * _g[i]);              // Ad_{g_T^{-1} g_i}
        _Jbd_tool.col(i) = _ad * _iXi[i];     // J^b_i = Ad_{g_T^{-1} g_i} * iXi_i
    }

    if (_debug_verbosity) {
        std::cout << "[computeBodyJacobianTCP] Jbd_tool (6 x " << _dof << "):\n";
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < _dof; ++c) {
                std::cout << _Jbd_tool(r, c) << "\t";
            }
            std::cout << "\n";
        }
    }
}

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

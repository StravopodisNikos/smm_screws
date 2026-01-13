#ifndef SCREWS_KINEMATICS_NDOF_H
#define SCREWS_KINEMATICS_NDOF_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#include "smm_screws/robot_parameters.h"
#include "smm_screws/core/ScrewsMain.h"
#include "smm_screws/core/RobotAbstractBaseNdof.h"

class ScrewsKinematicsNdof : public ScrewsMain {
public:
    // Alias to global robot parameters so everything stays consistent.
    static constexpr int MAX_DOF = robot_params::MAX_DOF;

    explicit ScrewsKinematicsNdof(RobotAbstractBaseNdof* ptr2abstract_ndof)
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

        // Initialize joint state arrays and exponentials
        for (int i = 0; i < MAX_DOF; ++i) {
            _joint_pos[i]   = 0.0f;
            _joint_vel[i]   = 0.0f;
            _joint_accel[i] = 0.0f;
            _active_expos[i].setIdentity();
            _active_expos_anat[i].setIdentity();
        }
    }

    int dof() const noexcept { return _dof; }

    // We will extend the public API step by step (updateJointState, FK, etc.)
    void setExponentials(float* q);
    void setExponentialsAnat(float* q);

private:
    RobotAbstractBaseNdof* _ptr2abstract_ndof {nullptr};
    int _dof {0};

    // Ndof equivalents of the 3-DOF private members
    Eigen::Isometry3f _active_expos[MAX_DOF];       // exp(ξ_ref_i * q_i)
    Eigen::Isometry3f _active_expos_anat[MAX_DOF];  // exp(ξ_anat_i * q_i)

    float _joint_pos[MAX_DOF];
    float _joint_vel[MAX_DOF];
    float _joint_accel[MAX_DOF];

    // Later we will add more Ndof data: _gst_ndof, _Pi_ndof, Jacobians, etc.
};

#endif // SCREWS_KINEMATICS_NDOF_H

#ifndef ROBOT_ABSTRACT_BASE_NDOF_H
#define ROBOT_ABSTRACT_BASE_NDOF_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <smm_screws/core/RobotYamlLoaderNdof.h>

class RobotAbstractBaseNdof {
public:
    RobotAbstractBaseNdof() = default;

    explicit RobotAbstractBaseNdof(const std::string & base_yaml_path)
    {
        yaml_loader.setBasePath(base_yaml_path);
    }

    // DOF actually loaded from YAML (3..6)
    int get_DOF() const { return dof_; }

    // --------------------------------------------------------------------
    // Storage for active twists, frames, COMs, inertias (compatibility with
    // existing ScrewsKinematics / ScrewsDynamics that expect arrays).
    // ONLY the first dof_ entries are valid; the rest are zero/identity.
    // --------------------------------------------------------------------

    // Active twists (reference/test anatomy)
    Eigen::Matrix<float, 6, 1>  active_twists[robot_params::MAX_DOF];         // legacy, kept as zero
    Eigen::Matrix<float, 6, 1>* ptr2_active_twists[robot_params::MAX_DOF];

    Eigen::Matrix<float, 6, 1>  active_twists_anat[robot_params::MAX_DOF];    // reference anatomy
    Eigen::Matrix<float, 6, 1>* ptr2_active_twists_anat[robot_params::MAX_DOF];

    // Joint frames and TCP
    Eigen::Isometry3f g_ref_0[robot_params::MAX_DOF + 1];   // reference anatomy (currently identity)
    Eigen::Isometry3f g_test_0[robot_params::MAX_DOF + 1];  // world -> joint_i, world -> tcp

    Eigen::Isometry3f gl_test_0[robot_params::MAX_DOF];     // link COM frames (test anatomy)

    Eigen::Isometry3f* gsai_ptr[robot_params::MAX_DOF + 1];       // reference anatomy ptrs
    Eigen::Isometry3f* gsai_test_ptr[robot_params::MAX_DOF + 1];  // test anatomy ptrs
    Eigen::Isometry3f* gsli_test_ptr[robot_params::MAX_DOF];      // COM ptrs

    // Spatial inertias and scalars
    float* link_mass[robot_params::MAX_DOF];
    float* link_inertia[robot_params::MAX_DOF];
    Eigen::Matrix<float, 6, 6>  Mi_s[robot_params::MAX_DOF];
    Eigen::Matrix<float, 6, 6>* Mi_s_ptr[robot_params::MAX_DOF];

    float* fc_coeffs[robot_params::MAX_DOF];
    float* fv_coeffs[robot_params::MAX_DOF];

    // Shared loader (Ndof version)
    RobotYamlLoaderNdof yaml_loader;

    // --------------------------------------------------------------------
    // Initialize from YAML (Ndof)
    // --------------------------------------------------------------------
    bool initializeFromYaml(const std::string & base_yaml_path)
    {
        yaml_loader.setBasePath(base_yaml_path);

        if (!yaml_loader.loadAll()) {
            std::cerr << "[RobotAbstractBaseNdof::initializeFromYaml] Failed to load robot data from YAML."
                      << std::endl;
            return false;
        }

        dof_ = yaml_loader.DOF;
        if (dof_ <= 0 || dof_ > robot_params::MAX_DOF) {
            std::cerr << "[RobotAbstractBaseNdof::initializeFromYaml] Invalid DOF = "
                      << dof_ << " (expected 1.." << robot_params::MAX_DOF << ")." << std::endl;
            return false;
        }

        // ----------------- active_twists_anat ----------------------------
        if (yaml_loader.active_twist_0.size() < static_cast<std::size_t>(dof_)) {
            std::cerr << "[RobotAbstractBaseNdof::initializeFromYaml] ERROR: active_twist_0 vector too small!"
                      << std::endl;
            return false;
        }

        for (int i = 0; i < dof_; ++i) {
            active_twists_anat[i] = yaml_loader.active_twist_0[i];
            ptr2_active_twists_anat[i] = &active_twists_anat[i];
            std::cout << "[RobotAbstractBaseNdof::initializeFromYaml] active_twists_anat[" << i << "] = "
                      << active_twists_anat[i].transpose() << std::endl;

            // keep legacy active_twists as zeros (deprecated but still allocated)
            active_twists[i] = Eigen::Matrix<float, 6, 1>::Zero();
            ptr2_active_twists[i] = &active_twists[i];
            std::cerr << "[RobotAbstractBaseNdof::initializeFromYaml] WARNING: active_twists[" << i
                      << "] is deprecated and initialized to zero.\n";
        }
        // Zero out remaining slots for safety
        for (int i = dof_; i < robot_params::MAX_DOF; ++i) {
            active_twists_anat[i].setZero();
            ptr2_active_twists_anat[i] = &active_twists_anat[i];
            active_twists[i].setZero();
            ptr2_active_twists[i] = &active_twists[i];
        }

        // ----------------- joint frames (test anatomy) -------------------
        if (yaml_loader.gsa_test_0.size() < static_cast<std::size_t>(dof_)) {
            std::cerr << "[RobotAbstractBaseNdof::initializeFromYaml] ERROR: gsa_test_0 vector too small!"
                      << std::endl;
            return false;
        }

        for (int i = 0; i < dof_; ++i) {
            g_test_0[i] = yaml_loader.gsa_test_0[i];
            gsai_test_ptr[i] = &g_test_0[i];
            std::cout << "[RobotAbstractBaseNdof::initializeFromYaml] gsa_test_0[" << i << "] =\n"
                      << g_test_0[i].matrix() << std::endl;
        }

        // TCP at index dof_
        g_test_0[dof_] = yaml_loader.gst_test_0;
        gsai_test_ptr[dof_] = &g_test_0[dof_];
        std::cout << "[RobotAbstractBaseNdof::initializeFromYaml] gst_test_0 =\n"
                  << g_test_0[dof_].matrix() << std::endl;

        // Initialize remaining unused frames as identity
        for (int i = dof_ + 1; i < robot_params::MAX_DOF + 1; ++i) {
            g_test_0[i] = Eigen::Isometry3f::Identity();
            gsai_test_ptr[i] = &g_test_0[i];
        }

        // ----------------- reference anatomy frames ----------------------
        for (int i = 0; i < dof_ + 1; ++i) {
            g_ref_0[i] = Eigen::Isometry3f::Identity();  // no reference anatomy yet
            gsai_ptr[i] = &g_ref_0[i];
            std::cerr << "[RobotAbstractBaseNdof::initializeFromYaml] WARNING: g_ref_0[" << i
                      << "] is deprecated and initialized to identity.\n";
            std::cout << "[RobotAbstractBaseNdof::initializeFromYaml] g_ref_0[" << i << "] =\n"
                      << g_ref_0[i].matrix() << std::endl;
        }
        for (int i = dof_ + 1; i < robot_params::MAX_DOF + 1; ++i) {
            g_ref_0[i] = Eigen::Isometry3f::Identity();
            gsai_ptr[i] = &g_ref_0[i];
        }

        // ----------------- COM frames ------------------------------------
        if (yaml_loader.gsl_test_0.size() < static_cast<std::size_t>(dof_)) {
            std::cerr << "[RobotAbstractBaseNdof::initializeFromYaml] ERROR: gsl_test_0 vector too small!"
                      << std::endl;
            return false;
        }
        for (int i = 0; i < dof_; ++i) {
            gl_test_0[i] = yaml_loader.gsl_test_0[i];
            gsli_test_ptr[i] = &gl_test_0[i];
            std::cout << "[RobotAbstractBaseNdof::initializeFromYaml] gsl_test_0[" << i << "] =\n"
                      << gl_test_0[i].matrix() << std::endl;
        }
        for (int i = dof_; i < robot_params::MAX_DOF; ++i) {
            gl_test_0[i] = Eigen::Isometry3f::Identity();
            gsli_test_ptr[i] = &gl_test_0[i];
        }

        // ----------------- spatial inertias ------------------------------
        if (yaml_loader.M_s_com_0.size() < static_cast<std::size_t>(dof_)) {
            std::cerr << "[RobotAbstractBaseNdof::initializeFromYaml] ERROR: M_s_com_0 vector too small!"
                      << std::endl;
            return false;
        }
        for (int i = 0; i < dof_; ++i) {
            Mi_s[i] = yaml_loader.M_s_com_0[i];
            Mi_s_ptr[i] = &Mi_s[i];
            std::cout << "[RobotAbstractBaseNdof::initializeFromYaml] M_s[" << i << "] =\n"
                      << Mi_s[i] << std::endl;
        }
        for (int i = dof_; i < robot_params::MAX_DOF; ++i) {
            Mi_s[i].setZero();
            Mi_s_ptr[i] = &Mi_s[i];
        }

        // ----------------- dummy scalars (friction, mass, inertia) -------
        static float dummy_mass[robot_params::MAX_DOF]    = {1.f, 1.f, 1.f, 1.f, 1.f, 1.f};
        static float dummy_inertia[robot_params::MAX_DOF] = {1.f, 1.f, 1.f, 1.f, 1.f, 1.f};
        static float dummy_fc[robot_params::MAX_DOF]      = {1.f, 1.f, 1.f, 1.f, 1.f, 1.f};
        static float dummy_fv[robot_params::MAX_DOF]      = {50.f, 50.f, 50.f, 50.f, 50.f, 50.f};

        for (int i = 0; i < robot_params::MAX_DOF; ++i) {
            link_mass[i]    = &dummy_mass[i];
            link_inertia[i] = &dummy_inertia[i];
            fc_coeffs[i]    = &dummy_fc[i];
            fv_coeffs[i]    = &dummy_fv[i];
        }

        return true;
    }

    virtual ~RobotAbstractBaseNdof() {}

    // --- Structure interface (now with 3 meta-links) --------------------
    virtual uint8_t get_STRUCTURE_ID()       = 0;
    virtual uint8_t get_PSEUDOS_METALINK1()  = 0;
    virtual uint8_t get_PSEUDOS_METALINK2()  = 0;
    virtual uint8_t get_PSEUDOS_METALINK3()  = 0;
    virtual float   get_PSEUDO_ANGLE(int index) = 0;
    virtual Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) = 0;

protected:
    int dof_{0};
};

// ======================================================================
// Concrete Ndof structures: 0, 2, 3, 4, 5, 6 pseudos
// ======================================================================

// ------------- Fixed structure (no pseudos) ----------------------------
class FixedStructureNdof : public RobotAbstractBaseNdof {
public:
    explicit FixedStructureNdof(const std::string & base_yaml_path)
    : RobotAbstractBaseNdof(base_yaml_path)
    {
        initializeFromYaml(base_yaml_path);
    }

    uint8_t get_STRUCTURE_ID() override      { return 0; }
    uint8_t get_PSEUDOS_METALINK1() override { return 0; }
    uint8_t get_PSEUDOS_METALINK2() override { return 0; }
    uint8_t get_PSEUDOS_METALINK3() override { return 0; }

    float get_PSEUDO_ANGLE(int) override {
        return 0.0f;
    }

    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int) override {
        return Eigen::Matrix<float, 6, 1>::Zero();
    }
};

// The following structures differ only in STRUCTURE_ID;
// all of them pull pseudo data from yaml_loader.

// ------------- 2 pseudos ------------------------------------------------
class Structure2PseudosNdof : public RobotAbstractBaseNdof {
public:
    explicit Structure2PseudosNdof(const std::string & base_yaml_path)
    : RobotAbstractBaseNdof(base_yaml_path)
    {
        initializeFromYaml(base_yaml_path);
    }

    uint8_t get_STRUCTURE_ID() override      { return 2; }
    uint8_t get_PSEUDOS_METALINK1() override { return yaml_loader.META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() override { return yaml_loader.META2_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK3() override { return yaml_loader.META3_PSEUDOS; }

    float get_PSEUDO_ANGLE(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.pseudo_angles.size())) {
            std::cerr << "[Structure2PseudosNdof::get_PSEUDO_ANGLE] Index "
                      << index << " out of bounds for pseudo_angles (size="
                      << yaml_loader.pseudo_angles.size() << ")."
                      << std::endl;
            return 0.0f;
        }
        return yaml_loader.pseudo_angles[static_cast<std::size_t>(index)];
    }

    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.passive_twist_0.size())) {
            std::cerr << "[Structure2PseudosNdof::get_PASSIVE_TWIST] Index "
                      << index << " out of bounds for passive_twist_0 (size="
                      << yaml_loader.passive_twist_0.size() << ")."
                      << std::endl;
            return Eigen::Matrix<float, 6, 1>::Zero();
        }
        return yaml_loader.passive_twist_0[static_cast<std::size_t>(index)];
    }
};

// ------------- 3 pseudos ------------------------------------------------
class Structure3PseudosNdof : public RobotAbstractBaseNdof {
public:
    explicit Structure3PseudosNdof(const std::string & base_yaml_path)
    : RobotAbstractBaseNdof(base_yaml_path)
    {
        initializeFromYaml(base_yaml_path);
    }

    uint8_t get_STRUCTURE_ID() override      { return 3; }
    uint8_t get_PSEUDOS_METALINK1() override { return yaml_loader.META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() override { return yaml_loader.META2_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK3() override { return yaml_loader.META3_PSEUDOS; }

    float get_PSEUDO_ANGLE(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.pseudo_angles.size())) {
            std::cerr << "[Structure3PseudosNdof::get_PSEUDO_ANGLE] Index "
                      << index << " out of bounds for pseudo_angles (size="
                      << yaml_loader.pseudo_angles.size() << ")."
                      << std::endl;
            return 0.0f;
        }
        return yaml_loader.pseudo_angles[static_cast<std::size_t>(index)];
    }

    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.passive_twist_0.size())) {
            std::cerr << "[Structure3PseudosNdof::get_PASSIVE_TWIST] Index "
                      << index << " out of bounds for passive_twist_0 (size="
                      << yaml_loader.passive_twist_0.size() << ")."
                      << std::endl;
            return Eigen::Matrix<float, 6, 1>::Zero();
        }
        return yaml_loader.passive_twist_0[static_cast<std::size_t>(index)];
    }
};

// ------------- 4 pseudos ------------------------------------------------
class Structure4PseudosNdof : public RobotAbstractBaseNdof {
public:
    explicit Structure4PseudosNdof(const std::string & base_yaml_path)
    : RobotAbstractBaseNdof(base_yaml_path)
    {
        initializeFromYaml(base_yaml_path);
    }

    uint8_t get_STRUCTURE_ID() override      { return 4; }
    uint8_t get_PSEUDOS_METALINK1() override { return yaml_loader.META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() override { return yaml_loader.META2_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK3() override { return yaml_loader.META3_PSEUDOS; }

    float get_PSEUDO_ANGLE(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.pseudo_angles.size())) {
            std::cerr << "[Structure4PseudosNdof::get_PSEUDO_ANGLE] Index "
                      << index << " out of bounds for pseudo_angles (size="
                      << yaml_loader.pseudo_angles.size() << ")."
                      << std::endl;
            return 0.0f;
        }
        return yaml_loader.pseudo_angles[static_cast<std::size_t>(index)];
    }

    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.passive_twist_0.size())) {
            std::cerr << "[Structure4PseudosNdof::get_PASSIVE_TWIST] Index "
                      << index << " out of bounds for passive_twist_0 (size="
                      << yaml_loader.passive_twist_0.size() << ")."
                      << std::endl;
            return Eigen::Matrix<float, 6, 1>::Zero();
        }
        return yaml_loader.passive_twist_0[static_cast<std::size_t>(index)];
    }
};

// ------------- 5 pseudos ------------------------------------------------
class Structure5PseudosNdof : public RobotAbstractBaseNdof {
public:
    explicit Structure5PseudosNdof(const std::string & base_yaml_path)
    : RobotAbstractBaseNdof(base_yaml_path)
    {
        initializeFromYaml(base_yaml_path);
    }

    uint8_t get_STRUCTURE_ID() override      { return 5; }
    uint8_t get_PSEUDOS_METALINK1() override { return yaml_loader.META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() override { return yaml_loader.META2_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK3() override { return yaml_loader.META3_PSEUDOS; }

    float get_PSEUDO_ANGLE(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.pseudo_angles.size())) {
            std::cerr << "[Structure5PseudosNdof::get_PSEUDO_ANGLE] Index "
                      << index << " out of bounds for pseudo_angles (size="
                      << yaml_loader.pseudo_angles.size() << ")."
                      << std::endl;
            return 0.0f;
        }
        return yaml_loader.pseudo_angles[static_cast<std::size_t>(index)];
    }

    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.passive_twist_0.size())) {
            std::cerr << "[Structure5PseudosNdof::get_PASSIVE_TWIST] Index "
                      << index << " out of bounds for passive_twist_0 (size="
                      << yaml_loader.passive_twist_0.size() << ")."
                      << std::endl;
            return Eigen::Matrix<float, 6, 1>::Zero();
        }
        return yaml_loader.passive_twist_0[static_cast<std::size_t>(index)];
    }
};

// ------------- 6 pseudos ------------------------------------------------
class Structure6PseudosNdof : public RobotAbstractBaseNdof {
public:
    explicit Structure6PseudosNdof(const std::string & base_yaml_path)
    : RobotAbstractBaseNdof(base_yaml_path)
    {
        initializeFromYaml(base_yaml_path);
    }

    uint8_t get_STRUCTURE_ID() override      { return 6; }
    uint8_t get_PSEUDOS_METALINK1() override { return yaml_loader.META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() override { return yaml_loader.META2_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK3() override { return yaml_loader.META3_PSEUDOS; }

    float get_PSEUDO_ANGLE(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.pseudo_angles.size())) {
            std::cerr << "[Structure6PseudosNdof::get_PSEUDO_ANGLE] Index "
                      << index << " out of bounds for pseudo_angles (size="
                      << yaml_loader.pseudo_angles.size() << ")."
                      << std::endl;
            return 0.0f;
        }
        return yaml_loader.pseudo_angles[static_cast<std::size_t>(index)];
    }

    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
        if (index < 0 || index >= static_cast<int>(yaml_loader.passive_twist_0.size())) {
            std::cerr << "[Structure6PseudosNdof::get_PASSIVE_TWIST] Index "
                      << index << " out of bounds for passive_twist_0 (size="
                      << yaml_loader.passive_twist_0.size() << ")."
                      << std::endl;
            return Eigen::Matrix<float, 6, 1>::Zero();
        }
        return yaml_loader.passive_twist_0[static_cast<std::size_t>(index)];
    }
};

#endif  // ROBOT_ABSTRACT_BASE_NDOF_H

#ifndef ROBOT_ABSTRACT_BASE_H
#define ROBOT_ABSTRACT_BASE_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <smm_screws/RobotYamlLoader.h>
#include "smm_screws/robot_parameters.h"

#define ARRAY_33_SIZE       9

class RobotAbstractBase {
public:
    //static constexpr int DOF = 3;

    Eigen::Matrix<float, 6, 1> active_twists[robot_params::DOF];      
    Eigen::Matrix<float, 6, 1> active_twists_anat[robot_params::DOF]; 
    Eigen::Matrix<float, 6, 1>* ptr2_active_twists_anat[robot_params::DOF];

    Eigen::Isometry3f* gsai_ptr[robot_params::DOF+1];        
    Eigen::Isometry3f* gsai_test_ptr[robot_params::DOF+1];   
    Eigen::Isometry3f* gsli_test_ptr[robot_params::DOF];     

    Eigen::Isometry3f g_ref_0[robot_params::DOF+1];
    Eigen::Isometry3f g_test_0[robot_params::DOF+1];
    Eigen::Isometry3f gl_test_0[robot_params::DOF];

    float* link_mass[robot_params::DOF];
    float* link_inertia[robot_params::DOF];
    Eigen::Matrix<float, 6, 6> Mi_s[robot_params::DOF];
    Eigen::Matrix<float, 6, 6>* Mi_s_ptr[robot_params::DOF];

    float* fc_coeffs[robot_params::DOF]; 
    float* fv_coeffs[robot_params::DOF]; 

    RobotYamlLoader yaml_loader;

    bool initializeFromYaml() {
        if (!yaml_loader.loadAll()) {
            std::cerr << "Failed to load robot data from YAML." << std::endl;
            return false;
        }

        for (int i = 0; i < robot_params::DOF; ++i) {
            active_twists_anat[i] = yaml_loader.active_twist_anat[i];
            ptr2_active_twists_anat[i] = &active_twists_anat[i];
        }

        for (int i = 0; i < robot_params::DOF; ++i) {
            g_test_0[i] = yaml_loader.gsa_test[i];
            gsai_test_ptr[i] = &g_test_0[i];
        }
        g_test_0[robot_params::DOF] = yaml_loader.gst_test_0;
        gsai_test_ptr[robot_params::DOF] = &g_test_0[robot_params::DOF];

        for (int i = 0; i < robot_params::DOF; ++i) {
            gl_test_0[i] = yaml_loader.gsl_test[i];
            gsli_test_ptr[i] = &gl_test_0[i];
        }

        for (int i = 0; i < robot_params::DOF; ++i) {
            Mi_s[i] = yaml_loader.M_s[i];
            Mi_s_ptr[i] = &Mi_s[i];
        }

        // Dummy assignments for abstract base
        static float dummy_mass[robot_params::DOF] = {1.0f, 1.0f, 1.0f};
        static float dummy_inertia[robot_params::DOF] = {1.0f, 1.0f, 1.0f};
        static float dummy_fc[robot_params::DOF] = {1.0f, 1.0f, 1.0f};
        static float dummy_fv[robot_params::DOF] = {50.0f, 50.0f, 50.0f};
        for (int i = 0; i < robot_params::DOF; i++) {
            link_mass[i] = &dummy_mass[i];
            link_inertia[i] = &dummy_inertia[i];
            fc_coeffs[i] = &dummy_fc[i];
            fv_coeffs[i] = &dummy_fv[i];
        }

        return true;
    }

    virtual ~RobotAbstractBase() {}
    virtual uint8_t get_STRUCTURE_ID() = 0;
    virtual uint8_t get_PSEUDOS_METALINK1() = 0;
    virtual uint8_t get_PSEUDOS_METALINK2() = 0;
    virtual float get_PSEUDO_ANGLES(int index) = 0;
    virtual Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) = 0;
};

// ========================= FIXED STRUCTURE =========================
class FixedStructure : public RobotAbstractBase {
public:
    FixedStructure() {
        initializeFromYaml();  // uses only reference data in this case
    }

    uint8_t get_STRUCTURE_ID() override { return 0; }
    uint8_t get_PSEUDOS_METALINK1() override { return 0; }
    uint8_t get_PSEUDOS_METALINK2() override { return 0; }
    float get_PSEUDO_ANGLES(int index) override { return 0.0f; }
    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) override {
        return Eigen::Matrix<float, 6, 1>::Zero();
    }
};

// ====================== STRUCTURE WITH 2 PSEUDOS ======================
class Structure2Pseudos : public RobotAbstractBase {
public:
    Structure2Pseudos() {
        initializeFromYaml();
    }

    uint8_t get_STRUCTURE_ID() override { return 2; }
    uint8_t get_PSEUDOS_METALINK1() override { return yaml_loader.META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() override { return yaml_loader.META2_PSEUDOS; }

    float get_PSEUDO_ANGLES(int index) override { return 0.0f; } // TODO: angle loading
    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) override {
        return yaml_loader.passive_twist[index];
    }
};

// ====================== STRUCTURE WITH 3 PSEUDOS ======================
class Structure3Pseudos : public RobotAbstractBase {
public:
    Structure3Pseudos() {
        initializeFromYaml();
    }

    uint8_t get_STRUCTURE_ID() override { return 3; }
    uint8_t get_PSEUDOS_METALINK1() override { return yaml_loader.META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() override { return yaml_loader.META2_PSEUDOS; }

    float get_PSEUDO_ANGLES(int index) override { return 0.0f; } // TODO: angle loading
    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) override {
        return yaml_loader.passive_twist[index];
    }
};

// ====================== STRUCTURE WITH 4 PSEUDOS ======================
class Structure4Pseudos : public RobotAbstractBase {
public:
    Structure4Pseudos() {
        initializeFromYaml();
    }

    uint8_t get_STRUCTURE_ID() override { return 4; }
    uint8_t get_PSEUDOS_METALINK1() override { return yaml_loader.META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() override { return yaml_loader.META2_PSEUDOS; }

    float get_PSEUDO_ANGLES(int index) override { return 0.0f; } // TODO: angle loading
    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) override {
        return yaml_loader.passive_twist[index];
    }
};

#endif  // ROBOT_ABSTRACT_BASE_H

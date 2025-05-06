#ifndef ROBOT_ABSTRACT_BASE_H
#define ROBOT_ABSTRACT_BASE_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <smm_screws/RobotYamlLoader.h>
#include "smm_screws/robot_parameters.h"

class RobotAbstractBase {
public:
    //static constexpr int DOF = 3;

    Eigen::Matrix<float, 6, 1> active_twists[robot_params::DOF];
    Eigen::Matrix<float, 6, 1>* ptr2_active_twists[robot_params::DOF];
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
            std::cerr << "[initializeFromYaml] Failed to load robot data from YAML." << std::endl;
            return false;
        }

        size_t xi_ai_size = sizeof(yaml_loader.active_twist_0) / sizeof(yaml_loader.active_twist_0[0]);
        if (xi_ai_size < robot_params::DOF) {
           std::cerr << "[initializeFromYaml] ERROR: active_twist_0 array too small!" << std::endl;
           return false;
        }   
        for (int i = 0; i < robot_params::DOF; ++i) {
            active_twists_anat[i] = yaml_loader.active_twist_0[i];
            ptr2_active_twists_anat[i] = &active_twists_anat[i];
            std::cout << "[initializeFromYaml] active_twists_anat[" << i << "] =\n" << active_twists_anat[i].transpose() << std::endl;

            // Legacy initialization for backward compatibility
            active_twists[i] = Eigen::Matrix<float, 6, 1>::Zero();
            ptr2_active_twists[i] = &active_twists[i];
            std::cerr << "[initializeFromYaml] WARNING: active_twists[" << i << "] is deprecated and initialized to zero.\n";
        }

        size_t gsa_size = sizeof(yaml_loader.gsa_test_0) / sizeof(yaml_loader.gsa_test_0[0]);
        if (gsa_size < robot_params::DOF) {
            std::cerr << "[initializeFromYaml] ERROR: gsa_test_0 array too small!" << std::endl;
            return false;
        }    
        for (int i = 0; i < robot_params::DOF; ++i) {
            g_test_0[i] = yaml_loader.gsa_test_0[i];
            gsai_test_ptr[i] = &g_test_0[i];
            std::cout << "[initializeFromYaml] gsa_test_0[" << i << "] =\n" << g_test_0[i].matrix() << std::endl;
        }
        g_test_0[robot_params::DOF] = yaml_loader.gst_test_0;
        gsai_test_ptr[robot_params::DOF] = &g_test_0[robot_params::DOF];
        std::cout << "[initializeFromYaml] gst_test_0 =\n" << g_test_0[robot_params::DOF].matrix() << std::endl;
        
        // [BACKWARD COMPATIBILITY] Initialized reference anatomy active joint tfs with identity
        for (int i = 0; i < robot_params::DOF+1; ++i) {
            g_ref_0[i] = Eigen::Isometry3f::Identity();  // Safe default (zeros + identity)
            gsai_ptr[i] = &g_ref_0[i];
            std::cerr << "[initializeFromYaml] WARNING: g_ref_0[" << i << "] is deprecated and initialized to identity.\n";
            std::cout << "[initializeFromYaml] g_ref_0[" << i << "] =\n" << g_ref_0[i].matrix() << std::endl;
        }

        size_t gsl_size = sizeof(yaml_loader.gsl_test_0) / sizeof(yaml_loader.gsl_test_0[0]);
        if (gsl_size < robot_params::DOF) {
            std::cerr << "[initializeFromYaml] ERROR: gsl_test_0 array too small!" << std::endl;
            return false;
        }        
        for (int i = 0; i < robot_params::DOF; ++i) {
            gl_test_0[i] = yaml_loader.gsl_test_0[i];
            gsli_test_ptr[i] = &gl_test_0[i];
            std::cout << "[initializeFromYaml] gsl_test_0[" << i << "] =\n" << gl_test_0[i].matrix() << std::endl;
        }

        size_t Msi_size = sizeof(yaml_loader.M_s_com_0) / sizeof(yaml_loader.M_s_com_0[0]);
        if (Msi_size < robot_params::DOF) {
            std::cerr << "[initializeFromYaml] ERROR: M_s_com_0 array too small!" << std::endl;
            return false;
        }  
        for (int i = 0; i < robot_params::DOF; ++i) {
            Mi_s[i] = yaml_loader.M_s_com_0[i];
            Mi_s_ptr[i] = &Mi_s[i];
            std::cout << "[initializeFromYaml] M_s[" << i << "] =\n" << Mi_s[i] << std::endl;
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
    virtual float get_PSEUDO_ANGLE(int index) = 0;
    virtual Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) = 0;
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
    float get_PSEUDO_ANGLE(int index) override { return 0.0f; }
    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
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

    //float get_PSEUDO_ANGLES(int index) override { return 0.0f; } // TODO: angle loading
    float get_PSEUDO_ANGLE(int index) override {
        if (index < 0 || index >= yaml_loader.pseudo_angles.size()) {
            ROS_ERROR_STREAM("Index " << index << " out of bounds for pseudo_angles.");
            return 0.0f;
        }
        return yaml_loader.pseudo_angles[index];
    }

    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
        return yaml_loader.passive_twist_0[index];
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

    //float get_PSEUDO_ANGLES(int index) override { return 0.0f; } // TODO: angle loading
    float get_PSEUDO_ANGLE(int index) override {
        if (index < 0 || index >= yaml_loader.pseudo_angles.size()) {
            ROS_ERROR_STREAM("Index " << index << " out of bounds for pseudo_angles.");
            return 0.0f;
        }
        return yaml_loader.pseudo_angles[index];
    }

    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
        return yaml_loader.passive_twist_0[index];
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

    //float get_PSEUDO_ANGLES(int index) override { return 0.0f; } // TODO: angle loading
    float get_PSEUDO_ANGLE(int index) override {
        if (index < 0 || index >= yaml_loader.pseudo_angles.size()) {
            ROS_ERROR_STREAM("Index " << index << " out of bounds for pseudo_angles.");
            return 0.0f;
        }
        return yaml_loader.pseudo_angles[index];
    }
    
    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWIST(int index) override {
        return yaml_loader.passive_twist_0[index];
    }
};

#endif  // ROBOT_ABSTRACT_BASE_H

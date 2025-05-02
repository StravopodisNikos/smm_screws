#include "smm_screws/RobotYamlLoader.h"
#include <fstream>
#include <iostream>

Eigen::Matrix4f RobotYamlLoader::loadMatrix4f(const YAML::Node& node) {
    Eigen::Matrix4f mat;
    for (int i = 0; i < 16; ++i) {
        mat(i / 4, i % 4) = node[i].as<float>();
    }
    return mat;
}

Eigen::Matrix<float, 6, 1> RobotYamlLoader::loadTwist6f(const YAML::Node& node) {
    Eigen::Matrix<float, 6, 1> vec;
    for (int i = 0; i < 6; ++i) {
        vec(i) = node[i].as<float>();
    }
    return vec;
}

Eigen::Matrix<float, 6, 6> RobotYamlLoader::loadMatrix6x6f(const YAML::Node& node) {
    Eigen::Matrix<float, 6, 6> mat;
    for (int i = 0; i < 36; ++i) {
        mat(i / 6, i % 6) = node[i].as<float>();
    }
    return mat;
}

bool RobotYamlLoader::loadAll() {
    try {
        // Load active twists
        YAML::Node xi_ai = YAML::LoadFile(basePath + "xi_ai_anat.yaml");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "__active_twist_anat_" + std::to_string(i);
            active_twist_anat[i] = loadTwist6f(xi_ai[key]);
        }

        // Load gst
        YAML::Node gst = YAML::LoadFile(basePath + "gst0.yaml");
        gst_test_0 = loadMatrix4f(gst["gst_test_0"]);

        // Load joint frames
        YAML::Node gsai = YAML::LoadFile(basePath + "gsai0.yaml");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "gsa" + std::to_string(i + 1) + "_test_0";
            gsa_test[i] = loadMatrix4f(gsai[key]);
        }

        // Load passive twists
        YAML::Node xi_pi = YAML::LoadFile(basePath + "xi_pi_anat.yaml");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "__passive_twist_" + std::to_string(i);
            if (xi_pi[key])
                passive_twist[i] = loadTwist6f(xi_pi[key]);
        }

        // Load link COM transforms
        YAML::Node gsli = YAML::LoadFile(basePath + "gsli0.yaml");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "gsl" + std::to_string(i + 1) + "_test_0";
            gsl_test[i] = loadMatrix4f(gsli[key]);
        }

        // Load spatial inertias
        YAML::Node Msi = YAML::LoadFile(basePath + "Mscomi0.yaml");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "M_s_" + std::to_string(i + 1);
            M_s[i] = loadMatrix6x6f(Msi[key]);
        }

        // Load structure information
        YAML::Node assembly = YAML::LoadFile(basePath + "assembly.yaml");
        int s2 = assembly["s2"].as<int>();
        int s3 = assembly["s3"].as<int>();
        int s5 = assembly["s5"].as<int>();
        int s6 = assembly["s6"].as<int>();

        META1_PSEUDOS = (s2 != 9) + (s3 != 9);
        META2_PSEUDOS = (s5 != 9) + (s6 != 9);
        NUM_OF_PSEUDOJOINTS = META1_PSEUDOS + META2_PSEUDOS;

        return true;
    } catch (const std::exception& ex) {
        std::cerr << "YAML load error: " << ex.what() << std::endl;
        return false;
    }
}

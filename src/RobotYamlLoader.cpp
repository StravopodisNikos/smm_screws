#include "smm_screws/RobotYamlLoader.h"
#include <fstream>
#include <iostream>

Eigen::Matrix4f RobotYamlLoader::loadMatrix4f(const YAML::Node& node) {
    Eigen::Matrix4f mat;
    for (int i = 0; i <algebra_params:: MATRIX4F_SIZE; ++i) {
        mat(i / 4, i % 4) = node[i].as<float>();
    }
    return mat;
}

Eigen::Matrix<float, 6, 1> RobotYamlLoader::loadTwist6f(const YAML::Node& node) {
    Eigen::Matrix<float, 6, 1> vec;
    for (int i = 0; i < algebra_params::TWIST6F_SIZE; ++i) {
        vec(i) = node[i].as<float>();
    }
    return vec;
}

Eigen::Matrix<float, 6, 6> RobotYamlLoader::loadMatrix6x6f(const YAML::Node& node) {
    Eigen::Matrix<float, 6, 6> mat;
    for (int i = 0; i < algebra_params::MATRIX6x6F_SIZE; ++i) {
        mat(i / 6, i % 6) = node[i].as<float>();
    }
    return mat;
}

bool RobotYamlLoader::loadAll() {
    try {
        // Load structure information
        YAML::Node assembly = YAML::LoadFile(basePath + "assembly.yaml");
        int s2 = assembly["s2"].as<int>();
        int s3 = assembly["s3"].as<int>();
        int s5 = assembly["s5"].as<int>();
        int s6 = assembly["s6"].as<int>();

        META1_PSEUDOS = (s2 != 9) + (s3 != 9);
        META2_PSEUDOS = (s5 != 9) + (s6 != 9);
        NUM_OF_PSEUDOJOINTS = META1_PSEUDOS + META2_PSEUDOS;

        // Load active twists
        // String keys MUST AGREE WITH MAP IN EXTRACTOR NODES IN ros_pkg: [smm_synthesis] 
        YAML::Node xi_ai = YAML::LoadFile(basePath + "xi_ai_anat.yaml");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "xi_a" + std::to_string(i) + "_0"; // DONE, 
            active_twist_0[i] = loadTwist6f(xi_ai[key]);
        }

        // Load gst
        YAML::Node gst = YAML::LoadFile(basePath + "gst0.yaml");
        gst_test_0 = loadMatrix4f(gst["gst0"]); // DONE

        // Load joint frames
        YAML::Node gsai = YAML::LoadFile(basePath + "gsai0.yaml"); // DONE
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "gsa" + std::to_string(i) + "0";
            gsa_test_0[i] = loadMatrix4f(gsai[key]);
        }

        // Load passive twists
        YAML::Node xi_pj = YAML::LoadFile(basePath + "xi_pj_anat.yaml");
        for (int j = 0; j < NUM_OF_PSEUDOJOINTS; ++j) {
            std::string key = "xi_p" + std::to_string(j) + "_0"; // DONE
            if (xi_pj[key])
                passive_twist_0[j] = loadTwist6f(xi_pj[key]);
        }

        // Load link COM transforms
        YAML::Node gsli = YAML::LoadFile(basePath + "gsli0.yaml"); // DONE
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "gsl" + std::to_string(i) + "0";
            gsl_test_0[i] = loadMatrix4f(gsli[key]);
        }

        // Load spatial inertias
        YAML::Node Msi = YAML::LoadFile(basePath + "Mscomi0.yaml");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "Mscom" + std::to_string(i) + "0"; // DONE
            M_s_com_0[i] = loadMatrix6x6f(Msi[key]);
        }

        return true;
    } catch (const std::exception& ex) {
        std::cerr << "YAML load error: " << ex.what() << std::endl;
        return false;
    }
}

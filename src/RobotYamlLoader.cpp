#include "smm_screws/RobotYamlLoader.h"
#include <fstream>
#include <iostream>

static const Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

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
        ROS_INFO_STREAM("Loading structure parameters from: " << basePath + "assembly.yaml");
        YAML::Node assembly = YAML::LoadFile(basePath + "assembly.yaml");

        int s2 = assembly["s2"].as<int>();
        int s3 = assembly["s3"].as<int>();
        int s5 = assembly["s5"].as<int>();
        int s6 = assembly["s6"].as<int>();

        META1_PSEUDOS = (s2 != 9) + (s3 != 9);
        META2_PSEUDOS = (s5 != 9) + (s6 != 9);
        NUM_OF_PSEUDOJOINTS = META1_PSEUDOS + META2_PSEUDOS;

        ROS_INFO_STREAM("[LoadFromYaml] META1_PSEUDOS: " << META1_PSEUDOS << ", META2_PSEUDOS: " << META2_PSEUDOS);
        ROS_INFO_STREAM("[LoadFromYaml] NUM_OF_PSEUDOJOINTS: " << NUM_OF_PSEUDOJOINTS);

        // Load active twists
        YAML::Node xi_ai = YAML::LoadFile(basePath + "xi_ai_anat.yaml");
        ROS_INFO("[LoadFromYaml] Loading active twists from xi_ai_anat.yaml:");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "xi_a" + std::to_string(i) + "_0";
            active_twist_0[i] = loadTwist6f(xi_ai[key]);
            ROS_INFO_STREAM("[LoadFromYaml] Loaded " << key << ": " << active_twist_0[i].transpose());
        }

        // Load gst
        YAML::Node gst = YAML::LoadFile(basePath + "gst0.yaml");
        gst_test_0 = loadMatrix4f(gst["gst0"]);
        ROS_INFO_STREAM("[LoadFromYaml] Loaded gst0:\n" << gst_test_0.matrix());

        // Load joint frames
        YAML::Node gsai = YAML::LoadFile(basePath + "gsai0.yaml");
        ROS_INFO("[LoadFromYaml] Loading joint frames from gsai0.yaml:");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "gsa" + std::to_string(i) + "0";
            gsa_test_0[i] = loadMatrix4f(gsai[key]);
            ROS_INFO_STREAM("[LoadFromYaml] Loaded " << key << ":\n" << gsa_test_0[i].matrix());
        }

        // Load passive twists
        YAML::Node xi_pj = YAML::LoadFile(basePath + "xi_pj_anat.yaml");
        ROS_INFO("[LoadFromYaml] Loading passive twists from xi_pj_anat.yaml:");
        for (int j = 0; j < NUM_OF_PSEUDOJOINTS; ++j) {
            std::string key = "xi_p" + std::to_string(j) + "_0";
            if (xi_pj[key]) {
                passive_twist_0[j] = loadTwist6f(xi_pj[key]);
                ROS_INFO_STREAM("[LoadFromYaml] Loaded " << key << ": " << passive_twist_0[j].transpose());
            } else {
                ROS_WARN_STREAM("[LoadFromYaml] Key " << key << " not found in xi_pj_anat.yaml");
            }
        }

        // Load link COM vectors and build full Isometry3f matrices
        YAML::Node gsli = YAML::LoadFile(basePath + "gsli0.yaml");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "gsl" + std::to_string(i) + "0";

            // Load vector from YAML
            const YAML::Node& vec = gsli[key];
            if (!vec || vec.size() != 3) {
                std::cerr << "[LoadFromYaml] Invalid or missing COM entry: " << key << std::endl;
                continue;
            }

            // Start from the corresponding joint frame (rotation)
            gsl_test_0[i] = gsa_test_0[i];  // Copy full Isometry (rotation and identity)
            
            // Set only the translation part from the COM data
            gsl_test_0[i].translation() = Eigen::Vector3f(
                vec[0].as<float>(),
                vec[1].as<float>(),
                vec[2].as<float>()
            );

            std::cout << "[LoadFromYaml] Loaded spatial COM vector " << key << ": " << gsl_test_0[i].translation().transpose() << std::endl;
        }

        // Load spatial inertias
        YAML::Node Msi = YAML::LoadFile(basePath + "Mscomi0.yaml");
        ROS_INFO("[LoadFromYaml] Loading spatial inertia matrices from Mscomi0.yaml:");
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "Mscom" + std::to_string(i) + "0";
            M_s_com_0[i] = loadMatrix6x6f(Msi[key]);
            //ROS_INFO_STREAM("Loaded " << key << ":\n" << M_s_com_0[i]);
            Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
            ROS_INFO_STREAM("[LoadFromYaml] Loaded " << key << ":\n" << M_s_com_0[i].format(fmt));
        }

        ROS_INFO("[LoadFromYaml] All robot parameters successfully loaded from YAML.");
        return true;
    } catch (const std::exception& ex) {
        ROS_ERROR_STREAM("[LoadFromYaml] YAML load error: " << ex.what());
        return false;
    }
}


#include "smm_screws/core/RobotYamlLoader.h"
#include <fstream>
#include <iostream>
#include <exception>

static const Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

Eigen::Matrix4f RobotYamlLoader::loadMatrix4f(const YAML::Node& node) {
    Eigen::Matrix4f mat;
    for (int i = 0; i < algebra_params::MATRIX4F_SIZE; ++i) {
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
    // Make sure basePath_ was set
    if (basePath_.empty()) {
        std::cerr << "[RobotYamlLoader::loadAll] basePath_ is empty. "
                  << "Call setBasePath() or use the constructor with a base path."
                  << std::endl;
        return false;
    }

    // Ensure we have a trailing slash (works for paths that already have one)
    std::string base = basePath_;
    if (!base.empty() && base.back() != '/' && base.back() != '\\') {
        base += "/";
    }

    try {
        // === assembly.yaml ===
        std::string assembly_file = base + "assembly.yaml";
        std::cout << "[LoadFromYaml] Loading structure parameters from: "
                  << assembly_file << std::endl;
        YAML::Node assembly = YAML::LoadFile(assembly_file);

        int s2 = assembly["s2"].as<int>();
        int s3 = assembly["s3"].as<int>();
        int s5 = assembly["s5"].as<int>();
        int s6 = assembly["s6"].as<int>();

        META1_PSEUDOS = (s2 != 9) + (s3 != 9);
        META2_PSEUDOS = (s5 != 9) + (s6 != 9);
        NUM_OF_PSEUDOJOINTS = META1_PSEUDOS + META2_PSEUDOS;

        std::cout << "[LoadFromYaml] META1_PSEUDOS: " << META1_PSEUDOS
                  << ", META2_PSEUDOS: " << META2_PSEUDOS << std::endl;
        std::cout << "[LoadFromYaml] NUM_OF_PSEUDOJOINTS: " << NUM_OF_PSEUDOJOINTS
                  << std::endl;

        // === xi_ai_anat.yaml: active twists ===
        std::string xi_ai_file = base + "xi_ai_anat.yaml";
        std::cout << "[LoadFromYaml] Loading active twists from "
                  << xi_ai_file << std::endl;
        YAML::Node xi_ai = YAML::LoadFile(xi_ai_file);
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "xi_a" + std::to_string(i) + "_0";
            active_twist_0[i] = loadTwist6f(xi_ai[key]);
            std::cout << "[LoadFromYaml] Loaded " << key << ": "
                      << active_twist_0[i].transpose() << std::endl;
        }

        // === gst0.yaml: end-effector frame ===
        std::string gst_file = base + "gst0.yaml";
        YAML::Node gst = YAML::LoadFile(gst_file);
        gst_test_0 = loadMatrix4f(gst["gst0"]);
        std::cout << "[LoadFromYaml] Loaded gst0:\n"
                  << gst_test_0.matrix().format(CleanFmt) << std::endl;

        // === gsai0.yaml: joint frames ===
        std::string gsai_file = base + "gsai0.yaml";
        std::cout << "[LoadFromYaml] Loading joint frames from "
                  << gsai_file << std::endl;
        YAML::Node gsai = YAML::LoadFile(gsai_file);
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "gsa" + std::to_string(i) + "0";
            gsa_test_0[i] = loadMatrix4f(gsai[key]);
            std::cout << "[LoadFromYaml] Loaded " << key << ":\n"
                      << gsa_test_0[i].matrix().format(CleanFmt) << std::endl;
        }

        // === xi_pj_anat.yaml: passive twists ===
        std::string xi_pj_file = base + "xi_pj_anat.yaml";
        std::cout << "[LoadFromYaml] Loading passive twists from "
                  << xi_pj_file << std::endl;
        YAML::Node xi_pj = YAML::LoadFile(xi_pj_file);

        passive_twist_0.clear();
        int index = 0;
        while (true) {
            std::string key = "xi_p" + std::to_string(index) + "_0";
            if (!xi_pj[key]) {
                break;
            }

            Eigen::Matrix<float, 6, 1> twist = loadTwist6f(xi_pj[key]);
            passive_twist_0.push_back(twist);
            std::cout << "[LoadFromYaml] Loaded " << key << ": "
                      << twist.transpose() << std::endl;
            ++index;
        }

        // === gsli0.yaml: link COMs (as translations) ===
        std::string gsli_file = base + "gsli0.yaml";
        YAML::Node gsli = YAML::LoadFile(gsli_file);
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "gsl" + std::to_string(i) + "0";

            const YAML::Node& vec = gsli[key];
            if (!vec || vec.size() != 3) {
                std::cerr << "[LoadFromYaml] Invalid or missing COM entry: "
                          << key << std::endl;
                continue;
            }

            // Start from the corresponding joint frame (rotation)
            gsl_test_0[i] = gsa_test_0[i];

            // Set translation from COM data
            gsl_test_0[i].translation() = Eigen::Vector3f(
                vec[0].as<float>(),
                vec[1].as<float>(),
                vec[2].as<float>()
            );

            std::cout << "[LoadFromYaml] Loaded spatial COM vector " << key
                      << ": " << gsl_test_0[i].translation().transpose()
                      << std::endl;
        }

        // === Mscomi0.yaml: spatial inertias ===
        std::string Mscom_file = base + "Mscomi0.yaml";
        std::cout << "[LoadFromYaml] Loading spatial inertia matrices from "
                  << Mscom_file << std::endl;
        YAML::Node Msi = YAML::LoadFile(Mscom_file);
        for (int i = 0; i < robot_params::DOF; ++i) {
            std::string key = "Mscom" + std::to_string(i) + "0";
            M_s_com_0[i] = loadMatrix6x6f(Msi[key]);
            std::cout << "[LoadFromYaml] Loaded " << key << ":\n"
                      << M_s_com_0[i].format(CleanFmt) << std::endl;
        }

        // === q_pj_anat.yaml: pseudo joint angles ===
        std::string pseudo_file = base + "q_pj_anat.yaml";
        YAML::Node pseudo_angles_node = YAML::LoadFile(pseudo_file);

        pseudo_angles.clear();
        const YAML::Node& angle_list = pseudo_angles_node["pseudo_angles"];
        if (!angle_list || !angle_list.IsSequence()) {
            std::cerr << "[LoadFromYaml] pseudo_angles entry is missing or not a sequence."
                      << std::endl;
            return false;
        }

        for (std::size_t i = 0; i < angle_list.size(); ++i) {
            float angle = angle_list[i].as<float>();
            pseudo_angles.push_back(angle);
            std::cout << "[LoadFromYaml] Loaded pseudo_angles[" << i
                      << "]: " << angle << std::endl;
        }

        std::cout << "[LoadFromYaml] All robot parameters successfully loaded from YAML."
                  << std::endl;
        return true;
    } catch (const std::exception& ex) {
        std::cerr << "[LoadFromYaml] YAML load error: " << ex.what() << std::endl;
        return false;
    }
}

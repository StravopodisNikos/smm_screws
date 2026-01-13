#include "smm_screws/core/RobotYamlLoaderNdof.h"
#include <fstream>
#include <iostream>
#include <exception>

// If algebra_params is not pulled in by robot_parameters.h,
// include its header here:
// #include "smm_screws/algebra_parameters.h"

static const Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

// Helper: load 4x4 matrix from a flat YAML sequence
Eigen::Matrix4f RobotYamlLoaderNdof::loadMatrix4f(const YAML::Node& node) {
    Eigen::Matrix4f mat;
    for (int i = 0; i < algebra_params::MATRIX4F_SIZE; ++i) {
        mat(i / 4, i % 4) = node[i].as<float>();
    }
    return mat;
}

// Helper: load 6×1 twist from YAML sequence
Eigen::Matrix<float, 6, 1> RobotYamlLoaderNdof::loadTwist6f(const YAML::Node& node) {
    Eigen::Matrix<float, 6, 1> vec;
    for (int i = 0; i < algebra_params::TWIST6F_SIZE; ++i) {
        vec(i) = node[i].as<float>();
    }
    return vec;
}

// Helper: load 6×6 matrix from a flat YAML sequence
Eigen::Matrix<float, 6, 6> RobotYamlLoaderNdof::loadMatrix6x6f(const YAML::Node& node) {
    Eigen::Matrix<float, 6, 6> mat;
    for (int i = 0; i < algebra_params::MATRIX6x6F_SIZE; ++i) {
        mat(i / 6, i % 6) = node[i].as<float>();
    }
    return mat;
}

// ======================================================================
// Public entry point
// ======================================================================

bool RobotYamlLoaderNdof::loadAll() {
    // Make sure basePath_ was set
    if (basePath_.empty()) {
        std::cerr << "[RobotYamlLoaderNdof::loadAll] basePath_ is empty. "
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
        // 1. assembly.yaml -> DOF, META1/2/3_PSEUDOS, NUM_OF_PSEUDOJOINTS
        if (!loadAssemblyYaml(base)) {
            return false;
        }

        // Resize dynamic containers now that DOF is known.
        active_twist_0.clear();
        gsa_test_0.clear();
        gsl_test_0.clear();
        M_s_com_0.clear();
        passive_twist_0.clear();
        pseudo_angles.clear();

        active_twist_0.resize(DOF);
        gsa_test_0.resize(DOF);
        gsl_test_0.resize(DOF);
        M_s_com_0.resize(DOF);

        // Note: passive_twist_0 and pseudo_angles will be sized by their
        // individual loaders, then cross-checked with NUM_OF_PSEUDOJOINTS.

        // 2. xi_ai_anat.yaml -> active_twist_0
        if (!loadActiveTwistsYaml(base))   return false;

        // 3. gst0.yaml + gsai0.yaml -> gst_test_0, gsa_test_0
        if (!loadActiveFramesYaml(base))   return false;

        // 4. xi_pj_anat.yaml -> passive_twist_0
        if (!loadPassiveTwistsYaml(base))  return false;

        // 5. gsli0.yaml -> gsl_test_0 (COM frames built from gsa_test_0 rotations + COM translations)
        if (!loadComFramesYaml(base))      return false;

        // 6. Mscomi0.yaml -> M_s_com_0
        if (!loadInertiasYaml(base))       return false;

        // 7. q_pj_anat.yaml -> pseudo_angles
        if (!loadPseudoAnglesYaml(base))   return false;

        std::cout << "[LoadFromYamlNdof] All robot parameters successfully loaded from YAML."
                  << std::endl;

        return true;
    } catch (const std::exception& ex) {
        std::cerr << "[LoadFromYamlNdof] YAML load error: " << ex.what() << std::endl;
        return false;
    }
}

// ======================================================================
// Private steps
// ======================================================================

bool RobotYamlLoaderNdof::loadAssemblyYaml(const std::string& base)
{
    std::string assembly_file = base + "assembly.yaml";
    std::cout << "[LoadFromYamlNdof] Loading structure parameters from: "
              << assembly_file << std::endl;

    YAML::Node assembly = YAML::LoadFile(assembly_file);

    // 1. DOF
    if (assembly["dof"]) {
        DOF = assembly["dof"].as<int>();
    } else {
        DOF = robot_params::DOF;  // fallback for backward compatibility
        std::cerr << "[LoadFromYamlNdof] WARNING: 'dof' key missing in assembly.yaml. "
                  << "Using robot_params::DOF = " << DOF << std::endl;
    }

    if (DOF < 3 || DOF > 6) {
        std::cerr << "[LoadFromYamlNdof] ERROR: Invalid DOF = " << DOF
                  << ". Expected 3..6." << std::endl;
        return false;
    }

    // 2. Structure digits s1..s9
    auto get_or_default = [&](const char* key, int def_val) -> int {
        if (assembly[key]) {
            return assembly[key].as<int>();
        }
        return def_val;
    };

    int s1 = get_or_default("s1", 9);
    int s2 = get_or_default("s2", 9);
    int s3 = get_or_default("s3", 9);
    int s4 = get_or_default("s4", 9);
    int s5 = get_or_default("s5", 9);
    int s6 = get_or_default("s6", 9);
    int s7 = get_or_default("s7", 9);
    int s8 = get_or_default("s8", 9);
    int s9 = get_or_default("s9", 9);

    // 3. Meta-link pseudo counts for 3 meta-links
    META1_PSEUDOS = (s2 != 9) + (s3 != 9);  // meta-link 1
    META2_PSEUDOS = (s5 != 9) + (s6 != 9);  // meta-link 2
    META3_PSEUDOS = (s8 != 9) + (s9 != 9);  // meta-link 3

    NUM_OF_PSEUDOJOINTS = META1_PSEUDOS + META2_PSEUDOS + META3_PSEUDOS;

    std::cout << "[LoadFromYamlNdof] DOF: " << DOF << std::endl;
    std::cout << "[LoadFromYamlNdof] s1..s9 = "
              << s1 << " " << s2 << " " << s3 << " "
              << s4 << " " << s5 << " " << s6 << " "
              << s7 << " " << s8 << " " << s9 << std::endl;

    std::cout << "[LoadFromYamlNdof] META1_PSEUDOS: " << META1_PSEUDOS
              << ", META2_PSEUDOS: " << META2_PSEUDOS
              << ", META3_PSEUDOS: " << META3_PSEUDOS << std::endl;
    std::cout << "[LoadFromYamlNdof] NUM_OF_PSEUDOJOINTS: "
              << NUM_OF_PSEUDOJOINTS << std::endl;

    return true;
}

bool RobotYamlLoaderNdof::loadActiveTwistsYaml(const std::string& base)
{
    std::string xi_ai_file = base + "xi_ai_anat.yaml";
    std::cout << "[LoadFromYamlNdof] Loading active twists from "
              << xi_ai_file << std::endl;

    YAML::Node xi_ai = YAML::LoadFile(xi_ai_file);

    if (active_twist_0.size() != static_cast<std::size_t>(DOF)) {
        active_twist_0.resize(DOF);
    }

    for (int i = 0; i < DOF; ++i) {
        std::string key = "xi_a" + std::to_string(i) + "_0";
        if (!xi_ai[key]) {
            std::cerr << "[LoadFromYamlNdof] Missing key " << key
                      << " in " << xi_ai_file << std::endl;
            return false;
        }
        active_twist_0[i] = loadTwist6f(xi_ai[key]);
        std::cout << "[LoadFromYamlNdof] Loaded " << key << ": "
                  << active_twist_0[i].transpose() << std::endl;
    }

    return true;
}

bool RobotYamlLoaderNdof::loadActiveFramesYaml(const std::string& base)
{
    // === gst0.yaml: end-effector frame ===
    {
        std::string gst_file = base + "gst0.yaml";
        YAML::Node gst = YAML::LoadFile(gst_file);
        gst_test_0 = loadMatrix4f(gst["gst0"]);
        std::cout << "[LoadFromYamlNdof] Loaded gst0:\n"
                  << gst_test_0.matrix().format(CleanFmt) << std::endl;
    }

    // === gsai0.yaml: joint frames ===
    {
        std::string gsai_file = base + "gsai0.yaml";
        std::cout << "[LoadFromYamlNdof] Loading joint frames from "
                  << gsai_file << std::endl;

        YAML::Node gsai = YAML::LoadFile(gsai_file);

        if (gsa_test_0.size() != static_cast<std::size_t>(DOF)) {
            gsa_test_0.resize(DOF);
        }

        for (int i = 0; i < DOF; ++i) {
            std::string key = "gsa" + std::to_string(i) + "0";
            if (!gsai[key]) {
                std::cerr << "[LoadFromYamlNdof] Missing key " << key
                          << " in " << gsai_file << std::endl;
                return false;
            }
            gsa_test_0[i] = loadMatrix4f(gsai[key]);
            std::cout << "[LoadFromYamlNdof] Loaded " << key << ":\n"
                      << gsa_test_0[i].matrix().format(CleanFmt) << std::endl;
        }
    }

    return true;
}

bool RobotYamlLoaderNdof::loadPassiveTwistsYaml(const std::string& base)
{
    std::string xi_pj_file = base + "xi_pj_anat.yaml";
    std::cout << "[LoadFromYamlNdof] Loading passive twists from "
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
        std::cout << "[LoadFromYamlNdof] Loaded " << key << ": "
                  << twist.transpose() << std::endl;
        ++index;
    }

    if (NUM_OF_PSEUDOJOINTS > 0 &&
        static_cast<int>(passive_twist_0.size()) != NUM_OF_PSEUDOJOINTS)
    {
        std::cerr << "[LoadFromYamlNdof] WARNING: NUM_OF_PSEUDOJOINTS ("
                  << NUM_OF_PSEUDOJOINTS << ") does not match number of "
                  << "passive twists found (" << passive_twist_0.size() << ")."
                  << std::endl;
        NUM_OF_PSEUDOJOINTS = static_cast<int>(passive_twist_0.size());
    }

    return true;
}

bool RobotYamlLoaderNdof::loadComFramesYaml(const std::string& base)
{
    // === gsli0.yaml: link COMs (as translations) ===
    std::string gsli_file = base + "gsli0.yaml";
    YAML::Node gsli = YAML::LoadFile(gsli_file);

    if (gsl_test_0.size() != static_cast<std::size_t>(DOF)) {
        gsl_test_0.resize(DOF);
    }

    for (int i = 0; i < DOF; ++i) {
        std::string key = "gsl" + std::to_string(i) + "0";

        const YAML::Node& vec = gsli[key];
        if (!vec || vec.size() != 3) {
            std::cerr << "[LoadFromYamlNdof] Invalid or missing COM entry: "
                      << key << std::endl;
            continue;
        }

        if (i >= static_cast<int>(gsa_test_0.size())) {
            std::cerr << "[LoadFromYamlNdof] gsa_test_0 not initialized for link "
                      << i << ", cannot build COM frame " << key << std::endl;
            continue;
        }

        gsl_test_0[i] = gsa_test_0[i];  // copy rotation and transform

        gsl_test_0[i].translation() = Eigen::Vector3f(
            vec[0].as<float>(),
            vec[1].as<float>(),
            vec[2].as<float>()
        );

        std::cout << "[LoadFromYamlNdof] Loaded spatial COM vector " << key
                  << ": " << gsl_test_0[i].translation().transpose()
                  << std::endl;
    }

    return true;
}

bool RobotYamlLoaderNdof::loadInertiasYaml(const std::string& base)
{
    // === Mscomi0.yaml: spatial inertias ===
    std::string Mscom_file = base + "Mscomi0.yaml";
    std::cout << "[LoadFromYamlNdof] Loading spatial inertia matrices from "
              << Mscom_file << std::endl;
    YAML::Node Msi = YAML::LoadFile(Mscom_file);

    if (M_s_com_0.size() != static_cast<std::size_t>(DOF)) {
        M_s_com_0.resize(DOF);
    }

    for (int i = 0; i < DOF; ++i) {
        std::string key = "Mscom" + std::to_string(i) + "0";
        if (!Msi[key]) {
            std::cerr << "[LoadFromYamlNdof] Missing key " << key
                      << " in " << Mscom_file << std::endl;
            return false;
        }
        M_s_com_0[i] = loadMatrix6x6f(Msi[key]);
        std::cout << "[LoadFromYamlNdof] Loaded " << key << ":\n"
                  << M_s_com_0[i].format(CleanFmt) << std::endl;
    }

    return true;
}

bool RobotYamlLoaderNdof::loadPseudoAnglesYaml(const std::string& base)
{
    // === q_pj_anat.yaml: pseudo joint angles ===
    std::string pseudo_file = base + "q_pj_anat.yaml";
    YAML::Node pseudo_angles_node = YAML::LoadFile(pseudo_file);

    pseudo_angles.clear();

    const YAML::Node& angle_list = pseudo_angles_node["pseudo_angles"];
    if (!angle_list || !angle_list.IsSequence()) {
        std::cerr << "[LoadFromYamlNdof] pseudo_angles entry is missing or not a sequence."
                  << std::endl;
        return false;
    }

    for (std::size_t i = 0; i < angle_list.size(); ++i) {
        float angle = angle_list[i].as<float>();
        pseudo_angles.push_back(angle);
        std::cout << "[LoadFromYamlNdof] Loaded pseudo_angles[" << i
                  << "]: " << angle << std::endl;
    }

    if (NUM_OF_PSEUDOJOINTS > 0 &&
        static_cast<int>(pseudo_angles.size()) != NUM_OF_PSEUDOJOINTS)
    {
        std::cerr << "[LoadFromYamlNdof] WARNING: NUM_OF_PSEUDOJOINTS ("
                  << NUM_OF_PSEUDOJOINTS << ") does not match number of "
                  << "pseudo angles found (" << pseudo_angles.size() << ")."
                  << std::endl;
        NUM_OF_PSEUDOJOINTS = static_cast<int>(pseudo_angles.size());
    }

    return true;
}

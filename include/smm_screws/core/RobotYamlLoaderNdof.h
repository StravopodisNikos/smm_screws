#ifndef ROBOT_YAML_LOADER_NDOF_H
#define ROBOT_YAML_LOADER_NDOF_H

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>

#include "smm_screws/robot_parameters.h"

// COMMENTS
// 1. [10-1-2026] No fixed size for loaded arrays. Elements are loaded
//    dynamically based on saved YAML files data.

class RobotYamlLoaderNdof {
public:
    // From assembly.yaml
    int DOF                 = 0;  // 3..6
    int NUM_OF_PSEUDOJOINTS = 0;
    int META1_PSEUDOS       = 0;
    int META2_PSEUDOS       = 0;
    int META3_PSEUDOS       = 0;

    // N-sized data (sizes depend on DOF / NUM_OF_PSEUDOJOINTS)
    std::vector<Eigen::Matrix<float, 6, 1>> active_twist_0;  // size = DOF
    Eigen::Isometry3f                       gst_test_0;      // TCP
    std::vector<Eigen::Isometry3f>          gsa_test_0;      // size = DOF (joint frames)

    std::vector<Eigen::Matrix<float, 6, 1>> passive_twist_0; // size = NUM_OF_PSEUDOJOINTS
    std::vector<Eigen::Isometry3f>          gsl_test_0;      // size = DOF (COM frames)
    std::vector<Eigen::Matrix<float, 6, 6>> M_s_com_0;       // size = DOF (spatial inertias)

    std::vector<float> pseudo_angles;                        // size = NUM_OF_PSEUDOJOINTS

    /// Default constructor: base path must be set later with setBasePath()
    RobotYamlLoaderNdof() = default;

    /// Convenience constructor: set the YAML base path directly.
    explicit RobotYamlLoaderNdof(const std::string & base_path)
    : basePath_(base_path)
    {}

    /// Set the directory where YAML files are stored, e.g.
    /// ".../smm_synthesis/config/yaml/"
    void setBasePath(const std::string & base_path)
    {
        basePath_ = base_path;
    }

    /// Load all YAML files using the current basePath_.
    /// Returns false if basePath_ is empty or any file fails.
    bool loadAll();

private:
    /// Directory containing YAML files, WITHOUT ROS dependency.
    /// Example value: "/home/nikos/ros2_ws/src/smm_synthesis/config/yaml/"
    std::string basePath_;

    // ---- helpers for loading individual YAMLs ----
    bool loadAssemblyYaml(const std::string& base);          // sets DOF, META1/2/3_PSEUDOS, NUM_OF_PSEUDOJOINTS
    bool loadActiveTwistsYaml(const std::string& base);      // fills active_twist_0 (size DOF)
    bool loadActiveFramesYaml(const std::string& base);      // fills gsa_test_0 (size DOF), gst_test_0
    bool loadPassiveTwistsYaml(const std::string& base);     // fills passive_twist_0 (size NUM_OF_PSEUDOJOINTS)
    bool loadComFramesYaml(const std::string& base);         // fills gsl_test_0 (size DOF)
    bool loadInertiasYaml(const std::string& base);          // fills M_s_com_0 (size DOF)
    bool loadPseudoAnglesYaml(const std::string& base);      // fills pseudo_angles (size NUM_OF_PSEUDOJOINTS)

    // Matrix extraction helpers
    Eigen::Matrix4f            loadMatrix4f(const YAML::Node& node);
    Eigen::Matrix<float, 6, 1> loadTwist6f(const YAML::Node& node);
    Eigen::Matrix<float, 6, 6> loadMatrix6x6f(const YAML::Node& node);
};

#endif  // ROBOT_YAML_LOADER_NDOF_H

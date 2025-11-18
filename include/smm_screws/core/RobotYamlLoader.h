#ifndef ROBOT_YAML_LOADER_H
#define ROBOT_YAML_LOADER_H

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include "smm_screws/robot_parameters.h"

class RobotYamlLoader {
public:
    int NUM_OF_PSEUDOJOINTS = 0;
    int META1_PSEUDOS = 0;
    int META2_PSEUDOS = 0;

    Eigen::Matrix<float, 6, 1> active_twist_0[robot_params::DOF];
    Eigen::Isometry3f gst_test_0;
    Eigen::Isometry3f gsa_test_0[robot_params::DOF];
    std::vector<Eigen::Matrix<float, 6, 1>> passive_twist_0;
    Eigen::Isometry3f gsl_test_0[robot_params::DOF];
    Eigen::Matrix<float, 6, 6> M_s_com_0[robot_params::DOF];
    std::vector<float> pseudo_angles;

    /// Default constructor: base path must be set later with setBasePath()
    RobotYamlLoader() = default;

    /// Convenience constructor: set the YAML base path directly.
    explicit RobotYamlLoader(const std::string & base_path)
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

    Eigen::Matrix4f loadMatrix4f(const YAML::Node& node);
    Eigen::Matrix<float, 6, 1> loadTwist6f(const YAML::Node& node);
    Eigen::Matrix<float, 6, 6> loadMatrix6x6f(const YAML::Node& node);
};

#endif  // ROBOT_YAML_LOADER_H

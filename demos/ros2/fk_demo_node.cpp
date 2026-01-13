#include <memory>
#include <string>
#include <stdexcept>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "smm_screws/core/RobotContext.h"
#include "smm_screws/core/ScrewsKinematics.h"

class FKDemoNode : public rclcpp::Node
{
public:
  FKDemoNode()
  : rclcpp::Node("smm_fk_demo_simple")
  {
    // 1) YAML base dir (where assembly.yaml etc live)
    yaml_base_dir_ = this->declare_parameter<std::string>(
      "yaml_base_dir",
      "/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml"
    );

    if (!yaml_base_dir_.empty() &&
        yaml_base_dir_.back() != '/' &&
        yaml_base_dir_.back() != '\\')
    {
      yaml_base_dir_ += "/";
    }

    // 2) Read assembly.yaml -> infer structure digit (2/3/4 pseudos)
    const std::string assembly_path = yaml_base_dir_ + "assembly.yaml";
    YAML::Node assembly;
    try {
      assembly = YAML::LoadFile(assembly_path);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Failed to load '%s': %s", assembly_path.c_str(), e.what());
      throw;
    }

    int s2 = assembly["s2"].as<int>();
    int s3 = assembly["s3"].as<int>();
    int s5 = assembly["s5"].as<int>();
    int s6 = assembly["s6"].as<int>();

    int count = 0;
    if (s2 != 9) ++count;
    if (s3 != 9) ++count;
    if (s5 != 9) ++count;
    if (s6 != 9) ++count;

    if (count < 2 || count > 4) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid pseudojoint count from assembly.yaml: %d (need 2..4)", count);
      throw std::runtime_error("Invalid pseudojoint count");
    }

    // 3) Construct the right RobotAbstractBase subclass for this structure
    std::unique_ptr<RobotAbstractBase> robot;

    switch (count) {
      case 2:
        robot = std::make_unique<Structure2Pseudos>(yaml_base_dir_);
        RCLCPP_INFO(this->get_logger(), "Using Structure2Pseudos");
        break;
      case 3:
        robot = std::make_unique<Structure3Pseudos>(yaml_base_dir_);
        RCLCPP_INFO(this->get_logger(), "Using Structure3Pseudos");
        break;
      case 4:
        robot = std::make_unique<Structure4Pseudos>(yaml_base_dir_);
        RCLCPP_INFO(this->get_logger(), "Using Structure4Pseudos");
        break;
      default:
        RCLCPP_FATAL(this->get_logger(), "Unexpected pseudojoint count=%d", count);
        throw std::runtime_error("Unexpected pseudojoint count");
    }

    // 4) Create RobotContext with that robot and initialize it
    robot_context_ = std::make_unique<RobotContext>(std::move(robot));

    if (!robot_context_->initializeSharedLib()) {
      RCLCPP_FATAL(
        this->get_logger(),
        "RobotContext::initializeSharedLib() failed.");
      throw std::runtime_error("RobotContext initializeSharedLib failed");
    }

    // 5) Get kinematics object
    auto & kin = robot_context_->get_kinematics();  // or get_screws_kinematics_solver() if that's your name

    // 6) Define joint config: q = 0, dq = 0 (3-DOF current implementation)
    float q[3]  = {0.0f, 0.0f, 0.0f};
    float dq[3] = {0.0f, 0.0f, 0.0f};

    kin.updateJointState(q, dq);
    kin.ForwardKinematics3DOF_2();

    Eigen::Vector3f tcp_pos = kin.updatePositionTCP(q);

    RCLCPP_INFO(
      this->get_logger(),
      "TCP position at q = [0, 0, 0] is: [%.5f, %.5f, %.5f]",
      tcp_pos.x(), tcp_pos.y(), tcp_pos.z());

  }

private:
  std::string yaml_base_dir_;
  std::unique_ptr<RobotContext> robot_context_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FKDemoNode>());
  rclcpp::shutdown();
  return 0;
}

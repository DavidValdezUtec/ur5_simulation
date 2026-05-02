#ifndef UR5_CONTROLLER__CSV_LOGGER_HPP_
#define UR5_CONTROLLER__CSV_LOGGER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <string>

namespace ur5_controller
{

struct CsvRow
{
  double t {0.0};
  double dt {0.0};

  Eigen::VectorXd q_meas_joints {Eigen::VectorXd::Zero(6)};
  Eigen::VectorXd q_cmd {Eigen::VectorXd::Zero(6)};
  Eigen::VectorXd u_control {Eigen::VectorXd::Zero(6)};

  Eigen::Vector3d x_des {Eigen::Vector3d::Zero()};
  Eigen::Vector3d x_meas {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_des {Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond q_meas_pose {Eigen::Quaterniond::Identity()};
  Eigen::Vector3d euler_des {Eigen::Vector3d::Zero()};
  Eigen::Vector3d euler_meas {Eigen::Vector3d::Zero()};
  double e_R_angle {0.0};
  Eigen::Vector3d pos_err {Eigen::Vector3d::Zero()};
  Eigen::Vector3d ori_err_axis {Eigen::Vector3d::Zero()};
  double ori_err_angle {0.0};

  double ik_ms {0.0};
  double loop_ms {0.0};
};

class CsvLogger
{
public:
  explicit CsvLogger(rclcpp::Logger logger = rclcpp::get_logger("CsvLogger"));

  void configure(bool enabled,
                 const std::string& csv_dir,
                 const std::string& csv_prefix,
                 const std::string& namespace_name);

  void writeRow(const CsvRow& row);
  bool isEnabled() const;
  const std::string& filePath() const;

private:
  static std::string getHomeDir();
  static std::string timestampString();

  rclcpp::Logger logger_;
  bool enabled_ {false};
  std::string file_path_;
  std::ofstream file_;
};

}  // namespace ur5_controller

#endif  // UR5_CONTROLLER__CSV_LOGGER_HPP_
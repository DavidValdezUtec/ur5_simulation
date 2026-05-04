#ifndef UR5_CONTROLLER__JOINT_STATE_MAPPER_HPP_
#define UR5_CONTROLLER__JOINT_STATE_MAPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>

#include <array>
#include <string>
#include <vector>

namespace ur5_controller
{

class JointStateMapper
{
public:
  JointStateMapper();

  void configure(const std::string& namespace_name,
                 rclcpp::Logger logger = rclcpp::get_logger("JointStateMapper"));

  bool updateOrderedState(const sensor_msgs::msg::JointState::SharedPtr& msg,
                          Eigen::VectorXd& q,
                          Eigen::VectorXd& qd);

  bool isReady() const;

private:
  std::vector<std::string> getExpectedJointNames() const;
  std::vector<std::string> getExpectedBaseJointNames() const;
  static bool endsWith(const std::string& str, const std::string& suffix);
  static bool sameNameList(const std::vector<std::string>& a, const std::vector<std::string>& b);
  void rebuildJointIndexMap(const sensor_msgs::msg::JointState::SharedPtr& msg);

  std::string namespace_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("JointStateMapper")};
  std::array<int, 6> joint_index_map_ {{-1, -1, -1, -1, -1, -1}};
  bool joint_map_initialized_ {false};
  std::vector<std::string> last_js_names_;
};

}  // namespace ur5_controller

#endif  // UR5_CONTROLLER__JOINT_STATE_MAPPER_HPP_
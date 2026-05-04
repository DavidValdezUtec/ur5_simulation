#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <vector>

namespace ur5_controller {

struct NodeConfig;

class ParameterManager {
public:
    ParameterManager(NodeConfig* config,
                     Eigen::Vector3i* map_pos,
                     Eigen::Vector3d* sign_pos,
                     Eigen::Vector3i* map_rot,
                     Eigen::Vector3d* sign_rot,
                     std::mutex* node_mutex,
                     const rclcpp::Logger& logger);

    rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter>& params);

private:
    NodeConfig* config_;
    Eigen::Vector3i* map_pos_;
    Eigen::Vector3d* sign_pos_;
    Eigen::Vector3i* map_rot_;
    Eigen::Vector3d* sign_rot_;
    rclcpp::Logger logger_;
    std::mutex* node_mutex_ = nullptr;

    bool validate_map_idx(double v, const std::string& name, rcl_interfaces::msg::SetParametersResult& result) const;
    bool validate_sign(double v, const std::string& name, rcl_interfaces::msg::SetParametersResult& result) const;
};

} // namespace ur5_controller

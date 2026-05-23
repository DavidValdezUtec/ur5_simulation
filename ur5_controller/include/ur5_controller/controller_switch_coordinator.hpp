#pragma once

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ur5_controller
{

class ControllerSwitchCoordinator
{
public:
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  ControllerSwitchCoordinator(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    const rclcpp::Client<SwitchController>::SharedPtr & switch_client);

  void setInitialForwardState();

  bool forwardActive() const;
  bool scaledActive() const;

  void requestScaledIfNeeded(const std::function<void()> & on_scaled_activated);
  void requestForwardIfNeeded(const std::function<void()> & on_forward_activated);

private:
  void requestSwitch(
    const std::vector<std::string> & activate_controllers,
    const std::vector<std::string> & deactivate_controllers,
    bool & request_in_progress,
    const std::string & switch_description,
    const std::function<void(bool)> & on_result);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;

  bool switch_to_forward_requested_ = false;
  bool switch_to_scaled_requested_ = false;
  bool forward_controller_active_ = false;
  bool scaled_controller_active_ = false;
};

}  // namespace ur5_controller

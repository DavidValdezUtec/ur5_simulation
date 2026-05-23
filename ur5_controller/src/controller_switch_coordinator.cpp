#include <ur5_controller/controller_switch_coordinator.hpp>

namespace ur5_controller
{

ControllerSwitchCoordinator::ControllerSwitchCoordinator(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  const rclcpp::Client<SwitchController>::SharedPtr & switch_client)
: logger_(std::move(logger)), clock_(std::move(clock)), switch_client_(switch_client)
{
}

void ControllerSwitchCoordinator::setInitialForwardState()
{
  forward_controller_active_ = true;
  scaled_controller_active_ = false;
}

bool ControllerSwitchCoordinator::forwardActive() const
{
  return forward_controller_active_;
}

bool ControllerSwitchCoordinator::scaledActive() const
{
  return scaled_controller_active_;
}

void ControllerSwitchCoordinator::requestSwitch(
  const std::vector<std::string> & activate_controllers,
  const std::vector<std::string> & deactivate_controllers,
  bool & request_in_progress,
  const std::string & switch_description,
  const std::function<void(bool)> & on_result)
{
  if (request_in_progress) {
    return;
  }

  if (!switch_client_) {
    RCLCPP_ERROR(logger_, "No se pudo crear el cliente de switch_controller");
    on_result(false);
    return;
  }

  if (!switch_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Esperando el servicio switch_controller del controller_manager...");
    return;
  }

  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = activate_controllers;
  request->deactivate_controllers = deactivate_controllers;
  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout.sec = 5;
  request->timeout.nanosec = 0;

  request_in_progress = true;
  RCLCPP_INFO(logger_, "Solicitando switch: %s", switch_description.c_str());

  auto callback = [this, &request_in_progress, on_result](rclcpp::Client<SwitchController>::SharedFuture future) {
      request_in_progress = false;
      try {
        const auto response = future.get();
        on_result(response->ok);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Error procesando respuesta de switch_controller: %s", e.what());
        on_result(false);
      }
    };

  switch_client_->async_send_request(request, callback);
}

void ControllerSwitchCoordinator::requestScaledIfNeeded(const std::function<void()> & on_scaled_activated)
{
  if (scaled_controller_active_) {
    return;
  }

  requestSwitch(
    {"scaled_joint_trajectory_controller"},
    {"forward_position_controller"},
    switch_to_scaled_requested_,
    "deactivate forward_position_controller -> activate scaled_joint_trajectory_controller",
    [this, on_scaled_activated](bool ok) {
      if (ok) {
        scaled_controller_active_ = true;
        forward_controller_active_ = false;
        on_scaled_activated();
        RCLCPP_INFO(
          logger_,
          "Switch completado: movimiento inicial sobre scaled_joint_trajectory_controller activo.");
      } else {
        RCLCPP_ERROR(
          logger_,
          "El switch_controller a scaled_joint_trajectory_controller fue rechazado.");
      }
    });
}

void ControllerSwitchCoordinator::requestForwardIfNeeded(const std::function<void()> & on_forward_activated)
{
  if (forward_controller_active_) {
    return;
  }

  requestSwitch(
    {"forward_position_controller"},
    {"scaled_joint_trajectory_controller"},
    switch_to_forward_requested_,
    "deactivate scaled_joint_trajectory_controller -> activate forward_position_controller",
    [this, on_forward_activated](bool ok) {
      if (ok) {
        forward_controller_active_ = true;
        scaled_controller_active_ = false;
        on_forward_activated();
        RCLCPP_INFO(
          logger_,
          "Switch completado: ya se puede continuar con forward_position_controller.");
      } else {
        RCLCPP_ERROR(
          logger_,
          "El switch_controller fue rechazado por controller_manager.");
      }
    });
}

}  // namespace ur5_controller

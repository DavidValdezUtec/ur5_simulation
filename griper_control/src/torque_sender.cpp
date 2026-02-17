#include <rclcpp/rclcpp.hpp>
#include "omni_msgs/msg/omni_button_event.hpp"

Class Force_Sender : public rclcpp :: Node {
    public:
    Force_Sender() : Node("force_sender") {
        subscription_phantom_button_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(
            "/phantom/button", 10, std::bind(&Force_Sender::button_callback, this, std::placeholders::_1));
        subscription_phantom_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/phantom/pose", 10, std::bind(&Force_Sender::pose_callback, this, std::placeholders::_1));
    }

    private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        if (capturar_pose_inicial_haptico_) { // Si aún no se ha capturado la pose inicial, no actualizar el estado
            RCLCPP_INFO(this->get_logger(), "Esperando captura de pose inicial del Geomagic...");


        }
}
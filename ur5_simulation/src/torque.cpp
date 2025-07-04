#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <omni_msgs/msg/omni_feedback.hpp>

class FuerzaFeedback : public rclcpp::Node {
public:
    FuerzaFeedback() : Node("force_feedback_publisher") {
        force_feedback_pub_ = this->create_publisher<omni_msgs::msg::OmniFeedback>("/phantom/force_feedback", 10);
        ur5e_force_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/ur5e/force", 10,
            std::bind(&FuerzaFeedback::ur5e_force_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FuerzaFeedback::publish_force_feedback, this));
    }
private:
    void ur5e_force_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        last_ur5e_force_ = *msg;
    }
    void publish_force_feedback() {        
        auto message = omni_msgs::msg::OmniFeedback();
        // Fuerzas (en Newtons)
        message.force = last_ur5e_force_;

        // Posición (opcional, puedes poner lo que quieras)
        message.position.x = 0.0;
        message.position.y = 0.0;
        message.position.z = 0.0;
        force_feedback_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "OmniFeedback publicado: Fuerza [%.2f, %.2f, %.2f], Posición [%.2f, %.2f, %.2f]",
            message.force.x, message.force.y, message.force.z,
            message.position.x, message.position.y, message.position.z);
    }
    rclcpp::Publisher<omni_msgs::msg::OmniFeedback>::SharedPtr force_feedback_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ur5e_force_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Vector3 last_ur5e_force_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FuerzaFeedback>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
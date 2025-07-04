#include <rclcpp/rclcpp.hpp>


#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <omni_msgs/msg/omni_feedback.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


#include <iostream>
#include <algorithm>
#include <memory>
#include <string>   
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;


void initializeUR5(std::unique_ptr<pinocchio::Model>& model,  std::unique_ptr<pinocchio::Data>& data, pinocchio::FrameIndex& tool_frame_id, const std::string& urdf_path) {     model = std::make_unique<pinocchio::Model>();

    auto logger = rclcpp::get_logger("UR5Kinematics");
    RCLCPP_INFO(logger, "Intentando cargar URDF desde: %s", urdf_path.c_str());

    try {
        pinocchio::urdf::buildModel(urdf_path, *model);
        RCLCPP_INFO(logger, "URDF cargado exitosamente!");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error cargando URDF: %s", e.what());
        throw;
    }

    data = std::make_unique<pinocchio::Data>(*model); // crea un puntero a los datos de pinocchio
    tool_frame_id = model->getFrameId("wrist_3_link"); // Obtener el ID del marco del efector final

    if (tool_frame_id == static_cast<pinocchio::FrameIndex>(model->nframes)) {
        RCLCPP_ERROR(logger, "Error: Marco 'tool0' no encontrado en el URDF!");
        throw std::runtime_error("Frame wrist_3_link no encontrado");
    }
}

std::string get_file_path(const std::string& package_name, const std::string& relative_path) {
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        return package_share_directory + "/" + relative_path;
    } catch (const std::exception& e) {
        throw std::runtime_error("No se pudo encontrar el paquete: " + package_name);
    }
}

class FuerzaFeedback : public rclcpp::Node {
public:
    FuerzaFeedback() : Node("force_feedback_publisher") {
        

        initializeUR5(model, data, tool_frame_id, urdf_path);    
        force_feedback_pub_ = this->create_publisher<omni_msgs::msg::OmniFeedback>("/phantom/force_feedback", 10);
        ur5e_force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/force_torque_sensor_broadcaster/wrench", 10,
            std::bind(&FuerzaFeedback::ur5e_force_callback, this, std::placeholders::_1));
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&FuerzaFeedback::update_joint_positions, this, std::placeholders::_1));            
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FuerzaFeedback::publish_force_feedback, this));
    }
private:
    void ur5e_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        last_ur5e_force_ = msg->wrench.force;
    }

    void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
            last_joint_state_ = msg;
            //bool implementacion = false; // Variable para determinar si se implementa la cinemática inversa
            if (implementacion){
                q_[0] = msg->position[5];           qd_[0] = msg->velocity[5];
                q_[1] = msg->position[0];           qd_[1] = msg->velocity[0];
                q_[2] = msg->position[1];           qd_[2] = msg->velocity[1];
                q_[3] = msg->position[2];           qd_[3] = msg->velocity[2];
                q_[4] = msg->position[3];           qd_[4] = msg->velocity[3];
                q_[5] = msg->position[4];           qd_[5] = msg->velocity[4];
            }
            else{
                for (int i = 0; i < 6; ++i) {
                    q_[i] = msg->position[i];
                    qd_[i] = msg->velocity[i];

                }
            }
            // Imprimir las posiciones articulares
            //RCLCPP_INFO(this->get_logger(), "Posiciones articulares: %.4f %.4f %.4f %.4f %.4f %.4f",   q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
            // Imprimir las velocidades articulares
            //RCLCPP_INFO(this->get_logger(), "Velocidades articulares: %.4f %.4f %.4f %.4f %.4f %.4f",  qd_[0], qd_[1], qd_[2], qd_[3], qd_[4], qd_[5]);
        }

    void publish_force_feedback() {        
        auto message = omni_msgs::msg::OmniFeedback();
        // Saturar la fuerza a ±3.3 N por componente
        message.force.x = last_ur5e_force_.x;//<std::max(-3.3, std::min(3.3, last_ur5e_force_.x/10));
        message.force.y = last_ur5e_force_.y;//std::max(-3.3, std::min(3.3, last_ur5e_force_.y/10));
        message.force.z = last_ur5e_force_.z;//std::max(-3.3, std::min(3.3, last_ur5e_force_.z/10));
        std::string end_effector_name = "wrist_3_link"; // Nombre del efector final del UR5e

        //actualizar la cinemática directa
        pinocchio::forwardKinematics(*model, *data, q_, qd_);
        pinocchio::updateFramePlacements(*model, *data);

        //obtiene la orientación del efector final
        R = data->oMf[tool_frame_id].rotation();
        std::cout << "Orientación del efector final (matriz de rotación):" << std::endl << R << std::endl;

        //obtiene las fuerzas rotadas al sistema de coordenadas global
        Eigen::Vector3d fuerza_efector(message.force.x, message.force.y, message.force.z);
        Eigen::Vector3d fuerza_base = R.transpose() * fuerza_efector;
        std::cout << "Fuerza en el efector final (sensor): [" << message.force.x << ", " << message.force.y << ", " << message.force.z << "]" << std::endl;
        std::cout << "Fuerza en el sistema de coordenadas base: [" << fuerza_base.x() << ", " << fuerza_base.y() << ", " << fuerza_base.z() << "]" << std::endl;

        double escala = 0.075;
        // Posición (opcional)
        message.position.x = 0.0;
        message.position.y = 0.0;
        message.position.z = 0.0;
        // Saturar la fuerza a ±3.3 N por componente
        message.force.x = -std::max(-3.3, std::min(3.3, fuerza_base.y()*escala));
        message.force.y = std::max(-3.3, std::min(3.3, fuerza_base.x()*escala));
        message.force.z = std::max(-3.3, std::min(3.3, fuerza_base.z()*escala));
        force_feedback_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "OmniFeedback publicado: Fuerza [%.2f, %.2f, %.2f], Posición [%.2f, %.2f, %.2f]",
            message.force.x, message.force.y, message.force.z,
            message.position.x, message.position.y, message.position.z);
    }


    Eigen::VectorXd q_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd qd_ = Eigen::VectorXd::Zero(6);
    bool implementacion = true; // Variable para determinar si se implementa la cinemática inversa
    //orientacion del efector final
    Eigen::Matrix3d R;


    rclcpp::Publisher<omni_msgs::msg::OmniFeedback>::SharedPtr force_feedback_pub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ur5e_force_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Vector3 last_ur5e_force_;
    std::string urdf_path = get_file_path("ur5_simulation",   "include/ur5.urdf");
    std::unique_ptr<pinocchio::Model> model; // declarar puntero único para el modelo
    std::unique_ptr<pinocchio::Data> data; // declarar puntero único para los datos  
    pinocchio::FrameIndex tool_frame_id; 
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;



};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FuerzaFeedback>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
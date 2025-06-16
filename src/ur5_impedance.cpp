#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
//dinamica
#include <pinocchio/algorithm/crba.hpp>      // Para la matriz de inercia
#include <pinocchio/algorithm/rnea.hpp>      // Para Coriolis y gravedad
#include <pinocchio/algorithm/aba.hpp>       // Para la dinámica inversa
#include <Eigen/Dense>

#include <iostream>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
using namespace std;


void initializeUR5(std::unique_ptr<pinocchio::Model>& model,  std::unique_ptr<pinocchio::Data>& data,
                   pinocchio::FrameIndex& tool_frame_id,      const std::string& urdf_path) {
    model = std::make_unique<pinocchio::Model>();

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
    tool_frame_id = model->getFrameId("tool0");

    if (tool_frame_id == static_cast<pinocchio::FrameIndex>(model->nframes)) {
        RCLCPP_ERROR(logger, "Error: Marco 'tool0' no encontrado en el URDF!");
        throw std::runtime_error("Frame tool0 no encontrado");
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

Eigen::MatrixXd jacobian(const pinocchio::Model& model, pinocchio::Data& data, 
                         const pinocchio::FrameIndex& tool_frame_id, const Eigen::VectorXd& q) {
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, tool_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
}

class UR5eJointController : public rclcpp::Node {
    public:
        UR5eJointController() : Node("ur5e_joint_controller"), time_elapsed_(0.0) {
            initializeUR5(model, data, tool_frame_id, urdf_path);            // Publicador para enviar trayectorias
            joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(control_topic, 10);
    
            // Suscriptor para leer el estado articular actual
            subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&UR5eJointController::update_joint_positions, this, std::placeholders::_1));            
                // Suscriptor para leer la posición cartesiana del haptic phantom
            
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&UR5eJointController::posicion_inicial, this));
        }
    
    private:
        bool posicion_inicial_alcanzada_ = false; // Bandera para sincronización
        double time_elapsed_;
        double previous_error_[3] = {0, 0, 0}; // Error anterior
        double error_[3] = {0, 0, 0}; // Error actual
        double r_[3] = {0, 0, 0}; // Posiciones del haptic phantom
        double qt_[4] = {0, 0, 0, 0}; // Orientacion del haptic phantom  
        double qt_inv[4] = {0, 0, 0, 0}; // Orientacion del haptic phantom inverso
        std::ofstream output_file_;
        std::ofstream output_file_2;
        std::ofstream output_file_3;

        Eigen::Vector3d euler_angles;
        Eigen::Matrix3d rotation_matrix;
        Eigen::Quaterniond q_x;        Eigen::Quaterniond q_y;        Eigen::Quaterniond q_z;        
        Eigen::Quaterniond quat_initial_UR5; 
        Eigen::Quaterniond quat_initial_geo; 
        Eigen::Quaterniond quat_real_geo; 
        
        
        double q_[6] ;
        double qd_[6] ;
        double h_[6] ;
        double q_init[6];
        double x_init[3]; 
        double x_des[3];
        double qt_init_ur5[4];
        double qt_init_geo[4];
        int control_loop_time = 1; 
        int ur5_time = 0.01;
        double max_iteraciones[1];
        double alpha[1];
        string control_topic = "/joint_trajectory_controller/joint_trajectory";
        sensor_msgs::msg::JointState::SharedPtr last_joint_state_;    
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;
        std::unique_ptr<pinocchio::Model> model; // declarar puntero único para el modelo
        std::unique_ptr<pinocchio::Data> data; // declarar puntero único para los datos
        pinocchio::FrameIndex tool_frame_id; 

        std::string urdf_path = get_file_path("ur5_simulation",   "include/ur5.urdf");

        

        // POSICIONES ARTICULARES DEL UR5
        void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
            last_joint_state_ = msg;
            bool implementacion = false; // Variable para determinar si se implementa la cinemática inversa
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
            RCLCPP_INFO(this->get_logger(), "Posiciones articulares: %.2f %.2f %.2f %.2f %.2f %.2f",
                        q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
            // Imprimir las velocidades articulares
            RCLCPP_INFO(this->get_logger(), "Velocidades articulares: %.2f %.2f %.2f %.2f %.2f %.2f",
                        qd_[0], qd_[1], qd_[2], qd_[3], qd_[4], qd_[5]);
        }
        
        
        void posicion_inicial() {
            //calcula el jacobiano y la cinemática directa
            pinocchio::forwardKinematics(*model, *data, Eigen::Map<Eigen::VectorXd>(q_, 6));
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);

            
            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            RCLCPP_INFO(this->get_logger(), "Enviando posición inicial: %.2f %.2f %.2f %.2f %.2f %.2f",
                        q_init[0], q_init[1], q_init[2], q_init[3], q_init[4], q_init[5]);
            point.positions = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
            point.time_from_start = rclcpp::Duration::from_seconds(0.1);
            trajectory_msg.points.push_back(point);
            joint_trajectory_pub_->publish(trajectory_msg);

            
        }

    };


int main(int argc, char **argv) {  
    cout<<"Control con Geomagic"<<endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5eJointController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  
} 


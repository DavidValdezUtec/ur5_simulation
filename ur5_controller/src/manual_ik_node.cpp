#include <ur5_kinematics/kinematics.hpp>
#include <ur5_impedance/impedance.hpp>
#include <ur5_sliding/sliding.hpp>
#include <ur5_controller/structs.hpp>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>

// Mensajes
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Eigen para matemáticas
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>

// Utilidades de ROS 2
#include <ament_index_cpp/get_package_share_directory.hpp>

// Standard C++
#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <unordered_map>
#include <filesystem>
#include <iomanip>
#include <ctime>
#include <thread>

// Usar el namespace para las structs
using namespace ur5_controller;

#define M_PI 3.14159265358979323846


void initializeUR5(PinocchioResources& pinocchio, const std::string& urdf_path) {
    pinocchio.model = std::make_unique<pinocchio::Model>();

    auto logger = rclcpp::get_logger("ManualIKNode");
    RCLCPP_INFO(logger, "Intentando cargar URDF desde: %s", urdf_path.c_str());

    try {
        pinocchio::urdf::buildModel(urdf_path, *pinocchio.model);
        RCLCPP_INFO(logger, "URDF cargado exitosamente!");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error cargando URDF: %s", e.what());
        throw;
    }

    pinocchio.data = std::make_unique<pinocchio::Data>(*pinocchio.model);
    pinocchio.tool_frame_id = pinocchio.model->getFrameId("tool0");

    if (pinocchio.tool_frame_id == static_cast<pinocchio::FrameIndex>(pinocchio.model->nframes)) {
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

class ManualIKNode : public rclcpp::Node
{
public:
    ManualIKNode() : Node("manual_ik_node")
    {
        // Parámetros de configuración
        this->declare_parameter<bool>("use_current_pose", true);
        this->declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("initial_orientation_quat", {0.0, 0.0, 0.0, 1.0}); // x,y,z,w
        this->declare_parameter<std::vector<double>>("target_pose", {0.55566, 0.42846, 0.5});
        this->declare_parameter<std::vector<double>>("target_orientation_quat", {0.49603, 0.11866, 0.20079, 0.83639}); // x,y,z,w
        this->declare_parameter<std::string>("controller", "QP"); // QP, IMP, SLD
        this->declare_parameter<std::string>("namespace", "");
        this->declare_parameter<std::string>("ur_model", "ur5");
        this->declare_parameter<double>("control_rate", 100.0); // Hz
        this->declare_parameter<int>("qp_max_iterations", 600);
        this->declare_parameter<double>("qp_control_hz", 100.0);
        
        // Parámetros para IMP
        this->declare_parameter<std::vector<double>>("imp_kp", {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
        this->declare_parameter<std::vector<double>>("imp_kd", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
        this->declare_parameter<double>("imp_dt", 0.01);
        
        // Parámetros para SLD
        this->declare_parameter<std::vector<double>>("sld_lambda", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        this->declare_parameter<std::vector<double>>("sld_k", {50.0, 50.0, 50.0, 50.0, 50.0, 50.0});
        this->declare_parameter<std::vector<double>>("sld_k2", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
        this->declare_parameter<double>("sld_alpha", 0.01);
        this->declare_parameter<double>("sld_damping_factor", 0.01);
        this->declare_parameter<double>("sld_dt", 0.01);

        // Cargar parámetros
        use_current_pose_ = this->get_parameter("use_current_pose").as_bool();
        controller_type_ = this->get_parameter("controller").as_string();
        namespace_ = this->get_parameter("namespace").as_string();
        ur_model_ = this->get_parameter("ur_model").as_string();
        control_rate_ = this->get_parameter("control_rate").as_double();
        qp_max_iterations_ = this->get_parameter("qp_max_iterations").as_int();
        qp_control_hz_ = this->get_parameter("qp_control_hz").as_double();
        
        // Cargar pose inicial si no se usa la actual
        auto initial_pose_vec = this->get_parameter("initial_pose").as_double_array();
        initial_position_ << initial_pose_vec[0], initial_pose_vec[1], initial_pose_vec[2];
        
        auto initial_ori_vec = this->get_parameter("initial_orientation_quat").as_double_array();
        initial_orientation_.x() = initial_ori_vec[0];
        initial_orientation_.y() = initial_ori_vec[1];
        initial_orientation_.z() = initial_ori_vec[2];
        initial_orientation_.w() = initial_ori_vec[3];
        initial_orientation_.normalize();
        
        // Cargar pose objetivo
        auto target_pose_vec = this->get_parameter("target_pose").as_double_array();
        target_position_ << target_pose_vec[0], target_pose_vec[1], target_pose_vec[2];
        
        auto target_ori_vec = this->get_parameter("target_orientation_quat").as_double_array();
        target_orientation_.x() = target_ori_vec[0];
        target_orientation_.y() = target_ori_vec[1];
        target_orientation_.z() = target_ori_vec[2];
        target_orientation_.w() = target_ori_vec[3];
        target_orientation_.normalize();
        
        // Cargar parámetros de controladores
        auto imp_kp_vec = this->get_parameter("imp_kp").as_double_array();
        for (int i = 0; i < 7; ++i) config_.Kp[i] = imp_kp_vec[i];
        
        auto imp_kd_vec = this->get_parameter("imp_kd").as_double_array();
        for (int i = 0; i < 7; ++i) config_.Kd[i] = imp_kd_vec[i];
        
        config_.dt = this->get_parameter("imp_dt").as_double();
        
        auto sld_lambda_vec = this->get_parameter("sld_lambda").as_double_array();
        for (int i = 0; i < 6; ++i) config_.lambda[i] = sld_lambda_vec[i];
        
        auto sld_k_vec = this->get_parameter("sld_k").as_double_array();
        for (int i = 0; i < 6; ++i) config_.k[i] = sld_k_vec[i];
        
        auto sld_k2_vec = this->get_parameter("sld_k2").as_double_array();
        for (int i = 0; i < 6; ++i) config_.k2[i] = sld_k2_vec[i];
        
        config_.alpha = this->get_parameter("sld_alpha").as_double();
        config_.damping_factor = this->get_parameter("sld_damping_factor").as_double();
        
        // Inicializar URDF
        std::string urdf_package = "ur5_description";
        std::string urdf_path = get_file_path(urdf_package, "urdf/ur5.urdf");
        initializeUR5(pinocchio_, urdf_path);
        
        // Inicializar controladores
        kinematics_solver_ = std::make_unique<UR5Kinematics>(urdf_path);
        impedance_controller_ = std::make_unique<ur5_impedance::UR5Impedance>(urdf_path);
        sliding_controller_ = std::make_unique<ur5_sliding::UR5Sliding>(urdf_path);
        
        // Configurar nombres de joints
        std::string prefix = namespace_.empty() ? "" : (namespace_ + "_");
        std::vector<std::string> joint_names = {
            prefix + "shoulder_pan_joint",
            prefix + "shoulder_lift_joint",
            prefix + "elbow_joint",
            prefix + "wrist_1_joint",
            prefix + "wrist_2_joint",
            prefix + "wrist_3_joint"
        };
        
        // Subscriber a joint_states
        std::string joint_states_topic = namespace_.empty() ? "/joint_states" : ("/" + namespace_ + "/joint_states");
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic, 10,
            std::bind(&ManualIKNode::joint_state_callback, this, std::placeholders::_1));
        
        // Publisher de comandos de posición
        std::string cmd_topic = namespace_.empty() ? 
            "/scaled_joint_trajectory_controller/joint_trajectory" : 
            ("/" + namespace_ + "/scaled_joint_trajectory_controller/joint_trajectory");
        
        // Usar forward_position_controller
        std::string position_topic = namespace_.empty() ? 
            "/forward_position_controller/commands" : 
            ("/" + namespace_ + "/forward_position_controller/commands");
            
        joint_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(position_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "=== Manual IK Node Iniciado ===");
        RCLCPP_INFO(this->get_logger(), "Controlador: %s", controller_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Usar pose actual: %s", use_current_pose_ ? "SI" : "NO");
        RCLCPP_INFO(this->get_logger(), "Pose objetivo: [%.3f, %.3f, %.3f]", 
                    target_position_.x(), target_position_.y(), target_position_.z());
        RCLCPP_INFO(this->get_logger(), "Esperando joint_states...");
    }

private:
    // Recursos
    PinocchioResources pinocchio_;
    std::unique_ptr<UR5Kinematics> kinematics_solver_;
    std::unique_ptr<ur5_impedance::UR5Impedance> impedance_controller_;
    std::unique_ptr<ur5_sliding::UR5Sliding> sliding_controller_;
    
    // Estado del robot
    RobotState robot_state_;
    CartesianState cartesian_state_;
    NodeConfig config_;
    
    // Parámetros
    bool use_current_pose_;
    std::string controller_type_;
    std::string namespace_;
    std::string ur_model_;
    double control_rate_;
    int qp_max_iterations_;
    double qp_control_hz_;
    
    // Poses inicial y objetivo
    Eigen::Vector3d initial_position_;
    Eigen::Quaterniond initial_orientation_;
    Eigen::Vector3d target_position_;
    Eigen::Quaterniond target_orientation_;
    
    // Flags de estado
    bool initial_pose_set_ = false;
    bool execution_started_ = false;
    bool execution_finished_ = false;
    
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "JointState recibido con menos de 6 posiciones");
            return;
        }
        
        // Copiar posiciones y velocidades
        for (int i = 0; i < 6 && i < static_cast<int>(msg->position.size()); ++i) {
            robot_state_.q[i] = msg->position[i];
            robot_state_.qd[i] = (i < static_cast<int>(msg->velocity.size())) ? msg->velocity[i] : 0.0;
        }
        
        // Calcular pose cartesiana actual
        pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
        pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
        const auto& frame_placement = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
        
        cartesian_state_.position = frame_placement.translation();
        cartesian_state_.rotation_matrix = frame_placement.rotation();
        cartesian_state_.orientation = Eigen::Quaterniond(frame_placement.rotation());
        
        // Si es la primera vez y usa pose actual, capturar pose inicial
        if (!initial_pose_set_) {
            if (use_current_pose_) {
                initial_position_ = cartesian_state_.position;
                initial_orientation_ = cartesian_state_.orientation;
                RCLCPP_INFO(this->get_logger(), "Pose inicial capturada (actual): [%.3f, %.3f, %.3f]",
                           initial_position_.x(), initial_position_.y(), initial_position_.z());
                RCLCPP_INFO(this->get_logger(), "Orientación inicial capturada (actual): [%.3f, %.3f, %.3f, %.3f]",
                           initial_orientation_.x(), initial_orientation_.y(), 
                           initial_orientation_.z(), initial_orientation_.w());
            } else {
                RCLCPP_INFO(this->get_logger(), "Usando pose inicial definida: [%.3f, %.3f, %.3f]",
                           initial_position_.x(), initial_position_.y(), initial_position_.z());
            }
            
            robot_state_.q_init = robot_state_.q;
            cartesian_state_.position_initial = initial_position_;
            cartesian_state_.orientation_initial = initial_orientation_;
            cartesian_state_.rotation_matrix_initial = initial_orientation_.toRotationMatrix();
            
            
            initial_pose_set_ = true;
            
            // Iniciar ejecución según el controlador
            start_execution();
        }
    }
    
    void start_execution() {
        if (execution_started_) return;
        
        execution_started_ = true;
        RCLCPP_INFO(this->get_logger(), "=== Iniciando ejecución del controlador %s ===", controller_type_.c_str());
        
        // Configurar poses deseadas
        cartesian_state_.position_desired = target_position_;
        cartesian_state_.orientation_desired = target_orientation_;
        cartesian_state_.rotation_matrix_desired = target_orientation_.toRotationMatrix();
        
        if (controller_type_ == "QP") {
            execute_qp_controller();
        } else {
            // Para IMP y SLD, crear un timer que ejecute el control periódicamente
            control_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)),
                std::bind(&ManualIKNode::control_loop, this));
        }
    }
    
    void execute_qp_controller() {
        RCLCPP_INFO(this->get_logger(), "Ejecutando controlador QP con publicación de cada iteración...");
        
        // Implementación de QP inline para publicar cada iteración
        Eigen::VectorXd q = robot_state_.q;
        Eigen::Matrix3d desired_orient = cartesian_state_.rotation_matrix_desired;
        Eigen::Vector3d desired_pos = cartesian_state_.position_desired;
        
        const double joint_limit = 2 * M_PI;
        const double dq_max_norm = 0.5;
        const int iter_cap = std::min(qp_max_iterations_, 150);
        const double alpha_eff = std::max(0.1, std::min(qp_control_hz_ / 100.0, 1.0));
        
        pinocchio::SE3 desired_pose(desired_orient, desired_pos);
        
        for (int i = 0; i < iter_cap; ++i) {
            pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, q);
            pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
            
            // Calcular error
            const pinocchio::SE3 current_pose = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
            Eigen::Vector3d position_error = desired_pose.translation() - current_pose.translation();
            Eigen::Matrix3d R_err = desired_pose.rotation() * current_pose.rotation().transpose();
            Eigen::Vector3d angular_error;
            angular_error << R_err(2,1) - R_err(1,2),
                             R_err(0,2) - R_err(2,0),
                             R_err(1,0) - R_err(0,1);
            angular_error *= 0.5;
            
            Eigen::Matrix<double, 6, 1> error;
            error << position_error, angular_error;
            
            // Verificar convergencia
            if (error.norm() < 1e-4) {
                RCLCPP_INFO(this->get_logger(), "QP convergió en %d iteraciones", i);
                break;
            }
            
            // Calcular Jacobiano
            pinocchio::Data::Matrix6x J(6, pinocchio_.model->nv);
            J.setZero();
            pinocchio::computeFrameJacobian(*pinocchio_.model, *pinocchio_.data, q, 
                                           pinocchio_.tool_frame_id, 
                                           pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
            
            // Resolver QP (pseudoinversa simplificada)
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::VectorXd dq = svd.solve(error);
            
            // Limitar paso
            double nrm = dq.norm();
            if (nrm > dq_max_norm) {
                dq *= (dq_max_norm / nrm);
            }
            
            double dt = 1.0 / alpha_eff;
            double max_step = 2.5 * dt;
            double max_dq = dq.cwiseAbs().maxCoeff();
            if (max_dq > max_step) {
                dq *= (max_step / max_dq);
            }
            
            // Actualizar q
            q += dq;
            
            // Aplicar límites de joints
            for (int j = 0; j < q.size(); ++j) {
                if (q[j] > joint_limit) q[j] = joint_limit;
                else if (q[j] < -joint_limit) q[j] = -joint_limit;
            }
            
            // PUBLICAR ESTA ITERACIÓN
            publish_joint_command(q);
            
            // Pequeña pausa para permitir que el robot se mueva
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            if (i % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "QP iteración %d/%d, error: %.4f", 
                           i, iter_cap, error.norm());
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "QP finalizado. Pose objetivo alcanzada.");
        execution_finished_ = true;
    }
    
    void control_loop() {
        if (execution_finished_) {
            control_timer_->cancel();
            return;
        }
        
        // Actualizar cinemática actual
        pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
        pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
        
        const auto& frame_placement = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
        cartesian_state_.position = frame_placement.translation();
        cartesian_state_.rotation_matrix = frame_placement.rotation();
        
        // Verificar si llegamos al objetivo
        Eigen::Vector3d error_pos = cartesian_state_.position - cartesian_state_.position_desired;
        if (error_pos.norm() < 0.001) {
            RCLCPP_INFO(this->get_logger(), "Objetivo alcanzado!");
            execution_finished_ = true;
            return;
        }
        
        // Ejecutar controlador seleccionado
        Eigen::VectorXd q_cmd;
        
        if (controller_type_ == "IMP") {
            auto result = impedance_controller_->calculateControlCommand(
                robot_state_.q,
                robot_state_.qd,
                cartesian_state_.position_desired,
                cartesian_state_.orientation_desired,
                Eigen::Vector3d::Zero(), // velocidad deseada
                Eigen::Vector3d::Zero(), // aceleración deseada
                config_.Kp,
                config_.Kd,
                config_.dt);
            q_cmd = result.q_desired;
            
        } else if (controller_type_ == "SLD") {
            auto result = sliding_controller_->calculateControlCommand(
                robot_state_.q,
                robot_state_.qd,
                cartesian_state_.position_desired,
                cartesian_state_.rotation_matrix_desired,
                Eigen::Vector3d::Zero(), // velocidad deseada
                Eigen::Vector3d::Zero(), // velocidad angular deseada
                Eigen::Vector3d::Zero(), // aceleración deseada
                Eigen::Vector3d::Zero(), // aceleración angular deseada
                config_.lambda,
                config_.k,
                config_.k2,
                config_.alpha,
                config_.damping_factor,
                config_.dt);
            q_cmd = result.q_desired;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Controlador desconocido: %s", controller_type_.c_str());
            execution_finished_ = true;
            return;
        }
        
        // Publicar comando
        publish_joint_command(q_cmd);
        
        // Log periódico
        static int loop_count = 0;
        if (++loop_count % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "%s: error pos = %.4f m", 
                       controller_type_.c_str(), error_pos.norm());
        }
    }
    
    void publish_joint_command(const Eigen::VectorXd& q) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.assign(q.data(), q.data() + q.size());
        joint_position_pub_->publish(msg);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualIKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
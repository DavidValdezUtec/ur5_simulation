// #include <ur5_kinematics/kinematics.hpp>
// #include <ur5_impedance/impedance.hpp>
// #include <ur5_sliding/sliding.hpp>
// #include <ur5_controller/structs.hpp>

// // ROS 2 Core
// #include <rclcpp/rclcpp.hpp>

// // Mensajes
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <omni_msgs/msg/omni_state.hpp>
// #include <omni_msgs/msg/omni_feedback.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <controller_manager_msgs/srv/switch_controller.hpp>

// // Eigen para matemáticas
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

// // Utilidades de ROS 2
// #include <ament_index_cpp/get_package_share_directory.hpp>

// #include <ur5_controller/csv_logger.hpp>
// #include <ur5_controller/joint_state_mapper.hpp>
// #include <ur5_controller/trajectory_generator.hpp>
// #include <ur5_controller/initial_motion_publisher.hpp>
// #include <ur5_controller/parameter_manager.hpp>
// #include <ur5_controller/controller_switch_coordinator.hpp>

// // Standard C++
// #include <iostream>
// #include <fstream>
// #include <chrono>
// #include <memory>
// #include <mutex>
// #include <string>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <sstream>
// #include <unordered_map>
// #include <filesystem>
// #include <iomanip>
// #include <ctime>

// // Usar el namespace para las structs
// using namespace ur5_controller;

// namespace
// {
// std::string make_namespaced_path(const std::string& namespace_name, const std::string& resource_path)
// {
//     if (resource_path.empty()) {
//         return namespace_name.empty() ? std::string{} : std::string("/") + namespace_name;
//     }

//     if (resource_path.front() == '/') {
//         return namespace_name.empty() ? resource_path : ("/" + namespace_name + resource_path);
//     }

//     return namespace_name.empty() ? ("/" + resource_path) : ("/" + namespace_name + "/" + resource_path);
// }
// }  // namespace


// void initializeUR5(PinocchioResources& pinocchio, const std::string& urdf_path) {
//     pinocchio.model = std::make_unique<pinocchio::Model>();

//     auto logger = rclcpp::get_logger("UR5Kinematics");
//     RCLCPP_INFO(logger, "Intentando cargar URDF desde: %s", urdf_path.c_str());

//     try {
//         pinocchio::urdf::buildModel(urdf_path, *pinocchio.model);
//         RCLCPP_INFO(logger, "URDF cargado exitosamente!");
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(logger, "Error cargando URDF: %s", e.what());
//         throw;
//     }

//     pinocchio.data = std::make_unique<pinocchio::Data>(*pinocchio.model);
    
//     // Obtener frame de la base
//     pinocchio.base_frame_id = pinocchio.model->getFrameId("world");
//     if (pinocchio.base_frame_id == static_cast<pinocchio::FrameIndex>(pinocchio.model->nframes)) {
//         RCLCPP_WARN(logger, "Frame 'base_link' no encontrado, se usará el frame 0 (universe)");
//         pinocchio.base_frame_id = 0;
//     } else {
//         RCLCPP_INFO(logger, "Frame base 'base_link' encontrado con ID: %d", pinocchio.base_frame_id);
//     }
    
//     // Obtener frame del efector final
//     pinocchio.tool_frame_id = pinocchio.model->getFrameId("tool0");
//     if (pinocchio.tool_frame_id == static_cast<pinocchio::FrameIndex>(pinocchio.model->nframes)) {
//         RCLCPP_ERROR(logger, "Error: Marco 'tool0' no encontrado en el URDF!");
//         throw std::runtime_error("Frame tool0 no encontrado");
//     } else {
//         RCLCPP_INFO(logger, "Frame efector 'tool0' encontrado con ID: %d", pinocchio.tool_frame_id);
//     }
// }


// std::string get_file_path(const std::string& package_name, const std::string& relative_path) {
//     try {
//         std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
//         return package_share_directory + "/" + relative_path;
//     } catch (const std::exception& e) {
//         throw std::runtime_error("No se pudo encontrar el paquete: " + package_name);
//     }
// }



// class UR5IKNode : public rclcpp::Node
// {
// public:
//   UR5IKNode() : Node("ur5_ik_node")
//   {
//     // 1. Declarar parámetros
//     config_.use_geomagic = this->declare_parameter<bool>("geomagic", config_.use_geomagic);
//     config_.ur_model = this->declare_parameter<std::string>("ur", config_.ur_model);
//     config_.nmspace = this->declare_parameter<std::string>("nmspace", config_.nmspace);
//     std::string urdf_param = this->declare_parameter<std::string>("urdf_path", "");
//     std::string geomagic_topic = this->declare_parameter<std::string>("geomagic_topic", "/phantom/state");
//     std::string geomagic_button_topic = this->declare_parameter<std::string>("geomagic_button_topic", "/phantom/button");
//     config_.use_ur5_pos_init = this->declare_parameter<bool>("use_ur5_pos_init", config_.use_ur5_pos_init);
//     config_.q_target = this->declare_parameter<std::vector<double>>("q_target", config_.q_target);
//     config_.q_target_time = this->declare_parameter<double>("q_target_time", config_.q_target_time);
//     config_.csv_enabled = this->declare_parameter<bool>("csv_log_enable", config_.csv_enabled);
//     config_.csv_path = this->declare_parameter<std::string>("csv_log_dir", config_.csv_path);
//     config_.csv_prefix = this->declare_parameter<std::string>("csv_log_prefix", config_.csv_prefix);

//     std::vector<double> traj_A_param = this->declare_parameter<std::vector<double>>("traj_A", {config_.traj_A.x(), config_.traj_A.y(), config_.traj_A.z()});
//     config_.traj_wn = this->declare_parameter<double>("traj_wn", config_.traj_wn);
//     config_.traj_c0 = this->declare_parameter<double>("traj_c0", config_.traj_c0);
//     config_.traj_mode = this->declare_parameter<int>("traj_mode", config_.traj_mode);
//     config_.controller = this->declare_parameter<std::string>("controller_type", config_.controller);

//     // Parámetros para QP_vel controller
//     Kp_pos_vel = this->declare_parameter<double>("Kp_pos_vel", 1.5);
//     Kp_orient_vel = this->declare_parameter<double>("Kp_orient_vel", 0.5);

//     std::vector<double> Kp_param = this->declare_parameter<std::vector<double>>("Kp", {1850.0, 1850.0, 1850.0, 500.0, 500.0, 500.0, 5000.0});
//     std::vector<double> Kd_param = this->declare_parameter<std::vector<double>>("Kd", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
//     std::vector<double> param_lambda = this->declare_parameter<std::vector<double>>("lambda", {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
//     std::vector<double> param_k = this->declare_parameter<std::vector<double>>("k", {50.0, 50.0, 50.0, 50.0, 50.0, 50.0});
//     std::vector<double> param_k2 = this->declare_parameter<std::vector<double>>("k2", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0});

//     config_.control_topic = this->declare_parameter<std::string>("control_topic", config_.control_topic);
//     config_.alpha = this->declare_parameter<double>("alpha", 0.01);
//     config_.damping_factor = this->declare_parameter<double>("damping_factor", 0.01);
//     config_.dt = this->declare_parameter<double>("dt", 0.01);
//     config_.ctrl_hz_ = this->declare_parameter<double>("ctrl_hz", 500.0); // 125Hz para estabilidad
//     max_joint_step_rad_ = this->declare_parameter<double>("max_joint_step_rad", 0.05); // Paso máximo por ciclo
//     large_error_threshold_rad_ = this->declare_parameter<double>("large_error_threshold_rad", 0.15); // Umbral error grande
//     map_pos_ = {static_cast<int>(this->declare_parameter<double>("map_x", 2.0)),
//                 static_cast<int>(this->declare_parameter<double>("map_y", 0.0)),
//                 static_cast<int>(this->declare_parameter<double>("map_z", 1.0))};
//     sign_pos_ = {this->declare_parameter<double>("sign_x", -1.0),
//                  this->declare_parameter<double>("sign_y", -1.0),
//                  this->declare_parameter<double>("sign_z", 1.0)};
//     map_rot_ = {static_cast<int>(this->declare_parameter<double>("map_roll", 2.0)),
//                 static_cast<int>(this->declare_parameter<double>("map_pitch", 0.0)),
//                 static_cast<int>(this->declare_parameter<double>("map_yaw", 1.0))};
//     sign_rot_ = {this->declare_parameter<double>("sign_roll", 1.0),
//                  this->declare_parameter<double>("sign_pitch", 1.0),
//                  this->declare_parameter<double>("sign_yaw", 1.0)};

//     joint_state_mapper_.configure(config_.nmspace, this->get_logger());

//     if (traj_A_param.size() == 3) {
//         config_.traj_A = Eigen::Vector3d(traj_A_param[0], traj_A_param[1], traj_A_param[2]);
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Parametro traj_A debe tener tamaño 3. Usando valores por defecto.");
//     }
//     if (config_.traj_mode < 1 || config_.traj_mode > 3) {
//         RCLCPP_WARN(this->get_logger(), "traj_mode fuera de rango (%d). Se usará 1.", config_.traj_mode);
//         config_.traj_mode = 1;
//     }

//     if (Kp_param.size() == 7) {
//         for (int i = 0; i < 7; ++i) {
//             config_.Kp(i) = Kp_param[i];
//         }
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Parametro Kp debe tener tamaño 7. Usando valores por defecto.");
//         config_.Kp << 1850.0, 1850.0, 1850.0, 500.0, 500.0, 500.0, 500.0;
//     }

//     if (Kd_param.size() == 7) {
//         for (int i = 0; i < 7; ++i) {
//             config_.Kd(i) = Kd_param[i];
//         }
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Parametro Kd debe tener tamaño 7. Usando valores por defecto.");
//         config_.Kd << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
//     }
//     if (param_lambda.size() == 6) {
//         for (int i = 0; i < 6; ++i) {
//             config_.lambda(i) = param_lambda[i];
//         }
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Parametro lambda debe tener tamaño 6. Usando valores por defecto.");
//         config_.lambda << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
//     }
//     if (param_k.size() == 6) {
//         for (int i = 0; i < 6; ++i) {
//             config_.k(i) = param_k[i];
//         }
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Parametro k debe tener tamaño 6. Usando valores por defecto.");
//         config_.k << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
//     }
//     if (param_k2.size() == 6) {
//         for (int i = 0; i < 6; ++i) {
//             config_.k2(i) = param_k2[i];
//         }
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Parametro k2 debe tener tamaño 6. Usando valores por defecto.");
//         config_.k2 << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
//     }

//     if (urdf_param.empty()) {
//         config_.urdf_path = get_file_path("ur5_description", "urdf/" + config_.ur_model + ".urdf");
//     } else {
//         config_.urdf_path = urdf_param;
//     }

//     // Inicializar Pinocchio
//     initializeUR5(pinocchio_, config_.urdf_path);
//     kinematics_solver_ = std::make_unique<UR5Kinematics>(config_.urdf_path);
//     impedance_controller_ = std::make_unique<ur5_impedance::UR5Impedance>(config_.urdf_path);
//     sliding_controller_ = std::make_unique<ur5_sliding::UR5Sliding>(config_.urdf_path);
//     std::string forward_command_topic = make_namespaced_path(config_.nmspace, config_.control_topic);
//     std::string initial_trajectory_topic = make_namespaced_path(
//         config_.nmspace, "/scaled_joint_trajectory_controller/joint_trajectory");
//     std::string controller_manager_service = make_namespaced_path(
//         config_.nmspace, "/controller_manager/switch_controller");
//     std::string joint_states_topic = config_.nmspace.empty() ? std::string("/joint_states")
//                                                             : std::string("/") + config_.nmspace + std::string("/joint_states");

//     RCLCPP_INFO(this->get_logger(), "Publicando en el tópico de control: '%s'", forward_command_topic.c_str());
//     RCLCPP_INFO(this->get_logger(), "Trayectoria inicial en: '%s'", initial_trajectory_topic.c_str());
//     RCLCPP_INFO(this->get_logger(), "Servicio switch_controller: '%s'", controller_manager_service.c_str());
//     RCLCPP_INFO(this->get_logger(), "Usando modelo UR: '%s'", config_.ur_model.c_str());
//     RCLCPP_INFO(this->get_logger(), "Usando URDF en: '%s'", config_.urdf_path.c_str());
//     joint_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(forward_command_topic, 10);
    
//     // Publisher para velocidades (QP_vel controller)
//     std::string velocity_command_topic = make_namespaced_path(config_.nmspace, "/forward_velocity_controller/commands");
//     RCLCPP_INFO(this->get_logger(), "Publicando velocidades en: '%s'", velocity_command_topic.c_str());
//     joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(velocity_command_topic, 10);
    
//     initial_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(initial_trajectory_topic, 10);
//     auto switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>(controller_manager_service);
//     controller_switch_coordinator_ = std::make_unique<ControllerSwitchCoordinator>(
//         this->get_logger(), this->get_clock(), switch_controller_client);
//     joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_topic, 10, std::bind(&UR5IKNode::update_joint_positions, this, std::placeholders::_1));
//     RCLCPP_INFO(this->get_logger(), "Suscrito a joint_states: '%s'", joint_states_topic.c_str());
//     // Suscripción de respaldo al tópico global en caso de que el driver no use namespace
//     if (!config_.nmspace.empty() && joint_states_topic != "/joint_states") {
//         joint_states_sub_global_ = this->create_subscription<sensor_msgs::msg::JointState>(
//             "/joint_states", 10, std::bind(&UR5IKNode::update_joint_positions, this, std::placeholders::_1));
//         RCLCPP_INFO(this->get_logger(), "Suscripción de respaldo a joint_states: '/joint_states'");
//     }

//     if (config_.use_geomagic) {
//         geomagic_state_sub_ = this->create_subscription<omni_msgs::msg::OmniState>(geomagic_topic, 10, std::bind(&UR5IKNode::pose_callback, this, std::placeholders::_1));
//         RCLCPP_INFO(this->get_logger(), "Usando tópico de Geomagic state: '%s'", geomagic_topic.c_str());
//         controller_switch_coordinator_->setInitialForwardState();
//     }
//     else {
//         RCLCPP_INFO(this->get_logger(), "Modo Geomagic deshabilitado. Seguimiento de la trayectoria predefinida.");
//         RCLCPP_INFO(this->get_logger(), "Controlador seleccionado: '%s'", config_.controller.c_str());
//         controller_switch_coordinator_->setInitialForwardState();
//         // Configurar y inicializar el movimiento inicial si está habilitado
//         if (config_.use_ur5_pos_init) {
//             initial_motion_publisher_.configure(
//                 robot_state_,
//                 cartesian_state_,
//                 config_,
//                 initial_trajectory_pub_,
//                 joint_state_mapper_,
//                 pinocchio_.model.get(),
//                 pinocchio_.data.get(),
//                 pinocchio_.tool_frame_id
//             );
//             RCLCPP_INFO(this->get_logger(), "Initial motion configured. Waiting to switch to scaled controller.");
//         }
//         // Trayectoria automática se activará una vez termine movimiento inicial o inmediatamente si no se usa init
//         if (!config_.use_ur5_pos_init) {
//             trajectory_active_ = false; // se activará tras captura de pose
//         }
//     }

//     // Inicializar CSV si está habilitado
//     if (config_.csv_enabled) {
//         csv_logger_.configure(config_.csv_enabled, config_.csv_path, config_.csv_prefix, config_.nmspace);
//     }

//     // Inicializar gestor de parámetros y registrar callback para cambios en caliente
//     param_manager_ = std::make_unique<ParameterManager>(&config_, &map_pos_, &sign_pos_, &map_rot_, &sign_rot_, &dynamic_params_mutex_, this->get_logger());
//     params_cb_handle_ = this->add_on_set_parameters_callback(
//         std::bind(&ParameterManager::on_parameters_set, param_manager_.get(), std::placeholders::_1));

//     rclcpp::on_shutdown([this]() {
//         if (config_.controller == "QP_vel" && joint_velocity_pub_) {
//             std_msgs::msg::Float64MultiArray zero_msg;
//             zero_msg.data.assign(6, 0.0);
//             joint_velocity_pub_->publish(zero_msg);
//             RCLCPP_INFO(this->get_logger(), "Apagado seguro: publicando velocidades cero en QP_vel.");
//         }
//     });

//     last_successful_publish_ = std::chrono::steady_clock::now();
//     timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / config_.ctrl_hz_)), std::bind(&UR5IKNode::control_loop, this));
//   }

// private:
//     rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
//     std::mutex dynamic_params_mutex_;
//     std::unique_ptr<ParameterManager> param_manager_;

//     // Suscriptores y publicadores
//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_pub_;
//     rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr initial_trajectory_pub_;
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_global_;
//     rclcpp::Subscription<omni_msgs::msg::OmniState>::SharedPtr geomagic_state_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr geomagic_pose_stamped_sub_;
//     sensor_msgs::msg::JointState::SharedPtr last_joint_state_;  

//     // Estado previo de botones recibidos vía OmniState
//     bool haptic_close_prev_ = false;
//     bool haptic_locked_prev_ = false;

//     // --- Variables agrupadas en structs ---
//     RobotState robot_state_;
//     CartesianState cartesian_state_;
//     HapticState haptic_state_;
//     NodeConfig config_;
//     PinocchioResources pinocchio_;
//     // ------------------------------------

//     // Filtro simple para comandos articulares
//     Eigen::VectorXd q_cmd_filtered_ = Eigen::VectorXd::Zero(6);
//     bool q_cmd_initialized_ = false;
//     double q_cmd_alpha_ = 0.2; // 0-1, mas alto = menos filtrado
    
//     // Interpolación adaptativa para comandos alejados
//     Eigen::VectorXd q_target_interpolated_ = Eigen::VectorXd::Zero(6);
//     Eigen::VectorXd q_solution_prev_ = Eigen::VectorXd::Zero(6);  // Solución IK anterior
//     double max_joint_step_rad_ = 0.05; // Paso máximo por ciclo (ajustable según ctrl_hz_)
//     double large_error_threshold_rad_ = 0.15; // Umbral para detectar error grande
//     double max_ik_change_threshold_rad_ = 0.10; // Umbral para detectar cambio brusco en solución IK
    
//     // Parámetros para QP_vel controller
//     double Kp_pos_vel = 10.1;      // Ganancia proporcional de posición
//     double Kp_orient_vel = 10.1;   // Ganancia proporcional de orientación
//     std::string last_active_controller_ = "";  // Rastrear cambios de controlador

//     // Banderas de estado
//     bool pose_inicial_capturada_ = false;
//     bool posicion_inicial_alcanzada_ = false;
//     bool capturar_pose_inicial_haptico_ = false;

//     rclcpp::TimerBase::SharedPtr timer_;

//     std::unique_ptr<UR5Kinematics> kinematics_solver_;
//     std::unique_ptr<ur5_impedance::UR5Impedance> impedance_controller_;
//     std::unique_ptr<ur5_sliding::UR5Sliding> sliding_controller_;
//     std::unique_ptr<ControllerSwitchCoordinator> controller_switch_coordinator_;
//     bool initial_motion_initialized_ = false;
   

//     // ---- Mapeo robusto de joints por nombre ----
//     JointStateMapper joint_state_mapper_;

//     // ---- Motion initialization (movimiento inicial) ----
//     InitialMotionPublisher initial_motion_publisher_{this->get_logger()};

//     // ---- CSV logging ----
//     rclcpp::Time start_time_{};
//     rclcpp::Time last_log_time_{};
//     // Métricas de tiempo por ciclo
//     double last_ik_ms_ {0.0};
//     double last_loop_ms_ {0.0};
//     CsvLogger csv_logger_;
    
//     // === Watchdog para detectar delays y desconexiones ===
//     std::chrono::steady_clock::time_point last_successful_publish_;
//     int consecutive_loop_delays_ = 0;
//     int consecutive_publish_failures_ = 0;
//     static constexpr int MAX_CONSECUTIVE_DELAYS = 3;  // Advertencia después de 3 ciclos lentos
//     static constexpr int MAX_CONSECUTIVE_FAILURES = 10; // Error crítico después de 10 fallos
//     bool watchdog_enabled_ = true;
//     double loop_timeout_ms_ = 10.0; // Alerta si ciclo tarda más de 10ms (debería ser 2ms a 500Hz)

//     Eigen::Vector3d sign_pos_, sign_rot_;
//     Eigen::Vector3i map_pos_, map_rot_;

//     // ---- Trayectoria automática sin geomagic ----
//     // (Parámetros de trayectoria ahora en config_)
//     bool trajectory_active_ {false};
//     rclcpp::Time trajectory_start_time_ {};
//     double t_traj_ = 0.0; // Tiempo para la trayectoria, independiente de dt

//     // ---- Estimación robusta de derivadas (haptico y loop) ----
//     std::chrono::steady_clock::time_point last_haptic_sample_time_{};
//     bool haptic_timing_initialized_ = false;
//     std::chrono::steady_clock::time_point last_control_loop_time_{};
//     bool control_timing_initialized_ = false;
//     double derivative_lpf_alpha_ = 0.2; // [0,1], mayor = menos filtrado
//     bool haptic_state_initialized_ = false;

//     // Callback de JOINT STATES del UR5(e)
//     void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
//         last_joint_state_ = msg;
        
//         // Update the initial motion publisher with the latest joint state
//         initial_motion_publisher_.setLastJointState(msg);

//         if (!joint_state_mapper_.updateOrderedState(msg, robot_state_.q, robot_state_.qd)) {
//             return;
//         }

//         // Actualizar medición cartesiana actual (posición y orientación) del efector
//         pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
//         pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
//         const auto& frame_placement_now = pinocchio_.data->oMf[pinocchio_.tool_frame_id];

//         cartesian_state_.position = frame_placement_now.translation();
//         cartesian_state_.rotation_matrix = frame_placement_now.rotation();
//         cartesian_state_.orientation = Eigen::Quaterniond(frame_placement_now.rotation());

//         // Capturar la pose inicial automáticamente en la primera recepción
//         if (!pose_inicial_capturada_) {
//             RCLCPP_INFO(this->get_logger(), "Capturando la pose inicial actual del robot...");
            
//             robot_state_.q_init = robot_state_.q;
//             // Ya tenemos medición cartesiana actual en cartesian_state_ (actualizada más arriba
//             // mediante forwardKinematics con q), por lo que podemos reutilizarla para inicial.
//             cartesian_state_.position_initial = cartesian_state_.position;
//             cartesian_state_.rotation_matrix_initial = cartesian_state_.rotation_matrix;
//             cartesian_state_.orientation_initial = cartesian_state_.orientation;

//             pose_inicial_capturada_ = true;
//             posicion_inicial_alcanzada_ = true;
            
//             if (config_.use_geomagic) {
//                 capturar_pose_inicial_haptico_ = true;
//             }
            
//             RCLCPP_INFO(this->get_logger(), "Pose inicial capturada. El control está activo.");
//             RCLCPP_INFO(this->get_logger(), "q_init: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
//                         robot_state_.q_init[0], robot_state_.q_init[1], robot_state_.q_init[2], robot_state_.q_init[3], robot_state_.q_init[4], robot_state_.q_init[5]);
//             // Si no se usa movimiento inicial, arrancar trayectoria ahora
//             if (!config_.use_geomagic && !config_.use_ur5_pos_init && !initial_motion_publisher_.isActive()) {
//                 trajectory_start_time_ = this->now();
//                 trajectory_active_ = true;
//                 RCLCPP_INFO(this->get_logger(), "Trayectoria automática ACTIVADA desde t=0 tras captura de pose inicial (sin movimiento inicial)." );
//             }
//         }
//     }
    
//     // Callback del Geomagic Touch (recibe OmniState con posición en mm, velocidad en mm/s)
//     void pose_callback(const omni_msgs::msg::OmniState::SharedPtr msg) {
//         if (!msg) {
//             RCLCPP_ERROR(this->get_logger(), "Mensaje nulo recibido en /phantom/state.");
//             return;
//         }

//         double dt = 1.0 / std::max(1.0, config_.ctrl_hz_);
//         const auto now_sample = std::chrono::steady_clock::now();
//         if (haptic_timing_initialized_) {
//             dt = std::chrono::duration<double>(now_sample - last_haptic_sample_time_).count();
//             if (dt < 1e-5 || dt > 0.1) {
//                 dt = 1.0 / std::max(1.0, config_.ctrl_hz_);
//             }
//         }
//         last_haptic_sample_time_ = now_sample;
//         haptic_timing_initialized_ = true;
        
//         // Convertir posición de mm a m (dividir por 1000)
//         haptic_state_.position << msg->pose.position.x / 1000.0, 
//                                   msg->pose.position.y / 1000.0, 
//                                   msg->pose.position.z / 1000.0;
//         haptic_state_.orientation.w() = msg->pose.orientation.w;
//         haptic_state_.orientation.x() = msg->pose.orientation.x;
//         haptic_state_.orientation.y() = msg->pose.orientation.y;
//         haptic_state_.orientation.z() = msg->pose.orientation.z;
//         haptic_state_.orientation.normalize();
//         // Manejo de botones usando campos de OmniState
//         bool close_btn = msg->close_gripper;
//         bool locked_btn = msg->locked;
//         if (close_btn && capturar_pose_inicial_haptico_) {
//             RCLCPP_INFO(this->get_logger(), "Botón gris presionado: Capturando pose inicial del Geomagic.");
//             capturar_pose_inicial_haptico_ = false; // el boton solo sirve para capturar la pose inicial una vez
//             RCLCPP_INFO(this->get_logger(), "Pose inicial háptica capturada.");
//             haptic_state_.position_initial = haptic_state_.position;
//             haptic_state_.orientation_initial = haptic_state_.orientation;
//         }
//         if (close_btn && !haptic_close_prev_) {
//             RCLCPP_INFO(this->get_logger(), "Botón gris presionado");
//         }
//         if (locked_btn && !haptic_locked_prev_) {
//             RCLCPP_INFO(this->get_logger(), "Botón blanco presionado");
//         }
//         haptic_close_prev_ = close_btn;
//         haptic_locked_prev_ = locked_btn;
//         //std::cout<<"Haptic position (m): "<<haptic_state_.position.transpose()<<std::endl;
//         //std::cout<<"Haptic orientation (quat): "<<haptic_state_.orientation.coeffs().transpose()<<std::endl;
//         // Velocidad directa del driver: convertir de mm/s a m/s (dividir por 1000)
//         Eigen::Vector3d vel_raw;
//         vel_raw << msg->velocity.x / 1000.0, msg->velocity.y / 1000.0, msg->velocity.z / 1000.0;

//         if (!haptic_state_initialized_) {
//             haptic_state_.position_last = haptic_state_.position;
//             haptic_state_.orientation_last = haptic_state_.orientation;
//             haptic_state_.velocity = vel_raw;  // Inicializar con velocidad del driver
//             haptic_state_.velocity_last = vel_raw;
//             haptic_state_.acceleration.setZero();
//             haptic_state_.angular_velocity.setZero(); 
//             haptic_state_.angular_acceleration.setZero();
//             haptic_state_.angular_velocity_last.setZero();
//             haptic_state_initialized_ = true;
//             return;
//         }

//         // Usar velocidad directa del driver con filtro LPF
//         haptic_state_.velocity = derivative_lpf_alpha_ * vel_raw + (1.0 - derivative_lpf_alpha_) * haptic_state_.velocity;
        
//         // Aceleración lineal: derivar velocidad filtrada
//         Eigen::Vector3d acc_raw = (haptic_state_.velocity - haptic_state_.velocity_last) / dt;
//         haptic_state_.acceleration = derivative_lpf_alpha_ * acc_raw + (1.0 - derivative_lpf_alpha_) * haptic_state_.acceleration;
//         haptic_state_.velocity_last = haptic_state_.velocity;
        
//         // Velocidad angular: derivar orientación (quaternion)
//         Eigen::Quaterniond delta_orientation = haptic_state_.orientation * haptic_state_.orientation_last.inverse();
//         delta_orientation.normalize();
//         if (delta_orientation.w() < 0.0) { delta_orientation.coeffs() *= -1.0; }
//         Eigen::AngleAxisd delta_aa(delta_orientation);
//         Eigen::Vector3d omega_raw = Eigen::Vector3d::Zero();
//         if (std::abs(delta_aa.angle()) > 1e-6) { omega_raw = delta_aa.axis() * (delta_aa.angle() / dt); }
        
//         haptic_state_.angular_velocity = derivative_lpf_alpha_ * omega_raw + (1.0 - derivative_lpf_alpha_) * haptic_state_.angular_velocity;
//         haptic_state_.orientation_last = haptic_state_.orientation;

//         // Aceleración angular: derivar velocidad angular
//         Eigen::Vector3d alpha_raw = (haptic_state_.angular_velocity - haptic_state_.angular_velocity_last) / dt;
//         haptic_state_.angular_acceleration = derivative_lpf_alpha_ * (alpha_raw) + (1.0 - derivative_lpf_alpha_) * haptic_state_.angular_acceleration;
//         haptic_state_.angular_velocity_last = haptic_state_.angular_velocity;

//     }

//     // button_callback removed: button events handled via OmniState in pose_callback

    
//     void control_loop() {
//         auto loop_t0 = std::chrono::steady_clock::now();
//         double loop_dt = 1.0 / std::max(1.0, config_.ctrl_hz_);
//         if (control_timing_initialized_) {
//             loop_dt = std::chrono::duration<double>(loop_t0 - last_control_loop_time_).count();
//             if (loop_dt < 1e-5 || loop_dt > 0.1) {
//                 loop_dt = 1.0 / std::max(1.0, config_.ctrl_hz_);
//             }
//         }
//         last_control_loop_time_ = loop_t0;
//         control_timing_initialized_ = true;
//         const double controller_dt = std::max(1e-4, loop_dt);
        
//         try {
//             // Copiar parámetros dinámicos a locales (para aplicar cambios en caliente)
//             Eigen::Vector3i map_pos_local;
//             Eigen::Vector3d sign_pos_local;
//             Eigen::Vector3i map_rot_local;
//             Eigen::Vector3d sign_rot_local;
//             {
//                 std::scoped_lock<std::mutex> lock(dynamic_params_mutex_);
//                 map_pos_local = map_pos_;
//                 sign_pos_local = sign_pos_;
//                 map_rot_local = map_rot_;
//                 sign_rot_local = sign_rot_;
//             }

//             if (!pose_inicial_capturada_ || !posicion_inicial_alcanzada_) {
//                 return;
//             }
//             if (config_.use_geomagic && capturar_pose_inicial_haptico_) {
//                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                     "Esperando a que se capture la pose inicial del Geomagic. Presione el botón gris.");
//                 return;
//             }
//             // if (!config_.use_geomagic && config_.use_ur5_pos_init && !controller_switch_coordinator_->scaledActive() && !config_.initial_motion_done) {
//             //     controller_switch_coordinator_->requestScaledIfNeeded([this]() {
//             //         if (!initial_motion_initialized_) {
//             //             initial_motion_publisher_.initialize(this, config_.ctrl_hz_);
//             //             initial_motion_initialized_ = true;
//             //             config_.initial_motion_done = false;
//             //         }
//             //     });
//             //     return;
//             // }
//             // if (!config_.use_geomagic && config_.use_ur5_pos_init && config_.initial_motion_done) {
//             //     if (config_.controller == "QP_vel") {
//             //         //std::cout << "QP_vel controller: verificando switch a velocity..." << std::endl;
//             //         if (!controller_switch_coordinator_->velocityActive()) {
//             //             std::cout << "QP_vel controller: switch a velocity solicitado." << std::endl;
//             //             controller_switch_coordinator_->requestVelocityIfNeeded([this]() {
//             //                 trajectory_active_ = true;
//             //                 trajectory_start_time_ = this->now();
//             //                 t_traj_ = 0.0;
//             //             });return;
                        
//             //         }
//             //         // 2. Si el controlador YA ESTÁ activo, liberamos la bandera para salir de este estado
//             //         std::cout << "QP_vel controller: switch a velocity activo. Iniciando control..." << std::endl;
//             //         config_.initial_motion_done = false;
                    
//             //     }
//             //     else{
//             //         if (!controller_switch_coordinator_->forwardActive()) {
//             //             controller_switch_coordinator_->requestForwardIfNeeded([this]() {
//             //                 trajectory_active_ = true;
//             //                 trajectory_start_time_ = this->now();
//             //                 t_traj_ = 0.0;
//             //             });
//             //         }
//             //         return;
//             //     }
//             //     RCLCPP_INFO(this->get_logger(), "Controlador posicional activo. Iniciando control...");
//             //     config_.initial_motion_done = false;

                
//             // }
//             // if (!config_.use_geomagic && config_.use_ur5_pos_init && controller_switch_coordinator_->scaledActive() && !config_.initial_motion_done) {
//             //     // Mientras scaled ejecuta el movimiento inicial, no entrar al bucle de control principal.
//             //     return;
//             // }
//             // === MÁQUINA DE ESTADOS DE CONTROLADORES ===
//             if (!config_.use_geomagic && config_.use_ur5_pos_init) {
                
//                 // 1. FASE DE MOVIMIENTO INICIAL (Aún no termina)
//                 if (!config_.initial_motion_done) {
//                     if (!controller_switch_coordinator_->scaledActive()) {
//                         controller_switch_coordinator_->requestScaledIfNeeded([this]() {
//                             if (!initial_motion_initialized_) {
//                                 initial_motion_publisher_.initialize(this, config_.ctrl_hz_);
//                                 initial_motion_initialized_ = true;
//                             }
//                         });
//                         return; // Esperamos a que se active el controlador scaled
//                     }
//                     // Si ya está activo pero no ha terminado, retornamos para que el publisher avance
//                     return; 
//                 }
                
//                 // 2. FASE DE CONTROL PRINCIPAL (Movimiento inicial completado)
//                 // Si llegamos aquí, config_.initial_motion_done es TRUE.
//                 // NUNCA lo volvemos a poner en FALSE, de lo contrario reiniciará el ciclo.
                
//                 if (config_.controller == "QP_vel") {
//                     if (!controller_switch_coordinator_->velocityActive()) {
//                         controller_switch_coordinator_->requestVelocityIfNeeded([this]() {
//                             // Al completar el switch, encendemos la trayectoria
//                             trajectory_active_ = true;
//                             trajectory_start_time_ = this->now();
//                             t_traj_ = 0.0;
//                             std::cout << "QP_vel: Switch a velocity activo. Iniciando trayectoria..." << std::endl;
//                         });
//                         return; // Esperamos a que el switch_controller responda
//                     }
//                 } 
//                 else { // Posicional (QP, IMP, SLD)
//                     if (!controller_switch_coordinator_->forwardActive()) {
//                         controller_switch_coordinator_->requestForwardIfNeeded([this]() {
//                             trajectory_active_ = true;
//                             trajectory_start_time_ = this->now();
//                             t_traj_ = 0.0;
//                             RCLCPP_INFO(this->get_logger(), "Posicional: Switch activo. Iniciando trayectoria...");
//                         });
//                         return; // Esperamos a que el switch_controller responda
//                     }
//                 }
//             }
//             // Rama geomagic: seguir referencia del háptico
//             if (config_.use_geomagic) {
//                 Eigen::Vector3d desired_vel_ori_cmd = Eigen::Vector3d::Zero();
//                 Eigen::Vector3d desired_acc_ori_cmd = Eigen::Vector3d::Zero();
//                 cartesian_state_.rotation_matrix_desired = cartesian_state_.rotation_matrix_initial;

//                 Eigen::Quaterniond dif_orientacion_haptic = haptic_state_.orientation_initial.inverse() * haptic_state_.orientation;
//                 Eigen::AngleAxisd angle_axis(dif_orientacion_haptic);
//                 Eigen::Vector3d axis_raw =  angle_axis.axis() * angle_axis.angle();
//                 double escala = 0.5; // factor de orientación
//                 Eigen::Vector3d axis_map;
//                 for(int i=0; i<3; i++) axis_map(i) = axis_raw(map_rot_local(i)) * sign_rot_local(i) * escala;
//                 if (axis_map.norm() > 1e-6){
//                     Eigen::Matrix3d R_delta = Eigen::AngleAxisd(axis_map.norm(), axis_map.normalized()).toRotationMatrix();
//                     cartesian_state_.rotation_matrix_desired = cartesian_state_.rotation_matrix_initial * R_delta;
//                 }
                
//                 // dif_orientacion_haptic = Eigen::Quaterniond(Eigen::AngleAxisd(escala * angle_axis.angle(), angle_axis.axis()));
//                 // cartesian_state_.orientation_desired = cartesian_state_.orientation_initial * dif_orientacion_haptic;

//                 RCLCPP_DEBUG(this->get_logger(), "Dif orientación háptico (eje): [%.3f, %.3f, %.3f]",
//                             dif_orientacion_haptic.vec().x(), dif_orientacion_haptic.vec().y(), dif_orientacion_haptic.vec().z());
                
//                 for(int i=0; i<3; i++){
//                     cartesian_state_.position_desired(i) = cartesian_state_.position_initial(i) +
//                         (haptic_state_.position(map_pos_local(i)) - haptic_state_.position_initial(map_pos_local(i))) * sign_pos_local(i)*2.5;     
//                 }
                

//                 cartesian_state_.velocity = haptic_state_.velocity * 2.5; // escala de velocidad
//                 cartesian_state_.acceleration = haptic_state_.acceleration * 2.5; // escala de aceleración
//                 for (int i = 0; i < 3; ++i) {
//                     desired_vel_ori_cmd(i) = haptic_state_.angular_velocity(map_rot_local(i)) * sign_rot_local(i) * escala;
//                     desired_acc_ori_cmd(i) = haptic_state_.angular_acceleration(map_rot_local(i)) * sign_rot_local(i) * escala;
//                 }
//                 cartesian_state_.angular_velocity << desired_vel_ori_cmd;
                
//                 cartesian_state_.angular_acceleration = desired_acc_ori_cmd;
                
//                 // Los controladores usan estas referencias de orientación en 3D
//                 cartesian_state_.orientation_desired = Eigen::Quaterniond(cartesian_state_.rotation_matrix_desired);
//                 cartesian_state_.orientation_desired.normalize();
                    
                
//             } else {// Rama automática: generar trayectoria paramétrica
                
//                 if (!trajectory_active_) {
//                     // Si la trayectoria no está activa, simplemente retornamos y esperamos.
//                     // Ya sea la captura de pose inicial (línea ~301) o el callback del switch 
//                     // (código arriba) se encargarán de encenderla cuando sea el momento adecuado.
//                     return; 
//                 }
//                 auto st = TrajectoryGenerator::calculate(
//                     cartesian_state_.position_initial,
//                     config_.traj_A,
//                     config_.traj_wn,
//                     config_.traj_c0,
//                     t_traj_,
//                     config_.traj_mode
//                 );
                
//                 cartesian_state_.position_desired = st.position;
//                 cartesian_state_.velocity = st.velocity;
//                 cartesian_state_.angular_velocity = Eigen::Vector3d::Zero(); // sin rot
//                 cartesian_state_.orientation_desired = cartesian_state_.orientation_initial;
//                 cartesian_state_.rotation_matrix_desired = cartesian_state_.rotation_matrix_initial;
//                 cartesian_state_.acceleration = st.acceleration;
//                 cartesian_state_.angular_acceleration = Eigen::Vector3d::Zero();
//                 t_traj_ += loop_dt;

//                 RCLCPP_DEBUG(this->get_logger(), "x_des: [%.3f, %.3f, %.3f]",
//                     cartesian_state_.position_desired.x(), cartesian_state_.position_desired.y(), cartesian_state_.position_desired.z());
//                 RCLCPP_DEBUG(this->get_logger(), "vel_des: [%.3f, %.3f, %.3f]",
//                     cartesian_state_.velocity.x(), cartesian_state_.velocity.y(), cartesian_state_.velocity.z());
//             }

//             Eigen::Vector3d desired_vel_ori_ctrl = config_.use_geomagic ? cartesian_state_.angular_velocity : Eigen::Vector3d::Zero();

//             Eigen::Vector3d desired_acc_ori_ctrl = config_.use_geomagic ? cartesian_state_.angular_acceleration : Eigen::Vector3d::Zero();
//             // Medición cartesiana actual (para logging y control si fuera necesario)
//             pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
//             pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);

//             const auto& frame_meas = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
//             cartesian_state_.position = frame_meas.translation();
//             RCLCPP_DEBUG(this->get_logger(), "x_meas: [%.3f, %.3f, %.3f]",
//             cartesian_state_.position.x(), cartesian_state_.position.y(), cartesian_state_.position.z());
//             Eigen::Matrix3d R_meas = frame_meas.rotation();
//             cartesian_state_.orientation = Eigen::Quaterniond(R_meas);

//             Eigen::Matrix3d R_des = cartesian_state_.orientation_desired.toRotationMatrix();
//             Eigen::Matrix3d R_err = R_meas.transpose() * R_des;
//             double e_R_angle = Eigen::AngleAxisd(R_err).angle();
//             // Quaterniones y Euler (roll-pitch-yaw) deseados y medidos
//             Eigen::Quaterniond q_des = cartesian_state_.orientation_desired;
//             Eigen::Quaterniond q_meas(R_meas);
//             Eigen::Vector3d euler_des = R_des.eulerAngles(0, 1, 2); // RPY
//             Eigen::Vector3d euler_meas = R_meas.eulerAngles(0, 1, 2); // RPY

//             // === MANEJO DE CAMBIO DE CONTROLADORES ===
//             // Si el tipo de controlador cambió, hacer el switch correspondiente
//             if (config_.controller != last_active_controller_) {
//                 if (config_.controller == "QP_vel") {
//                     // Cambiar a forward_velocity_controller
//                     controller_switch_coordinator_->requestVelocityIfNeeded([this]() {
//                         RCLCPP_INFO(this->get_logger(), "Controlador cambió a QP_vel (velocity)");
//                     });
//                 } else if (last_active_controller_ == "QP_vel") {
//                     // Cambiar de velocity a position controller (QP, IMP, SLD)
//                     controller_switch_coordinator_->requestForwardIfNeeded([this]() {
//                         RCLCPP_INFO(this->get_logger(), "Controlador cambió de QP_vel a position-based");
//                     });
//                 } else if (config_.controller != "QP" && config_.controller != "IMP" && config_.controller != "SLD") {
//                     // Controlador desconocido, cambiar a forward_position como fallback
//                     controller_switch_coordinator_->requestForwardIfNeeded([this]() {
//                         RCLCPP_INFO(this->get_logger(), "Controlador desconocido, usando forward_position como fallback");
//                     });
//                 }
//                 last_active_controller_ = config_.controller;
//             }

//             auto ik_t0 = std::chrono::steady_clock::now();

//             if (config_.controller == "QP") {
//                 robot_state_.q_solution = kinematics_solver_->inverseKinematicsQP2(
//                     robot_state_.q,
//                     cartesian_state_.position_desired,
//                     cartesian_state_.rotation_matrix_desired,
//                     600,
//                     config_.ctrl_hz_
//                 );
//                 robot_state_.u_control = Eigen::VectorXd::Zero(6); // QP no calcula tau
//             }
//             else if (config_.controller == "QP_vel") {
//                 std::cout<<"Controlador QP_vel activo. Calculando comando de velocidad..."<<std::endl;
//                 // Controlador de velocidad usando computeVelocityControlStep
//                 robot_state_.dq_command = kinematics_solver_->computeVelocityControlStep(
//                     robot_state_.q,
//                     cartesian_state_.position_desired,
//                     cartesian_state_.rotation_matrix_desired,
//                     cartesian_state_.velocity,           // NUEVO: Feedforward lineal (\dot{r} lineal)
//                     cartesian_state_.angular_velocity,
//                     Kp_pos_vel,  // Kp_pos
//                     Kp_orient_vel,  // Kp_orient
//                     controller_dt
//                 );
//                 robot_state_.u_control = Eigen::VectorXd::Zero(6); // QP_vel no calcula tau
//             }
//             else if (config_.controller == "IMP") {
//                 auto output = impedance_controller_->calculateControlCommand(
//                     robot_state_.q,
//                     robot_state_.qd,
//                     cartesian_state_.position_desired,
//                     cartesian_state_.rotation_matrix_desired, //3x3 orientation_desired,
//                     cartesian_state_.velocity,
//                     desired_vel_ori_ctrl,
//                     cartesian_state_.acceleration,
//                     desired_acc_ori_ctrl,
//                     config_.Kp.head<6>(),
//                     config_.Kd.head<6>(),
//                     controller_dt);

//                 robot_state_.q_solution = output.q_desired;
//                 robot_state_.u_control = output.tau;
//             }
//             else if (config_.controller == "SLD") {                
//                 // Eigen::Matrix<double, 6, 1> lambda = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
//                 // Eigen::Matrix<double, 6, 1> k = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
//                 // Eigen::Matrix<double, 6, 1> k2 = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
//                 // double alpha = 0.01;
//                 // double damping_factor = 0.01;
//                 // double dt = 0.01;
//                 auto output = sliding_controller_->calculateControlCommand(
//                     robot_state_.q,
//                     robot_state_.qd,
//                     cartesian_state_.position_desired,
//                     cartesian_state_.rotation_matrix_desired,
//                     cartesian_state_.velocity,
//                     desired_vel_ori_ctrl,
//                     cartesian_state_.acceleration,
//                     desired_acc_ori_ctrl,
//                     config_.lambda,
//                     config_.k,
//                     config_.k2,
//                     config_.alpha,
//                     config_.damping_factor,
//                     controller_dt);
                    
//                 robot_state_.q_solution = output.q_desired;
//                 robot_state_.u_control = output.tau;
//             }
//             else {
//                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                     "Controlador desconocido: %s. Usando QP.", config_.controller.c_str());
//                 config_.controller = "QP";
//                 robot_state_.q_solution = kinematics_solver_->inverseKinematicsQP2(
//                     robot_state_.q,
//                     cartesian_state_.position_desired,
//                     cartesian_state_.rotation_matrix_desired,
//                     600,
//                     config_.ctrl_hz_
//                 );
//                 robot_state_.u_control = Eigen::VectorXd::Zero(6);
//             }
        
//             last_ik_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(
//                                std::chrono::steady_clock::now() - ik_t0)
//                                .count() / 1000.0;
//             //RCLCPP_INFO(this->get_logger(), "Tiempo IK: %.3f ms", last_ik_ms_);

//             // === INTERPOLACIÓN ADAPTATIVA PARA COMANDOS ALEJADOS ===
//             // Inicializar target interpolado en el primer ciclo
//             if (!q_cmd_initialized_) {
//                 q_target_interpolated_ = robot_state_.q;
//                 q_solution_prev_ = robot_state_.q_solution;
//             }
            
//             // Detectar cambios bruscos en la solución IK (operador movió háptico nuevamente)
//             Eigen::VectorXd ik_change = robot_state_.q_solution - q_solution_prev_;
//             double max_ik_change = ik_change.cwiseAbs().maxCoeff();
//             bool ik_changed_abruptly = (max_ik_change > max_ik_change_threshold_rad_);
            
//             // Calcular error entre posición actual y solución IK
//             Eigen::VectorXd q_error = robot_state_.q_solution - robot_state_.q;
//             double max_error = q_error.cwiseAbs().maxCoeff();
            
//             // Si hay un error grande (movimiento brusco del háptico)
//             if (max_error > large_error_threshold_rad_) {
//                 // Si la solución IK cambió drásticamente, reiniciar interpolación
//                 if (ik_changed_abruptly) {
//                     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
//                         "Cambio brusco en solución IK detectado (%.3f rad). Reiniciando interpolación.",
//                         max_ik_change);
//                     q_target_interpolated_ = robot_state_.q;  // Reiniciar desde posición actual
//                 } else {
//                     // Interpolar gradualmente hacia la solución alejada
//                     Eigen::VectorXd direction = (robot_state_.q_solution - q_target_interpolated_).normalized();
//                     double remaining_distance = (robot_state_.q_solution - q_target_interpolated_).norm();
                    
//                     // Avanzar un paso limitado hacia el objetivo
//                     double step_size = std::min(max_joint_step_rad_, remaining_distance);
//                     q_target_interpolated_ += direction * step_size;
                    
//                     RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
//                         "Error articular grande detectado: %.3f rad. Interpolando suavemente (restante: %.3f rad)",
//                         max_error, remaining_distance);
//                 }
//             } else {
//                 // Error pequeño: seguir normalmente la solución IK
//                 q_target_interpolated_ = robot_state_.q_solution;
//             }
            
//             // Guardar solución IK actual para próximo ciclo
//             q_solution_prev_ = robot_state_.q_solution;
//             RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                 "q_actual: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
//                 robot_state_.q[0], robot_state_.q[1], robot_state_.q[2], robot_state_.q[3], robot_state_.q[4], robot_state_.q[5]);
//             RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                 "q_solution: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
//                 robot_state_.q_solution[0], robot_state_.q_solution[1], robot_state_.q_solution[2], robot_state_.q_solution[3], robot_state_.q_solution[4], robot_state_.q_solution[5]);    
//         // --------------------------------------------------------------------------------
        
//             // Filtro pasabajo sobre el target interpolado (no sobre q_solution directamente)
//             if (!q_cmd_initialized_) {
//                 q_cmd_filtered_ = q_target_interpolated_;
//                 q_cmd_initialized_ = true;
//             } else {
//                 q_cmd_filtered_ = q_cmd_alpha_ * q_target_interpolated_ + (1.0 - q_cmd_alpha_) * q_cmd_filtered_;
//             }

//             // === PUBLICACIÓN CON GARANTÍA ===
//             // Siempre publicar para mantener heartbeat con el driver
//             try {
//                 auto position_msg = std_msgs::msg::Float64MultiArray();
                
//                 // Seleccionar comando basado en tipo de controlador
//                 if (config_.controller == "QP_vel") {
//                     // Publicar velocidades para el controlador de velocidad
//                     position_msg.data.assign(robot_state_.dq_command.data(), 
//                                             robot_state_.dq_command.data() + robot_state_.dq_command.size());
//                     joint_velocity_pub_->publish(position_msg);
                    
//                     RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                         "Publicando velocidades (QP_vel): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
//                         robot_state_.dq_command(0), robot_state_.dq_command(1), robot_state_.dq_command(2),
//                         robot_state_.dq_command(3), robot_state_.dq_command(4), robot_state_.dq_command(5));
//                 } else {
//                     // Publicar posiciones para controladores de posición (QP, IMP, SLD)
//                     position_msg.data.assign(q_cmd_filtered_.data(), 
//                                             q_cmd_filtered_.data() + q_cmd_filtered_.size());
//                     joint_position_pub_->publish(position_msg);
                    
//                     RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                         "Publicando posiciones: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
//                         q_cmd_filtered_(0), q_cmd_filtered_(1), q_cmd_filtered_(2),
//                         q_cmd_filtered_(3), q_cmd_filtered_(4), q_cmd_filtered_(5));
//                 }
                
//                 // Registro exitoso de publicación
//                 consecutive_publish_failures_ = 0;
//                 last_successful_publish_ = std::chrono::steady_clock::now();
//             } catch (const std::exception& e) {
//                 consecutive_publish_failures_++;
//                 RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                     "Error publicando comando: %s (fallos consecutivos: %d)",
//                     e.what(), consecutive_publish_failures_);
                
//                 if (consecutive_publish_failures_ >= MAX_CONSECUTIVE_FAILURES) {
//                     RCLCPP_FATAL(this->get_logger(),
//                         "Demasiados fallos consecutivos de publicación (%d). Revisar conexión ethernet.",
//                         consecutive_publish_failures_);
//                 }
//             }

//             // Tiempo total del ciclo (hasta después de publicar)
//             last_loop_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(
//                         std::chrono::steady_clock::now() - loop_t0)
//                         .count() / 1000.0;
//             //std::cout<<"Tiempo total del ciclo de control: "<<last_loop_ms_<<" ms"<<std::endl;
//             // === WATCHDOG: Detectar delays del loop ===
//             if (last_loop_ms_ > loop_timeout_ms_) {
//                 consecutive_loop_delays_++;
//                 if (consecutive_loop_delays_ >= MAX_CONSECUTIVE_DELAYS) {
//                     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                         "Loop lento detectado: %.2f ms (esperado <2ms a 500Hz). Fallos consecutivos: %d. "
//                         "Posible causa: IK solver, FK, logging CSV, o saturación de CPU.",
//                         last_loop_ms_, consecutive_loop_delays_);
//                 }
//             } else {
//                 consecutive_loop_delays_ = 0;
//             }

//             // === Logging de timeout ===
//             auto now = std::chrono::steady_clock::now();
//             double ms_since_last_publish = std::chrono::duration<double, std::milli>(
//                 now - last_successful_publish_).count();
//             if (ms_since_last_publish > 100.0) {
//                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                     "ADVERTENCIA: %.0f ms sin publicación exitosa. Driver UR podría desconectarse.",
//                     ms_since_last_publish);
//             }

//             RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                 "[Control Loop] ik: %.3f ms | loop: %.3f ms | publish: OK",
//                 last_ik_ms_, last_loop_ms_);

//             // Logging CSV (sin bloqueo, en thread de fondo ideal)
//             if (csv_logger_.isEnabled()) {
//                 try {
//                     rclcpp::Time now_ros = this->now();
//                     if (start_time_.nanoseconds() == 0) {
//                         start_time_ = now_ros;
//                         last_log_time_ = now_ros;
//                     }
//                     const double t = (now_ros - start_time_).seconds();
//                     const double dt = (now_ros - last_log_time_).seconds();
//                     last_log_time_ = now_ros;

//                     CsvRow row;
//                     row.t = t;
//                     row.dt = dt;
//                     row.q_meas_joints = robot_state_.q;
//                     row.q_cmd = robot_state_.q_solution;
//                     row.x_des = cartesian_state_.position_desired;
//                     row.x_meas = cartesian_state_.position;
//                     row.q_des = q_des;
//                     row.q_meas_pose = cartesian_state_.orientation;
//                     row.euler_des = euler_des;
//                     row.euler_meas = euler_meas;
//                     row.e_R_angle = e_R_angle;
//                     row.pos_err = cartesian_state_.position_desired - cartesian_state_.position;
//                     Eigen::Quaterniond q_err = q_des * cartesian_state_.orientation.inverse();
//                     Eigen::AngleAxisd aa(q_err);
//                     row.ori_err_axis = aa.axis();
//                     row.ori_err_angle = aa.angle();
//                     row.u_control = robot_state_.u_control;
//                     row.ik_ms = last_ik_ms_;
//                     row.loop_ms = last_loop_ms_;

//                     RCLCPP_DEBUG(this->get_logger(), "pos_err: [%.3f, %.3f, %.3f]", row.pos_err.x(), row.pos_err.y(), row.pos_err.z());
//                     csv_logger_.writeRow(row);
//                 } catch (const std::exception& e) {
//                     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                         "Error escribiendo CSV: %s", e.what());
//                 }
//             }
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(),
//                 "Excepción en control_loop: %s. Continuando con último comando válido.", e.what());
//             // Intentar publicar última posición conocida para mantener conexión
//             try {
//                 auto position_msg = std_msgs::msg::Float64MultiArray();
//                 position_msg.data.assign(q_cmd_filtered_.data(), q_cmd_filtered_.data() + q_cmd_filtered_.size());
//                 joint_position_pub_->publish(position_msg);
//             } catch (...) {
//                 RCLCPP_ERROR(this->get_logger(), "No se puede publicar comando. Desconexión probable.");
//             }
//         }
//     }
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<UR5IKNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


// // ros2 run ur5_controller controller_backup --ros-args -p control_topic:="/scaled_joint_trajectory_controller/joint_trayectory" -p ur:="ur5e" -p nmspace:="ur5e" -p geomagic_topic:="/phantom2/pose" -p geomagic_button_topic:="/phantom2/button" -p csv_log_enable:="true" -p geomagic:="true"



#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody.hpp>
//#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial.hpp> // Necesario para pinocchio::log6
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Sparse>

#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>

#include <ur5_controller/joint_state_mapper.hpp>

using namespace std::chrono_literals;
using Eigen::MatrixXd;
using Eigen::VectorXd;


class TrajectoryGenerator
{
public:
  struct State
  {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
  };

    static State calculate(
    const Eigen::Vector3d& x_init,
    const Eigen::Vector3d& A,
    double wn,
    double c0,
    double time_elapsed,
    int grafica)
    {
        State state;
        const double t = time_elapsed;

        const double exp_neg_c0_t = std::exp(-c0 * t);
        const double sin_wn_t = std::sin(wn * t);
        const double cos_wn_t = std::cos(wn * t);

        const double amp_factor = 1.0 - exp_neg_c0_t;
        const double d_amp_factor_dt = c0 * exp_neg_c0_t;
        const double d2_amp_factor_dt2 = -c0 * c0 * exp_neg_c0_t;

        if (grafica == 1) {
            state.position.x() = x_init.x() + A.x() * amp_factor * sin_wn_t;
            state.position.y() = x_init.y() + A.y() * amp_factor * cos_wn_t;
            state.position.z() = x_init.z() + A.z() * amp_factor * sin_wn_t;

            state.velocity.x() = A.x() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);
            state.velocity.y() = A.y() * (d_amp_factor_dt * cos_wn_t - amp_factor * wn * sin_wn_t);
            state.velocity.z() = A.z() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);

            const double term1_sin = d2_amp_factor_dt2 - amp_factor * wn * wn;
            const double term2_cos = 2.0 * d_amp_factor_dt * wn;
            state.acceleration.x() = A.x() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
            state.acceleration.y() = A.y() * (term1_sin * cos_wn_t - term2_cos * sin_wn_t);
            state.acceleration.z() = A.z() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
        } else if (grafica == 2) {
            state.position.x() = x_init.x() + A.x() - A.x() * exp_neg_c0_t;
            state.position.y() = x_init.y() + A.y() - A.y() * exp_neg_c0_t;
            state.position.z() = x_init.z() + A.z() - A.z() * exp_neg_c0_t;

            state.velocity.x() = A.x() * (-c0) * exp_neg_c0_t;
            state.velocity.y() = A.y() * (-c0) * exp_neg_c0_t;
            state.velocity.z() = A.z() * (-c0) * exp_neg_c0_t;

            state.acceleration.x() = A.x() * (c0 * c0) * exp_neg_c0_t;
            state.acceleration.y() = A.y() * (c0 * c0) * exp_neg_c0_t;
            state.acceleration.z() = A.z() * (c0 * c0) * exp_neg_c0_t;
        } else if (grafica == 3) {
            const double r_x = A.x();
            const double r_y = A.y();

            // Start the circle from the current point instead of jumping to the perimeter.
            state.position.x() = x_init.x() + r_x * (cos_wn_t - 1.0);
            state.position.y() = x_init.y() + r_y * sin_wn_t;
            state.position.z() = x_init.z();

            state.velocity.x() = -r_x * wn * sin_wn_t;
            state.velocity.y() = r_y * wn * cos_wn_t;
            state.velocity.z() = 0.0;

            state.acceleration.x() = -r_x * wn * wn * cos_wn_t;
            state.acceleration.y() = -r_y * wn * wn * sin_wn_t;
            state.acceleration.z() = 0.0;
        }

        return state;
    };
};

class UR5eDifferentialIKNode : public rclcpp::Node {
public:
    UR5eDifferentialIKNode() : Node("ur5e_differential_ik_node") {
        // Declarar parámetros
        this->declare_parameter<std::string>("urdf_path", "");
        this->declare_parameter<double>("damping_factor", 0.01);
        this->declare_parameter<double>("velocity_scaling", 0.5);
        this->declare_parameter<int>("kinematic_output_mode", 1);
        this->declare_parameter<double>("orientation_error_switch_rad", 0.25);
        this->declare_parameter<bool>("use_trajectory_generator", false);
        this->declare_parameter<std::vector<double>>("trajectory_A", {0.05, 0.05, 0.0});
        this->declare_parameter<double>("trajectory_wn", 0.6);
        this->declare_parameter<double>("trajectory_c0", 0.2);
        this->declare_parameter<int>("trajectory_mode", 1);
        this->declare_parameter<std::vector<double>>("x_ee_desired", {0.000889178, 0.8629, 1.1294});
        this->declare_parameter<std::vector<double>>("q_ee_desired", {-0.707107, 0.000563088, 0.000563088, 0.707107});
        Kp_pos = this->declare_parameter<double>("Kp_pos", 10.0);  // Ganancia proporcional para posición
        Kp_orient = this->declare_parameter<double>("Kp_orient", 15.0);  // Ganancia proporcional para orientación
        Kd_pos = this->declare_parameter<double>("Kd_pos", 2.0 * std::sqrt(Kp_pos));
        Kd_orient = this->declare_parameter<double>("Kd_orient", 2.0 * std::sqrt(Kp_orient));
        
        // Kd_pos = this->declare_parameter<double>("Kd_pos", 0.1);  // Ganancia derivativa para posición
        // Kd_orient = this->declare_parameter<double>("Kd_orient", 0.1);  // Ganancia derivativa para orientación

        // Obtener parámetros
        urdf_path_ = this->get_parameter("urdf_path").as_string();
        damping_factor_ = this->get_parameter("damping_factor").as_double();
        velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
        kinematic_output_mode_ = this->get_parameter("kinematic_output_mode").as_int();
        orientation_error_switch_rad_ = this->get_parameter("orientation_error_switch_rad").as_double();
        use_trajectory_generator_ = this->get_parameter("use_trajectory_generator").as_bool();
        trajectory_wn_ = this->get_parameter("trajectory_wn").as_double();
        trajectory_c0_ = this->get_parameter("trajectory_c0").as_double();
        trajectory_mode_ = this->get_parameter("trajectory_mode").as_int();

        const auto trajectory_a_param = this->get_parameter("trajectory_A").as_double_array();
        if (trajectory_a_param.size() == 3) {
            trajectory_A_ << trajectory_a_param[0], trajectory_a_param[1], trajectory_a_param[2];
        } else {
            RCLCPP_WARN(this->get_logger(), "Parametro trajectory_A debe tener tamaño 3. Usando valores por defecto.");
            trajectory_A_ << 0.05, 0.05, 0.0;
        }

        if (orientation_error_switch_rad_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(),
                "orientation_error_switch_rad debe ser mayor que 0. Usando 0.25 rad.");
            orientation_error_switch_rad_ = 0.25;
        }

        if (trajectory_wn_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "trajectory_wn debe ser mayor que 0. Usando 0.6.");
            trajectory_wn_ = 0.6;
        }

        if (trajectory_c0_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "trajectory_c0 no puede ser negativa. Usando 0.2.");
            trajectory_c0_ = 0.2;
        }

        if (trajectory_mode_ < 1 || trajectory_mode_ > 3) {
            RCLCPP_WARN(this->get_logger(), "trajectory_mode debe ser 1, 2 o 3. Usando 1.");
            trajectory_mode_ = 1;
        }

        if (kinematic_output_mode_ != 1 && kinematic_output_mode_ != 2) {
            RCLCPP_WARN(this->get_logger(),
                "kinematic_output_mode debe ser 1 o 2. Usando 1 (posiciones integradas).");
            kinematic_output_mode_ = 1;
        }

        const auto x_ee_desired_param = this->get_parameter("x_ee_desired").as_double_array();
        if (x_ee_desired_param.size() == 3) {
            x_ee_desired_ << x_ee_desired_param[0], x_ee_desired_param[1], x_ee_desired_param[2];
        } else {
            RCLCPP_WARN(this->get_logger(), "Parametro x_ee_desired debe tener tamaño 3. Usando valores por defecto.");
            x_ee_desired_ << 0.000889178, 0.8629, 1.1294;
        }

        const auto q_ee_desired_param = this->get_parameter("q_ee_desired").as_double_array();
        if (q_ee_desired_param.size() == 4) {
            q_ee_desired_ << q_ee_desired_param[0], q_ee_desired_param[1], q_ee_desired_param[2], q_ee_desired_param[3];
        } else {
            RCLCPP_WARN(this->get_logger(), "Parametro q_ee_desired debe tener tamaño 4. Usando valores por defecto.");
            q_ee_desired_ << -0.707107, 0.000563088, 0.000563088, 0.707107;
        }

        Eigen::Quaterniond desired_quaternion(
            q_ee_desired_(3), q_ee_desired_(0), q_ee_desired_(1), q_ee_desired_(2));
        desired_quaternion.normalize();
        q_ee_desired_ << desired_quaternion.x(), desired_quaternion.y(), desired_quaternion.z(), desired_quaternion.w();
        R_ee_desired_ = desired_quaternion.toRotationMatrix();

        // Si no se proporciona ruta URDF, usar ruta por defecto
        if (urdf_path_.empty()) {
            urdf_path_ = "/home/david/tesis_ws/src/ur5_simulation/ur5_description/urdf/ur5e.urdf";
        }

        // Cargar modelo pinocchio
        RCLCPP_INFO(this->get_logger(), "Cargando modelo URDF desde: %s", urdf_path_.c_str());
        
        try {
            model_ = std::make_unique<pinocchio::Model>();
            pinocchio::urdf::buildModel(urdf_path_, *model_);
            data_ = std::make_unique<pinocchio::Data>(*model_);
            tool_frame_id_ = model_->getFrameId("tool0");
            
            RCLCPP_INFO(this->get_logger(), "Modelo cargado exitosamente");
            RCLCPP_INFO(this->get_logger(), "Número de joints: %ld", model_->nq);
            RCLCPP_INFO(this->get_logger(), "Número de DOF: %ld", model_->nv);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error cargando URDF: %s", e.what());
            throw;
        }

        // Definir orden esperado de articulaciones
        joint_order_ = {
            "r1_shoulder_pan_joint",
            "r1_shoulder_lift_joint",
            "r1_elbow_joint",
            "r1_wrist_1_joint",
            "r1_wrist_2_joint",
            "r1_wrist_3_joint"
        };

        // Inicializar variables
        q_ = VectorXd::Zero(model_->nq);
        q_command_ = VectorXd::Zero(model_->nq);
        v_ee_desired_ = Eigen::VectorXd::Zero(6);
        joint_velocities_ = VectorXd::Zero(model_->nv);
        trajectory_start_position_ = Eigen::Vector3d::Zero();

        // Posición y orientación deseadas se leen desde parámetros ROS.

        d_position_error = Eigen::Vector3d::Zero();
        d_orientation_error = Eigen::Vector3d::Zero();
        position_error_prev_ = Eigen::Vector3d::Zero();
        orientation_error_prev_ = Eigen::Vector3d::Zero();

        // Variables de estado
        joint_positions_map_.clear();
        joint_velocities_map_.clear();

        // Inicializar tiempo de simulación
        simulation_start_time_ = std::chrono::high_resolution_clock::now();

        // Suscriptor a joint_states
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/r1/joint_states",
            10,
            std::bind(&UR5eDifferentialIKNode::jointStatesCallback, this, std::placeholders::_1)
        );

        // Publicador de velocidades
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/r1/forward_velocity_controller/commands",
            rclcpp::SystemDefaultsQoS()
        );

        // Publicador de posiciones
        position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/r1/forward_position_controller/commands",
            rclcpp::SystemDefaultsQoS()
        );

        // Timer a 500 Hz (2 ms)
        timer_ = this->create_wall_timer(
            2ms,
            std::bind(&UR5eDifferentialIKNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Nodo UR5e Differential IK inicializado a 500 Hz");
    }

private:
    // Variables de Pinocchio
    std::unique_ptr<pinocchio::Model> model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex tool_frame_id_;
    
    // ROS2 publishers/subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Configuración
    std::vector<std::string> joint_order_;
    double damping_factor_;
    double velocity_scaling_;

    // Tiempo de simulación
    std::chrono::high_resolution_clock::time_point simulation_start_time_;
    
    // Estado actual
    VectorXd q_;  // Posiciones articulares
    VectorXd joint_velocities_;  // Velocidades calculadas
    VectorXd q_command_;  // Posición articular integrada a publicar
    VectorXd joint_velocities_prev_;  // Velocidades previas para cálculo de derivadas
    VectorXd joint_positions;
    Eigen::Matrix<double, 6, 1> v_ee_desired_;  // Velocidad deseada del efector final (6D)

    // Posicion deseada
    Eigen::Vector3d x_ee_desired_;  // Posición deseada del efector final (3D) [x,y,z]
    Eigen::Vector4d q_ee_desired_;  // Orientación deseada del efector final (quaternion)
    Eigen::Matrix3d R_ee_desired_;  // Orientación deseada del efector final (3D)


    Eigen::Vector3d d_position_error;
    Eigen::Vector3d d_orientation_error;
    Eigen::Vector3d position_error_prev_;
    Eigen::Vector3d orientation_error_prev_;

    // Kp
    double Kp_pos;
    double Kp_orient;

    double Kd_pos;  // Ganancia derivativa para posición
    double Kd_orient;  // Ganancia derivativa para orientación

    int kinematic_output_mode_ = 1;
    double orientation_error_switch_rad_ = 0.25;
    bool use_trajectory_generator_ = false;
    bool trajectory_initialized_ = false;
    Eigen::Vector3d trajectory_start_position_;
    std::chrono::high_resolution_clock::time_point trajectory_start_time_;
    Eigen::Vector3d trajectory_A_ = Eigen::Vector3d(0.05, 0.05, 0.0);
    double trajectory_wn_ = 0.6;
    double trajectory_c0_ = 0.2;
    int trajectory_mode_ = 1;
    bool q_command_initialized_ = false;

    double dt_ = 0.002;  // Intervalo de tiempo del timer (2 ms)

    
    std::map<std::string, double> joint_positions_map_;
    std::map<std::string, double> joint_velocities_map_;
    
    std::string urdf_path_;

    bool new_data_received_ = false;

    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Joint states callback received with %ld joints", msg->name.size());
        
        // Guardar posiciones y velocidades en mapas
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_positions_map_[msg->name[i]] = msg->position[i];
            //std::cout << joint_positions_map_[msg->name[i]] << std::endl;
            
            if (i < msg->velocity.size()) {
                joint_velocities_map_[msg->name[i]] = msg->velocity[i];
            }
        }
        new_data_received_ = true;
    }

    void timerCallback() {
        if (!new_data_received_) {
            return;
        }

        try {

            // tiempo transcurrido

            auto now = std::chrono::high_resolution_clock::now();
            double elapsed_time = std::chrono::duration<double>(now - simulation_start_time_).count();
 
            std::cout << "Tiempo transcurrido: " << elapsed_time << " segundos" << std::endl;
            // Reordenar posiciones según joint_order_
            for (size_t i = 0; i < joint_order_.size(); ++i) {
                const auto& joint_name = joint_order_[i];
                
                if (joint_positions_map_.find(joint_name) != joint_positions_map_.end()) {
                    q_[i] = joint_positions_map_[joint_name];
                }
            }
            std::cout << "Posiciones articulares actuales: [";
            for (int i = 0; i < q_.size(); ++i) {
                std::cout << q_[i];
                if (i < q_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;



            // Actualizar cinemática en Pinocchio
            model_ = std::make_unique<pinocchio::Model>();
            pinocchio::urdf::buildModel(urdf_path_, *model_);
            data_ = std::make_unique<pinocchio::Data>(*model_);
            tool_frame_id_ = model_->getFrameId("tool0");
            
            pinocchio::forwardKinematics(*model_, *data_, q_);
            pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);

            const pinocchio::SE3 current_pose = data_->oMf[tool_frame_id_];
            std::cout<<"Current end-effector position: ["<<current_pose.translation().transpose()<<"]"<<std::endl;
            std::cout<<"Current end-effector orientation (quaternion): ["<<Eigen::Quaterniond(current_pose.rotation()).coeffs().transpose()<<"]"<<std::endl;

            Eigen::Vector3d position_reference = x_ee_desired_;
            Eigen::Vector3d velocity_reference = Eigen::Vector3d::Zero();
            Eigen::Vector3d acceleration_reference = Eigen::Vector3d::Zero();

            if (use_trajectory_generator_) {
                if (!trajectory_initialized_) {
                    trajectory_start_position_ = current_pose.translation();
                    trajectory_start_time_ = now;
                    trajectory_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "Trayectoria inicializada desde la pose actual.");
                }

                const double trajectory_elapsed = std::chrono::duration<double>(now - trajectory_start_time_).count();
                const auto trajectory_state = TrajectoryGenerator::calculate(
                    trajectory_start_position_,
                    trajectory_A_,
                    trajectory_wn_,
                    trajectory_c0_,
                    trajectory_elapsed,
                    trajectory_mode_);

                position_reference = trajectory_state.position;
                velocity_reference = trajectory_state.velocity;
                acceleration_reference = trajectory_state.acceleration;
            }
            

            
            // Calcular Jacobiano del efector final
            // El índice del efector final es el último frame
            int ee_frame_id = model_->nframes - 1;
            MatrixXd jacobian = MatrixXd::Zero(6, model_->nv);
            
            pinocchio::computeFrameJacobian(*model_, *data_, q_, ee_frame_id, jacobian);

            // Ejemplo: Cinemática inversa diferencial con objetivo de movimiento
            // calculo de error

            Eigen::Vector3d position_error = position_reference - current_pose.translation();
            d_position_error = (position_error - position_error_prev_) / dt_;
            position_error_prev_ = position_error;

            // Error de orientación con conmutación por magnitud del ángulo.
            R_ee_desired_ = Eigen::Quaterniond(q_ee_desired_).toRotationMatrix();
            Eigen::Matrix3d R_err = R_ee_desired_ * current_pose.rotation().transpose();

            Eigen::Vector3d orientation_error;
            const Eigen::AngleAxisd angle_axis_error(R_err);
            const double orientation_error_angle = angle_axis_error.angle();

            if (orientation_error_angle > orientation_error_switch_rad_) {
                orientation_error = angle_axis_error.axis() * orientation_error_angle;
                std::cout << "Usando error de orientacion robusto (angle-axis)" << std::endl;
            } else {
                orientation_error << R_err(2,1) - R_err(1,2),
                                     R_err(0,2) - R_err(2,0),
                                     R_err(1,0) - R_err(0,1);
                orientation_error *= 0.5;
                std::cout << "Usando error de orientacion pequeño (aproximacion antisimetica)" << std::endl;
            }

            std::cout<<"Error de posición: ["<<position_error.transpose()<<"]"<<std::endl;
            std::cout<<"Referencia de posicion: ["<<position_reference.transpose()<<"]"<<std::endl;
            //std::cout<<"Error de orientación (matriz): \n"<<R_err<<std::endl;

            //std::cout << "Magnitud del error de orientación: " << orientation_error_angle << " rad" << std::endl;



            d_orientation_error = (orientation_error - orientation_error_prev_) / dt_;
            orientation_error_prev_ = orientation_error;
            std::cout<<"Error de orientación (vector): ["<<orientation_error.transpose()<<"]"<<std::endl;

            // double Kp_pos = 10.0;  // Ganancia proporcional para posición
            // double Kp_orient = 15.0;  // Ganancia proporcional para orientación
            v_ee_desired_.head(3) = Kp_pos * position_error; //+ velocity_reference + ;// + Kd_pos * d_position_error;
            v_ee_desired_.tail(3) = Kp_orient * orientation_error;// + Kd_orient * d_orientation_error;

            // Calcular pseudoinversa del Jacobiano con damping
            // J_pinv = J^T * (J*J^T + lambda^2*I)^-1
            MatrixXd JJt = jacobian * jacobian.transpose();
            MatrixXd I = MatrixXd::Identity(6, 6);
            MatrixXd dampingMatrix = JJt + (damping_factor_ * damping_factor_) * I;
            
            MatrixXd J_pinv = jacobian.transpose() * dampingMatrix.inverse();

            // Calcular velocidades articulares
            joint_velocities_ = J_pinv * v_ee_desired_;

            // Aplicar scaling
            joint_velocities_ *= velocity_scaling_;

            // Limitar velocidades máximas (rad/s)
            double max_velocity = 0.5;  // rad/s para UR5e
            for (int i = 0; i < joint_velocities_.size(); ++i) {
                if (joint_velocities_[i] > max_velocity) {
                    joint_velocities_[i] = max_velocity;
                } else if (joint_velocities_[i] < -max_velocity) {
                    joint_velocities_[i] = -max_velocity;
                }
            }
            std::cout<<"Velocidades articulares calculadas: [";
            for (int i = 0; i < joint_velocities_.size(); ++i) {
                std::cout << joint_velocities_[i];
                if (i < joint_velocities_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;

            

            if (kinematic_output_mode_ == 1) {
                if (!q_command_initialized_) {
                    q_command_ = q_;
                    q_command_initialized_ = true;
                }
                q_command_ += dt_ * joint_velocities_;

                // Publicar posiciones articulares integradas
                publishPositions();
            } else {
                // Publicar velocidades articulares directas
                publishVelocities();
            }

            // Log de debug (cada 50 iteraciones = 0.1 segundos a 500 Hz)
            static int counter = 0;
            if (++counter % 50 == 0) {
                RCLCPP_DEBUG(this->get_logger(), 
                    "q: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]
                );
                RCLCPP_DEBUG(this->get_logger(), 
                    "dq: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    joint_velocities_[0], joint_velocities_[1], joint_velocities_[2],
                    joint_velocities_[3], joint_velocities_[4], joint_velocities_[5]
                );
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error en timerCallback: %s", e.what());
        }
    }

    void publishPositions() {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(q_command_.size());
        
        for (int i = 0; i < q_command_.size(); ++i) {
            msg.data[i] = q_command_[i];
        }
        
        position_pub_->publish(msg);
    }

    void publishVelocities() {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(joint_velocities_.size());
        
        for (int i = 0; i < joint_velocities_.size(); ++i) {
            msg.data[i] = joint_velocities_[i];
        }
        
        velocity_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<UR5eDifferentialIKNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Excepción: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
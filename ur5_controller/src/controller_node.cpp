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
#include <omni_msgs/msg/omni_button_event.hpp>
#include <omni_msgs/msg/omni_state.hpp>
#include <omni_msgs/msg/omni_feedback.hpp>

// Eigen para matemáticas
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

// Usar el namespace para las structs
using namespace ur5_controller;


class TrajectoryGenerator {
public:
    // Estructura para contener los resultados
    struct State {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
    };

    // Calcula el estado completo de la trayectoria en un tiempo dado
    static State calculate(
        const Eigen::Vector3d& x_init,
        const Eigen::Vector3d& A,
        double wn,
        double c0,
        double time_elapsed,
        int grafica)
    {
        State state;
        double t = time_elapsed;

        // --- Términos comunes (se calculan una sola vez) ---
        double exp_neg_c0_t = exp(-c0 * t);
        double sin_wn_t = sin(wn * t);
        double cos_wn_t = cos(wn * t);
        
        double amp_factor = 1.0 - exp_neg_c0_t;
        double d_amp_factor_dt = c0 * exp_neg_c0_t;
        double d2_amp_factor_dt2 = -c0 * c0 * exp_neg_c0_t;
        // ----------------------------------------------------
        if (grafica ==1) {
            // --- Cálculo de Posición ---
            state.position.x() = x_init.x() + A.x() * amp_factor * sin_wn_t;
            state.position.y() = x_init.y() + A.y() * amp_factor * cos_wn_t;
            state.position.z() = x_init.z() + A.z() * amp_factor * sin_wn_t;

            // --- Cálculo de Velocidad ---
            state.velocity.x() = A.x() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);
            state.velocity.y() = A.y() * (d_amp_factor_dt * cos_wn_t - amp_factor * wn * sin_wn_t);
            state.velocity.z() = A.z() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);

            // --- Cálculo de Aceleración ---
            double term1_sin = d2_amp_factor_dt2 - amp_factor * wn * wn;
            double term2_cos = 2 * d_amp_factor_dt * wn;
            state.acceleration.x() = A.x() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
            state.acceleration.y() = A.y() * (term1_sin * cos_wn_t - term2_cos * sin_wn_t);
            state.acceleration.z() = A.z() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
        }
        else if (grafica == 2){
            state.position.x() = x_init.x() -0.3 + 0.3*exp_neg_c0_t;
            state.position.y() = x_init.y() + 0.1 - 0.1*exp_neg_c0_t;
            state.position.z() = x_init.z() + 0.1 - 0.1*exp_neg_c0_t;

            state.velocity.x() = -0.3 * (-c0) * exp_neg_c0_t;
            state.velocity.y() = 0.1 * (-c0) * exp_neg_c0_t;
            state.velocity.z() = 0.1 * (-c0) * exp_neg_c0_t;

            state.acceleration.x() = -0.3 * (c0 * c0) * exp_neg_c0_t;
            state.acceleration.y() = 0.1 * (c0 * c0) * exp_neg_c0_t;
            state.acceleration.z() = 0.1 * (c0 * c0) * exp_neg_c0_t;
        } 
        // grafica == 3 eliminado

        return state;
    }
};

Eigen::Vector3d trayectoria_geomagic(Eigen::Vector3d x_init, Eigen::Vector3d r_init,Eigen::Vector3d r_actual, double escala){
    Eigen::Vector3d x_des = Eigen::Vector3d::Zero();
    x_des[0] = x_init[0] + (r_actual[1]-r_init[1])*escala;
    x_des[1] = x_init[1] - (r_actual[0]-r_init[0])*escala;
    x_des[2] = x_init[2] + (r_actual[2]-r_init[2])*escala;
    return x_des;
};


void initializeUR5(PinocchioResources& pinocchio, const std::string& urdf_path) {
    pinocchio.model = std::make_unique<pinocchio::Model>();

    auto logger = rclcpp::get_logger("UR5Kinematics");
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





class UR5IKNode : public rclcpp::Node
{
public:
  UR5IKNode() : Node("ur5_ik_node")
  {
    // 1. Declarar parámetros
    this->declare_parameter<std::string>("control_topic", config_.control_topic);
    this->declare_parameter<bool>("geomagic", config_.use_geomagic);
    this->declare_parameter<std::string>("ur", config_.ur_model);
    this->declare_parameter<std::string>("nmspace", config_.nmspace);
    this->declare_parameter<std::string>("urdf_path", "");
    this->declare_parameter<std::string>("geomagic_topic", "/phantom/pose");
    this->declare_parameter<std::string>("geomagic_button_topic", "/phantom/button");
    // Movimiento inicial
    this->declare_parameter<bool>("use_ur5_pos_init", config_.use_ur5_pos_init);
    this->declare_parameter<std::vector<double>>("q_target", config_.q_target);
    this->declare_parameter<double>("q_target_time", config_.q_target_time);
    // CSV
    this->declare_parameter<bool>("csv_log_enable", config_.csv_enabled);
    this->declare_parameter<std::string>("csv_log_dir", config_.csv_path);
    this->declare_parameter<std::string>("csv_log_prefix", config_.csv_prefix);
    // Trayectoria automática
    this->declare_parameter<std::vector<double>>("traj_A", {config_.traj_A.x(), config_.traj_A.y(), config_.traj_A.z()});
    this->declare_parameter<double>("traj_wn", config_.traj_wn);
    this->declare_parameter<double>("traj_c0", config_.traj_c0);
    this->declare_parameter<int>("traj_mode", config_.traj_mode);
    this->declare_parameter<std::string>("controller_type", config_.controller);
    this->declare_parameter<std::vector<double>>("Kp", {1850.0, 1850.0, 1850.0, 500.0, 500.0, 500.0, 5000.0});
    this->declare_parameter<std::vector<double>>("Kd", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    this->declare_parameter<std::vector<double>>("lambda",{0.5,0.5,0.5,0.5,0.5,0.5});
    this->declare_parameter<std::vector<double>>("k",{50.0, 50.0, 50.0, 50.0, 50.0, 50.0});
    this->declare_parameter<std::vector<double>>("k2",{10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    config_.alpha = this->declare_parameter<double>("alpha", 0.01);
    config_.damping_factor = this->declare_parameter<double>("damping_factor", 0.01);
    config_.dt = this->declare_parameter<double>("dt", 0.01);

    config_.ctrl_hz_ = this->declare_parameter<double>("ctrl_hz", 500.0); // 125Hz para estabilidad
    max_joint_step_rad_ = this->declare_parameter<double>("max_joint_step_rad", 0.05); // Paso máximo por ciclo
    large_error_threshold_rad_ = this->declare_parameter<double>("large_error_threshold_rad", 0.15); // Umbral error grande
    map_pos_ = {static_cast<int>(this->declare_parameter<double>("map_x", 2.0)),
                static_cast<int>(this->declare_parameter<double>("map_y", 0.0)),
                static_cast<int>(this->declare_parameter<double>("map_z", 1.0))};
    sign_pos_ = {this->declare_parameter<double>("sign_x", -1.0),
                    this->declare_parameter<double>("sign_y", -1.0),
                    this->declare_parameter<double>("sign_z", 1.0)};

    map_rot_ = {static_cast<int>(this->declare_parameter<double>("map_roll", 2.0)),
                static_cast<int>(this->declare_parameter<double>("map_pitch", 0.0)),
                static_cast<int>(this->declare_parameter<double>("map_yaw", 1.0))};
    sign_rot_ = {this->declare_parameter<double>("sign_roll", 1.0),
                    this->declare_parameter<double>("sign_pitch", 1.0),
                    this->declare_parameter<double>("sign_yaw", 1.0)};



    // 2. Obtener parámetros y guardarlos en la struct de configuración
    this->get_parameter("control_topic", config_.control_topic);
    this->get_parameter("geomagic", config_.use_geomagic);
    this->get_parameter("ur", config_.ur_model);
    this->get_parameter("nmspace", config_.nmspace);
    this->get_parameter("use_ur5_pos_init", config_.use_ur5_pos_init);
    this->get_parameter("q_target", config_.q_target);
    this->get_parameter("q_target_time", config_.q_target_time);
    this->get_parameter("csv_log_enable", config_.csv_enabled);
    this->get_parameter("csv_log_dir", config_.csv_path);
    this->get_parameter("csv_log_prefix", config_.csv_prefix);
    this->get_parameter("controller_type", config_.controller);
    // Obtener parámetros de trayectoria
    std::vector<double> A_param;
    this->get_parameter("traj_A", A_param);
    if (A_param.size() == 3) {
        config_.traj_A = Eigen::Vector3d(A_param[0], A_param[1], A_param[2]);
    } else {
        RCLCPP_WARN(this->get_logger(), "Parametro traj_A debe tener tamaño 3. Usando valores por defecto.");
    }
    this->get_parameter("traj_wn", config_.traj_wn);
    this->get_parameter("traj_c0", config_.traj_c0);
    this->get_parameter("traj_mode", config_.traj_mode);
    if (config_.traj_mode < 1 || config_.traj_mode > 3) {
        RCLCPP_WARN(this->get_logger(), "traj_mode fuera de rango (%d). Se usará 1.", config_.traj_mode);
        config_.traj_mode = 1;
    }
    
    // Obtener parámetros de control
    std::vector<double> Kp_param;
    this->get_parameter("Kp", Kp_param);
    if (Kp_param.size() == 7) {
        for (int i = 0; i < 7; ++i) {
            config_.Kp(i) = Kp_param[i];
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Parametro Kp debe tener tamaño 7. Usando valores por defecto.");
        config_.Kp << 1850.0, 1850.0, 1850.0, 500.0, 500.0, 500.0, 500.0;
    }
    
    std::vector<double> Kd_param;
    this->get_parameter("Kd", Kd_param);
    if (Kd_param.size() == 7) {
        for (int i = 0; i < 7; ++i) {
            config_.Kd(i) = Kd_param[i];
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Parametro Kd debe tener tamaño 7. Usando valores por defecto.");
        config_.Kd << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    }
    std::vector<double> param_lambda;
    this->get_parameter("lambda", param_lambda);
    if (param_lambda.size() == 6) {
        for (int i = 0; i < 6; ++i) {
            config_.lambda(i) = param_lambda[i];
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Parametro lambda debe tener tamaño 6. Usando valores por defecto.");
        config_.lambda << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    }
    std::vector<double> param_k;
    this->get_parameter("k", param_k);
    if (param_k.size() == 6) {
        for (int i = 0; i < 6; ++i) {
            config_.k(i) = param_k[i];
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Parametro k debe tener tamaño 6. Usando valores por defecto.");
        config_.k << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
    }
    std::vector<double> param_k2;
    this->get_parameter("k2", param_k2);
    if (param_k2.size() == 6) {
        for (int i = 0; i < 6; ++i) {
            config_.k2(i) = param_k2[i];
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Parametro k2 debe tener tamaño 6. Usando valores por defecto.");
        config_.k2 << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    }
    
    this->get_parameter("alpha", config_.alpha);
    
    
    std::string urdf_param;
    std::string geomagic_topic;
    std::string geomagic_button_topic;

    this->get_parameter("geomagic_button_topic", geomagic_button_topic);
    this->get_parameter("geomagic_topic", geomagic_topic);
    this->get_parameter("urdf_path", urdf_param);
    if (urdf_param.empty()) {
        config_.urdf_path = get_file_path("ur5_description", "urdf/" + config_.ur_model + ".urdf");
    } else {
        config_.urdf_path = urdf_param;
    }

    // Inicializar Pinocchio
    initializeUR5(pinocchio_, config_.urdf_path);
    kinematics_solver_ = std::make_unique<UR5Kinematics>(config_.urdf_path);
    impedance_controller_ = std::make_unique<ur5_impedance::UR5Impedance>(config_.urdf_path);
    sliding_controller_ = std::make_unique<ur5_sliding::UR5Sliding>(config_.urdf_path);
    std::string c_topic = "/" + config_.nmspace + config_.control_topic;
    std::string joint_states_topic = config_.nmspace.empty() ? std::string("/joint_states")
                                                            : std::string("/") + config_.nmspace + std::string("/joint_states");

    RCLCPP_INFO(this->get_logger(), "Publicando en el tópico de control: '%s'", c_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Usando modelo UR: '%s'", config_.ur_model.c_str());
    RCLCPP_INFO(this->get_logger(), "Usando URDF en: '%s'", config_.urdf_path.c_str());
    joint_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(c_topic, 10);
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_topic, 10, std::bind(&UR5IKNode::update_joint_positions, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Suscrito a joint_states: '%s'", joint_states_topic.c_str());
    // Suscripción de respaldo al tópico global en caso de que el driver no use namespace
    if (!config_.nmspace.empty() && joint_states_topic != "/joint_states") {
        joint_states_sub_global_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&UR5IKNode::update_joint_positions, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Suscripción de respaldo a joint_states: '/joint_states'");
    }

    if (config_.use_geomagic) {
        geomagic_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(geomagic_topic, 10, std::bind(&UR5IKNode::pose_callback, this, std::placeholders::_1));
        subscription_phantom_button_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(geomagic_button_topic, 10, std::bind(&UR5IKNode::button_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Usando tópico de Geomagic: '%s'", geomagic_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Usando tópico de botón Geomagic: '%s'", geomagic_button_topic.c_str());
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Modo Geomagic deshabilitado. Seguimiento de la trayectoria predefinida.");
        RCLCPP_INFO(this->get_logger(), "Controlador seleccionado: '%s'", config_.controller.c_str());
        // Programar movimiento inicial tipo ur5_pos si está habilitado
        if (config_.use_ur5_pos_init) {
            init_move_active_ = true;            // activar modo publicación hasta detectar movimiento
            init_baseline_set_ = false;          // baseline de joints se tomará al recibir JointState
            step_publishing_initialized_ = false; // resetear estado de pasos para nueva secuencia
            step_count_ = 0;
            step_total_ = 0;
            init_move_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / config_.ctrl_hz_)),  // 10 Hz (cada 100ms)
                std::bind(&UR5IKNode::init_move_tick, this)
            );
            RCLCPP_INFO(this->get_logger(), "Modo ur5_pos activo: publicando objetivo hasta detectar movimiento (umbral %.3f rad)", move_detect_threshold_);
        }
        // Trayectoria automática se activará una vez termine movimiento inicial o inmediatamente si no se usa init
        if (!config_.use_ur5_pos_init) {
            trajectory_active_ = false; // se activará tras captura de pose
        }
    }

    // Inicializar CSV si está habilitado
    if (config_.csv_enabled) {
        init_csv_logger();
    }

    // Callback para parámetros dinámicos (aplica cambios en caliente)
    params_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&UR5IKNode::on_parameters_set, this, std::placeholders::_1));

    last_successful_publish_ = std::chrono::steady_clock::now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / config_.ctrl_hz_)), std::bind(&UR5IKNode::control_loop, this));
  }

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
    std::mutex dynamic_params_mutex_;

    rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";

        // Copias locales para aplicar todo de forma consistente
        Eigen::Vector3i new_map_pos = map_pos_;
        Eigen::Vector3d new_sign_pos = sign_pos_;
        Eigen::Vector3i new_map_rot = map_rot_;
        Eigen::Vector3d new_sign_rot = sign_rot_;

        Eigen::Vector3d new_traj_A = config_.traj_A;
        double new_traj_wn = config_.traj_wn;
        double new_traj_c0 = config_.traj_c0;
        int new_traj_mode = config_.traj_mode;
        std::string new_controller = config_.controller;

        auto reject = [&](const std::string& why) {
            result.successful = false;
            result.reason = why;
        };

        auto validate_map_idx = [&](int v, const std::string& name) {
            if (v < 0 || v > 2) {
                reject(name + " debe estar en [0,2]");
                return false;
            }
            return true;
        };
        auto validate_sign = [&](double v, const std::string& name) {
            if (!(v == 1.0 || v == -1.0)) {
                reject(name + " debe ser 1.0 o -1.0");
                return false;
            }
            return true;
        };

        for (const auto& p : params) {
            const auto& name = p.get_name();

            // --- Mapeo de posición ---
            if (name == "map_x") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) { reject("map_x debe ser int"); break; }
                int v = static_cast<int>(p.as_int());
                if (!validate_map_idx(v, name)) break;
                new_map_pos.x() = v;
            } else if (name == "map_y") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) { reject("map_y debe ser int"); break; }
                int v = static_cast<int>(p.as_int());
                if (!validate_map_idx(v, name)) break;
                new_map_pos.y() = v;
            } else if (name == "map_z") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) { reject("map_z debe ser int"); break; }
                int v = static_cast<int>(p.as_int());
                if (!validate_map_idx(v, name)) break;
                new_map_pos.z() = v;
            } else if (name == "sign_x") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_x debe ser double"); break; }
                double v = p.as_double();
                if (!validate_sign(v, name)) break;
                new_sign_pos.x() = v;
            } else if (name == "sign_y") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_y debe ser double"); break; }
                double v = p.as_double();
                if (!validate_sign(v, name)) break;
                new_sign_pos.y() = v;
            } else if (name == "sign_z") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_z debe ser double"); break; }
                double v = p.as_double();
                if (!validate_sign(v, name)) break;
                new_sign_pos.z() = v;

            // --- Mapeo de rotación ---
            } else if (name == "map_roll") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) { reject("map_roll debe ser int"); break; }
                int v = static_cast<int>(p.as_int());
                if (!validate_map_idx(v, name)) break;
                new_map_rot.x() = v;
            } else if (name == "map_pitch") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) { reject("map_pitch debe ser int"); break; }
                int v = static_cast<int>(p.as_int());
                if (!validate_map_idx(v, name)) break;
                new_map_rot.y() = v;
            } else if (name == "map_yaw") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) { reject("map_yaw debe ser int"); break; }
                int v = static_cast<int>(p.as_int());
                if (!validate_map_idx(v, name)) break;
                new_map_rot.z() = v;
            } else if (name == "sign_roll") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_roll debe ser double"); break; }
                double v = p.as_double();
                if (!validate_sign(v, name)) break;
                new_sign_rot.x() = v;
            } else if (name == "sign_pitch") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_pitch debe ser double"); break; }
                double v = p.as_double();
                if (!validate_sign(v, name)) break;
                new_sign_rot.y() = v;
            } else if (name == "sign_yaw") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("sign_yaw debe ser double"); break; }
                double v = p.as_double();
                if (!validate_sign(v, name)) break;
                new_sign_rot.z() = v;

            // --- Trayectoria automática ---
            } else if (name == "traj_A") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) { reject("traj_A debe ser double[]"); break; }
                auto v = p.as_double_array();
                if (v.size() != 3) { reject("traj_A debe tener tamaño 3"); break; }
                new_traj_A = Eigen::Vector3d(v[0], v[1], v[2]);
            } else if (name == "traj_wn") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("traj_wn debe ser double"); break; }
                new_traj_wn = p.as_double();
            } else if (name == "traj_c0") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) { reject("traj_c0 debe ser double"); break; }
                new_traj_c0 = p.as_double();
            } else if (name == "traj_mode") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) { reject("traj_mode debe ser int"); break; }
                int v = static_cast<int>(p.as_int());
                // En este archivo sólo existen modos 1 y 2 (el 3 está comentado/eliminado)
                if (v != 1 && v != 2) { reject("traj_mode debe ser 1 o 2"); break; }
                new_traj_mode = v;

            // --- Controlador ---
            } else if (name == "controller_type") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) { reject("controller_type debe ser string"); break; }
                auto v = p.as_string();
                if (!(v == "QP" || v == "IMP" || v == "SLD")) {
                    reject("controller_type debe ser 'QP', 'IMP' o 'SLD'");
                    break;
                }
                new_controller = v;
            }
        }

        if (!result.successful) {
            return result;
        }

        {
            std::scoped_lock<std::mutex> lock(dynamic_params_mutex_);
            map_pos_ = new_map_pos;
            sign_pos_ = new_sign_pos;
            map_rot_ = new_map_rot;
            sign_rot_ = new_sign_rot;
            config_.traj_A = new_traj_A;
            config_.traj_wn = new_traj_wn;
            config_.traj_c0 = new_traj_c0;
            config_.traj_mode = new_traj_mode;
            config_.controller = new_controller;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Parametros actualizados: map_pos=[%d,%d,%d] sign_pos=[%.0f,%.0f,%.0f] map_rot=[%d,%d,%d] sign_rot=[%.0f,%.0f,%.0f] traj_mode=%d controller=%s",
                    map_pos_.x(), map_pos_.y(), map_pos_.z(),
                    sign_pos_.x(), sign_pos_.y(), sign_pos_.z(),
                    map_rot_.x(), map_rot_.y(), map_rot_.z(),
                    sign_rot_.x(), sign_rot_.y(), sign_rot_.z(),
                    config_.traj_mode,
                    config_.controller.c_str());

        return result;
    }

    // Suscriptores y publicadores
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_global_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr geomagic_pose_sub_;
    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr subscription_phantom_button_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;  

    // --- Variables agrupadas en structs ---
    RobotState robot_state_;
    CartesianState cartesian_state_;
    HapticState haptic_state_;
    NodeConfig config_;
    PinocchioResources pinocchio_;
    // ------------------------------------

    // Filtro simple para comandos articulares
    Eigen::VectorXd q_cmd_filtered_ = Eigen::VectorXd::Zero(6);
    bool q_cmd_initialized_ = false;
    double q_cmd_alpha_ = 0.2; // 0-1, mas alto = menos filtrado
    
    // Interpolación adaptativa para comandos alejados
    Eigen::VectorXd q_target_interpolated_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_solution_prev_ = Eigen::VectorXd::Zero(6);  // Solución IK anterior
    double max_joint_step_rad_ = 0.05; // Paso máximo por ciclo (ajustable según ctrl_hz_)
    double large_error_threshold_rad_ = 0.15; // Umbral para detectar error grande
    double max_ik_change_threshold_rad_ = 0.10; // Umbral para detectar cambio brusco en solución IK

    // Banderas de estado
    bool pose_inicial_capturada_ = false;
    bool posicion_inicial_alcanzada_ = false;
    bool capturar_pose_inicial_haptico_ = false;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<UR5Kinematics> kinematics_solver_;
    std::unique_ptr<ur5_impedance::UR5Impedance> impedance_controller_;
    std::unique_ptr<ur5_sliding::UR5Sliding> sliding_controller_;
   

    // ---- Mapeo robusto de joints por nombre ----
    std::vector<int> joint_index_map_{};          // índice en msg->name para cada joint esperado
    bool joint_map_initialized_ = false;
    std::vector<std::string> last_js_names_{};    // para detectar cambios y reconstruir el mapa

    // ---- CSV logging ----
    //bool csv_enabled = false;
    //std::string csv_path;
    //std::string config_.csv_prefix = "ur5_log";
    std::string csv_file_path_;
    std::ofstream csv_file_;
    rclcpp::Time start_time_{};
    rclcpp::Time last_log_time_{};
    // Métricas de tiempo por ciclo
    double last_ik_ms_ {0.0};
    double last_loop_ms_ {0.0};
    
    // === Watchdog para detectar delays y desconexiones ===
    std::chrono::steady_clock::time_point last_successful_publish_;
    int consecutive_loop_delays_ = 0;
    int consecutive_publish_failures_ = 0;
    static constexpr int MAX_CONSECUTIVE_DELAYS = 3;  // Advertencia después de 3 ciclos lentos
    static constexpr int MAX_CONSECUTIVE_FAILURES = 10; // Error crítico después de 10 fallos
    bool watchdog_enabled_ = true;
    double loop_timeout_ms_ = 10.0; // Alerta si ciclo tarda más de 10ms (debería ser 2ms a 500Hz)

    Eigen::Vector3d sign_pos_, sign_rot_;
    Eigen::Vector3i map_pos_, map_rot_;


    // ---- Movimiento inicial tipo ur5_pos ----
    // (Variables trasladadas a config_)
    bool init_pose_published_ = false; // legado (no usado en modo repetitivo)
    bool init_move_active_ = false;    // publicar hasta detectar movimiento
    bool init_baseline_set_ = false;   // baseline ya capturada
    std::vector<double> q_baseline_{0.0,0.0,0.0,0.0,0.0,0.0};
    double move_detect_threshold_ = 0.010; // rad, ~0.57 deg
    rclcpp::TimerBase::SharedPtr init_move_timer_;
    rclcpp::TimerBase::SharedPtr trajectory_delay_timer_;
    // Detección de llegada a objetivo articular inicial
    bool init_movement_started_ = false;     // ya comenzó a moverse hacia el objetivo
    double reach_threshold_rad_ = 0.020;     // rad, condición de cercanía a q_target
    int reach_count_ = 0;                    // conteo de comprobaciones consecutivas en umbral
    int reach_count_required_ = 5;           // número de comprobaciones consecutivas para confirmar llegada
    // ---- Control de pasos para movimiento inicial ----
    bool step_publishing_initialized_ = false;  // saber si ya inicializamos el proceso de pasos
    std::vector<double> step_error_{0.0,0.0,0.0,0.0,0.0,0.0};  // error actual por joint
    std::vector<double> step_q_{0.0,0.0,0.0,0.0,0.0,0.0};      // posición siendo pasada
    int step_count_ = 0;                        // número de pasos ya publicados
    int step_total_ = 0;                        // número total de pasos necesarios
    // ---- Trayectoria automática sin geomagic ----
    // (Parámetros de trayectoria ahora en config_)
    bool trajectory_active_ {false};
    rclcpp::Time trajectory_start_time_ {};
    double t_traj_ = 0.0; // Tiempo para la trayectoria, independiente de dt

    static std::string get_home_dir() {
        const char* home = std::getenv("HOME");
        return home ? std::string(home) : std::string(".");
    }

    std::string now_timestamp_string() {
        auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        localtime_r(&tt, &tm);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    void init_csv_logger() {
        // Directorio por defecto: ~/.ros/ur5_logs
        if (config_.csv_path.empty()) {
            config_.csv_path = get_home_dir() + std::string("/.ros/ur5_logs");
        }
        std::error_code ec;
        std::filesystem::create_directories(config_.csv_path, ec);
        if (ec) {
            RCLCPP_WARN(this->get_logger(), "No se pudo crear el directorio de CSV '%s': %s", config_.csv_path.c_str(), ec.message().c_str());
        }

        // Nombre de archivo único: <prefix>_<ns>_<YYYYMMDD_HHMMSS>.csv
        std::string ns_part = config_.nmspace.empty() ? std::string("nonamespace") : config_.nmspace;
        std::string fname = config_.csv_prefix + std::string("_") + ns_part + std::string("_") + now_timestamp_string() + std::string(".csv");
        csv_file_path_ = (std::filesystem::path(config_.csv_path) / fname).string();

        csv_file_.open(csv_file_path_, std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir archivo CSV: %s", csv_file_path_.c_str());
            config_.csv_enabled = false;
            return;
        }

        // Tiempos base
        start_time_ = this->now();
        last_log_time_ = start_time_;

    // Encabezado
    csv_file_ << "t,dt"
        << ",q_meas_0,q_meas_1,q_meas_2,q_meas_3,q_meas_4,q_meas_5"
        << ",q_cmd_0,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5"
        << ",e_q_0,e_q_1,e_q_2,e_q_3,e_q_4,e_q_5"
        << ",x_des_x,x_des_y,x_des_z"
        << ",x_meas_x,x_meas_y,x_meas_z"
        << ",q_des_w,q_des_x,q_des_y,q_des_z"
        << ",q_meas_w,q_meas_x,q_meas_y,q_meas_z"
        << ",euler_des_r,euler_des_p,euler_des_y"
        << ",euler_meas_r,euler_meas_p,euler_meas_y"
        << ",e_R_angle"
        << ",pos_err_x,pos_err_y,pos_err_z"
        << ",ori_err_axis_x,ori_err_axis_y,ori_err_axis_z,ori_err_angle"
        << ",u_control_0,u_control_1,u_control_2,u_control_3,u_control_4,u_control_5"
        << ",ik_ms,loop_ms"
        << std::endl;

        RCLCPP_INFO(this->get_logger(), "CSV logging habilitado: %s", csv_file_path_.c_str());
    }

    void write_csv_row(double t, double dt,
                       const Eigen::Vector3d& x_des,
                       const Eigen::Vector3d& x_meas,
                       const Eigen::Quaterniond& q_des,
                       const Eigen::Quaterniond& q_meas,
                       const Eigen::Vector3d& euler_des,
                       const Eigen::Vector3d& euler_meas,
                       double e_R_angle,
                       const Eigen::Vector3d& pos_err,
                       const Eigen::Vector3d& ori_err_axis,
                       double ori_err_angle,
                       const Eigen::VectorXd& u_control,
                       double ik_ms,
                       double loop_ms) {
        if (!config_.csv_enabled || !csv_file_.is_open()) return;

        csv_file_ << std::fixed << std::setprecision(6)
                  << t << "," << dt;

        // q_meas
        for (int i = 0; i < 6; ++i) csv_file_ << "," << robot_state_.q[i];
        // q_cmd
        for (int i = 0; i < 6; ++i) csv_file_ << "," << robot_state_.q_solution[i];
        // e_q = q_cmd - q_meas
        for (int i = 0; i < 6; ++i) csv_file_ << "," << (robot_state_.q_solution[i] - robot_state_.q[i]);

        // x_des, x_meas
        csv_file_ << "," << x_des.x() << "," << x_des.y() << "," << x_des.z();
        csv_file_ << "," << x_meas.x() << "," << x_meas.y() << "," << x_meas.z();
        // quaternions deseado/medido
        csv_file_ << "," << q_des.w() << "," << q_des.x() << "," << q_des.y() << "," << q_des.z();
        csv_file_ << "," << q_meas.w() << "," << q_meas.x() << "," << q_meas.y() << "," << q_meas.z();
        // euler deseado/medido (roll, pitch, yaw)
        csv_file_ << "," << euler_des.x() << "," << euler_des.y() << "," << euler_des.z();
        csv_file_ << "," << euler_meas.x() << "," << euler_meas.y() << "," << euler_meas.z();
        // e_R_angle
        csv_file_ << "," << e_R_angle;
        // Error cartesiano de posición (desired - measured)
        csv_file_ << "," << pos_err.x() << "," << pos_err.y() << "," << pos_err.z();
        // Error de orientación (eje normalizado y ángulo)
        csv_file_ << "," << ori_err_axis.x() << "," << ori_err_axis.y() << "," << ori_err_axis.z() << "," << ori_err_angle;
        // Esfuerzos de control
        for (int i = 0; i < 6; ++i) csv_file_ << "," << u_control[i];

        // métricas de tiempos
        csv_file_ << "," << ik_ms << "," << loop_ms;

        csv_file_ << std::endl;
    }

    // Publica el objetivo articular a forward_position_controller (UN PASO cada 100ms)
    void publish_initial_joint_position() {
        if (config_.q_target.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "q_target debe tener 6 elementos, tiene %zu", config_.q_target.size());
            return;
        }
        
        // === INICIALIZACIÓN (primera llamada) ===
        if (!step_publishing_initialized_) {
            // Usar posición actual del robot como punto de inicio
            std::vector<double> robot_q(robot_state_.q.data(), robot_state_.q.data() + 6);
            step_q_ = robot_q;
            std::cout << "Posición actual del robot (baseline): [";
            for (size_t i = 0; i < 6; ++i) {
                std::cout << robot_q[i];
                if (i < 5) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            
            step_count_ = 0;
            
            // Calcular error elemento por elemento desde la posición actual (baseline)
            double max_error = 0.0;
            for (size_t i = 0; i < 6; ++i) {
                step_error_[i] = config_.q_target[i] - step_q_[i];
                max_error = std::max(max_error, std::abs(step_error_[i]));
            }
            
            // Redondear y calcular número total de pasos necesarios
            max_error = std::round(max_error * 100000.0) / 100000.0;
            step_total_ = static_cast<int>(std::ceil(max_error / 0.00001));
            
            RCLCPP_INFO(this->get_logger(), "Publicación de pasos INICIADA: %d pasos totales (máx error: %.3f rad)", 
                        step_total_, max_error);
            
            step_publishing_initialized_ = true;
        }
        
        // === PUBLICACIÓN DE UN SOLO PASO ===
        if (step_count_ < step_total_) {
            // Actualizar posición para este paso
            for (size_t j = 0; j < 6; ++j) {
                if (std::abs(step_error_[j]) > 0.001) {
                    // Avanzar 0.001 rad en la dirección del error
                    step_q_[j] = step_q_[j] + 0.001 * std::copysign(1.0, step_error_[j]);
                    step_error_[j] -= 0.001 * std::copysign(1.0, step_error_[j]);
                }
                else if (std::abs(step_error_[j]) > 1e-6) {
                    // Último paso: fijar exactamente al objetivo
                    step_q_[j] = config_.q_target[j];
                    step_error_[j] = 0.0;
                }
            }
            
            // Publicar este paso
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = step_q_;
            step_count_++;
            
            RCLCPP_INFO(this->get_logger(), "Publicando objetivo articular [paso %d/%d]: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                        step_count_, step_total_,
                        step_q_[0], step_q_[1], step_q_[2], step_q_[3], step_q_[4], step_q_[5]);
            joint_position_pub_->publish(msg);
        }
    }

    // Tick periódico: publica mientras no detecte movimiento respecto a baseline
    void init_move_tick() {
        if (!init_move_active_) return;
        // Asegurar baseline al recibir primeras articulaciones
        if (!init_baseline_set_) {
            // Necesitamos haber recibido joint_states y tener un mapeo válido
            if (last_joint_state_ && joint_map_initialized_) {
                for (int i = 0; i < 6; ++i) q_baseline_[i] = robot_state_.q[i];
                init_baseline_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Baseline articular capturada para detección de movimiento: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                           q_baseline_[0], q_baseline_[1], q_baseline_[2], q_baseline_[3], q_baseline_[4], q_baseline_[5]);
            }
            return; // NO publicar hasta tener baseline
        }

        // Detectar inicio de movimiento respecto a baseline
        if (!init_movement_started_ && init_baseline_set_) {
            for (int i = 0; i < 6; ++i) {
                if (std::abs(robot_state_.q[i] - q_baseline_[i]) > move_detect_threshold_) {
                    init_movement_started_ = true;
                    RCLCPP_INFO(this->get_logger(), "Movimiento inicial detectado (> %.3f rad). Continuando hasta alcanzar q_target.", move_detect_threshold_);
                    break;
                }
            }
        }

        // Verificar llegada a q_target con histéresis (consecutivo)
        double max_err = 0.0;
        for (int i = 0; i < 6; ++i) {
            max_err = std::max(max_err, std::abs(robot_state_.q[i] - config_.q_target[i]));
        }
        if (max_err < reach_threshold_rad_) {
            reach_count_++;
        } else {
            reach_count_ = 0;
        }

        if (reach_count_ >= reach_count_required_) {
            init_move_active_ = false;
            step_publishing_initialized_ = false;  // resetear estado de pasos para próxima secuencia
            if (init_move_timer_) init_move_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Objetivo articular alcanzado (|e|_max < %.3f rad por %d ciclos).", reach_threshold_rad_, reach_count_required_);
            // Activar trayectoria automática desde t = 0
            if (!config_.use_geomagic) {
                // Reasignar x_init y R_init a la pose actual alcanzada para arrancar la trayectoria exactamente desde q_target
                pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
                pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
                const auto& frame_now = pinocchio_.data->oMf[pinocchio_.tool_frame_id];

                cartesian_state_.position_initial = frame_now.translation();
                cartesian_state_.rotation_matrix_initial = frame_now.rotation();
                cartesian_state_.orientation_initial = Eigen::Quaterniond(frame_now.rotation());
                
                RCLCPP_INFO(this->get_logger(), "x_init y R_init reconfigurados a la pose actual antes de iniciar trayectoria.");
                RCLCPP_INFO(this->get_logger(), "Esperando 1 segundo antes de activar trayectoria automática...");
                
                // Crear timer de una sola ejecución (one-shot) de 1 segundo
                trajectory_delay_timer_ = this->create_wall_timer(
                    std::chrono::seconds(1),
                    [this]() {
                        trajectory_start_time_ = this->now();
                        trajectory_active_ = true;
                        RCLCPP_INFO(this->get_logger(), "Trayectoria automática ACTIVADA desde t=0 tras delay de 1 segundo.");
                        if (trajectory_delay_timer_) trajectory_delay_timer_->cancel();
                    }
                );
            }
            return;
        }

        // Publicar el objetivo mientras no se alcance
        publish_initial_joint_position();
    }

    static bool ends_with(const std::string& str, const std::string& suffix) {
        if (suffix.size() > str.size()) return false;
        return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
    }

    std::vector<std::string> get_expected_joint_names() const {
        std::string prefix = config_.nmspace.empty() ? std::string("") : (config_.nmspace + std::string("_"));
        return {
            prefix + "shoulder_pan_joint",
            prefix + "shoulder_lift_joint",
            prefix + "elbow_joint",
            prefix + "wrist_1_joint",
            prefix + "wrist_2_joint",
            prefix + "wrist_3_joint"
        };
    }

    std::vector<std::string> get_expected_base_joint_names() const {
        return {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };
    }

    bool same_name_list(const std::vector<std::string>& a, const std::vector<std::string>& b) {
        if (a.size() != b.size()) return false;
        for (size_t i = 0; i < a.size(); ++i) if (a[i] != b[i]) return false;
        return true;
    }

    void rebuild_joint_index_map(const sensor_msgs::msg::JointState::SharedPtr& msg) {
        joint_map_initialized_ = false;
        joint_index_map_.assign(6, -1);
        last_js_names_ = msg->name;

        const auto expected = get_expected_joint_names();
        const auto expected_base = get_expected_base_joint_names();

        // Mapa rápido de nombre -> índice del mensaje
        std::unordered_map<std::string, int> name_to_idx;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            name_to_idx[msg->name[i]] = static_cast<int>(i);
        }

        // 1) Intento por coincidencia exacta
        int found_exact = 0;
        for (int i = 0; i < 6; ++i) {
            auto it = name_to_idx.find(expected[i]);
            if (it != name_to_idx.end()) {
                joint_index_map_[i] = it->second;
                ++found_exact;
            }
        }

        // 2) Fallback: buscar por nombre base exacto si faltan
        if (found_exact < 6) {
            for (int i = 0; i < 6; ++i) {
                if (joint_index_map_[i] != -1) continue;
                auto itb = name_to_idx.find(expected_base[i]);
                if (itb != name_to_idx.end()) {
                    joint_index_map_[i] = itb->second;
                    ++found_exact;
                }
            }
        }

        // 3) Último recurso: buscar entradas que terminen con el nombre base
        if (found_exact < 6) {
            for (int i = 0; i < 6; ++i) {
                if (joint_index_map_[i] != -1) continue;
                for (size_t j = 0; j < msg->name.size(); ++j) {
                    if (ends_with(msg->name[j], expected_base[i])) {
                        joint_index_map_[i] = static_cast<int>(j);
                        ++found_exact;
                        break;
                    }
                }
            }
        }

        if (found_exact == 6) {
            joint_map_initialized_ = true;
            std::ostringstream map_info;
            map_info << "Mapa de joints establecido (idx en /joint_states): ";
            for (int i = 0; i < 6; ++i) map_info << joint_index_map_[i] << (i < 5 ? "," : "");
            RCLCPP_INFO(this->get_logger(), "%s", map_info.str().c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No se pudo establecer el mapeo de joints por nombre. Recibidos %zu nombres.", msg->name.size());
        }
    }

    // Callback de JOINT STATES del UR5(e)
    void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_joint_state_ = msg;
        // (Re)construir mapeo si es la primera vez o si la lista de nombres cambió
        if (!joint_map_initialized_ || !same_name_list(msg->name, last_js_names_)) {
            rebuild_joint_index_map(msg);
        }

        if (!joint_map_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Esperando mapeo válido de joints para reordenar JointState");
            return;
        }

        // Reordenar posiciones y velocidades según el mapeo
        for (int i = 0; i < 6; ++i) {
            int j = joint_index_map_[i];
            if (j < 0 || static_cast<size_t>(j) >= msg->position.size()) {
                RCLCPP_WARN(this->get_logger(), "Índice de joint fuera de rango para posiciones: %d", j);
                return;
            }
            robot_state_.q[i] = msg->position[j];
            if (static_cast<size_t>(j) < msg->velocity.size()) {
                robot_state_.qd[i] = msg->velocity[j];
            } else {
                robot_state_.qd[i] = 0.0;
            }
        }

        // Actualizar medición cartesiana actual (posición y orientación) del efector
        pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
        pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
        const auto& frame_placement_now = pinocchio_.data->oMf[pinocchio_.tool_frame_id];

        cartesian_state_.position = frame_placement_now.translation();
        cartesian_state_.rotation_matrix = frame_placement_now.rotation();
        cartesian_state_.orientation = Eigen::Quaterniond(frame_placement_now.rotation());

        // Si estamos en modo ur5_pos y aún no hay baseline, capturarla cuando llegan los primeros joints
        if (config_.use_ur5_pos_init && !config_.use_geomagic && init_move_active_ && !init_baseline_set_) {
            for (int i = 0; i < 6; ++i) q_baseline_[i] = robot_state_.q[i];
            init_baseline_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Baseline articular capturada (update_joint_positions).");
        }

        // Capturar la pose inicial automáticamente en la primera recepción
        if (!pose_inicial_capturada_) {
            RCLCPP_INFO(this->get_logger(), "Capturando la pose inicial actual del robot...");
            
            robot_state_.q_init = robot_state_.q;
            // Ya tenemos medición cartesiana actual en cartesian_state_ (actualizada más arriba
            // mediante forwardKinematics con q), por lo que podemos reutilizarla para inicial.
            cartesian_state_.position_initial = cartesian_state_.position;
            cartesian_state_.rotation_matrix_initial = cartesian_state_.rotation_matrix;
            cartesian_state_.orientation_initial = cartesian_state_.orientation;

            pose_inicial_capturada_ = true;
            posicion_inicial_alcanzada_ = true;
            
            if (config_.use_geomagic) {
                capturar_pose_inicial_haptico_ = true;
            }
            
            RCLCPP_INFO(this->get_logger(), "Pose inicial capturada. El control está activo.");
            RCLCPP_INFO(this->get_logger(), "q_init: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
                        robot_state_.q_init[0], robot_state_.q_init[1], robot_state_.q_init[2], robot_state_.q_init[3], robot_state_.q_init[4], robot_state_.q_init[5]);
            // Si geomagic es false y no hay movimiento inicial activo, arrancar trayectoria ahora
            if (!config_.use_geomagic && !init_move_active_) {
                trajectory_start_time_ = this->now();
                trajectory_active_ = true;
                RCLCPP_INFO(this->get_logger(), "Trayectoria automática ACTIVADA desde t=0 tras captura de pose inicial (sin movimiento inicial)." );
            }
        }
    }
    
    // Callback del Geomagic Touch
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "Mensaje nulo recibido en /phantom/pose.");
            return;
        }
        
        haptic_state_.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        haptic_state_.orientation.w() = msg->pose.orientation.w;
        haptic_state_.orientation.x() = msg->pose.orientation.x;
        haptic_state_.orientation.y() = msg->pose.orientation.y;
        haptic_state_.orientation.z() = msg->pose.orientation.z;



        
        //guardamos velocidad cartesiana del haptico por derivada de euler
        haptic_state_.velocity = (haptic_state_.position - haptic_state_.position_last) / (1e-3); //asumiendo 1ms entre callbacks
        haptic_state_.position_last = haptic_state_.position;
        Eigen::Quaterniond delta_orientation = haptic_state_.orientation * haptic_state_.orientation_last.inverse();
        // Aproximación de velocidad angular a partir de delta quaternion usando eje-ángulo
        haptic_state_.angular_velocity << delta_orientation.x()/(1e-3), delta_orientation.y()/(1e-3), delta_orientation.z()/(1e-3), delta_orientation.w()/(1e-3); //asumiendo 1ms entre callbacks
        haptic_state_.orientation_last = haptic_state_.orientation;
        haptic_state_.acceleration = (haptic_state_.velocity - haptic_state_.velocity_last) / (1e-3); //asumiendo 1ms entre callbacks
        haptic_state_.velocity_last = haptic_state_.velocity;
        

    }

    void button_callback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg){
        if (msg->grey_button == 1 && capturar_pose_inicial_haptico_){ //
            RCLCPP_INFO(this->get_logger(), "Botón gris presionado: Capturando pose inicial del Geomagic.");
            capturar_pose_inicial_haptico_ = false; // el boton solo sirve para capturar la pose inicial una vez
            RCLCPP_INFO(this->get_logger(), "Pose inicial háptica capturada.");

            haptic_state_.position_initial = haptic_state_.position;
            haptic_state_.orientation_initial = haptic_state_.orientation;
            
        }
        if (msg->grey_button == 1) {
                RCLCPP_INFO(this->get_logger(), "Botón gris presionado");
            }
        if (msg->white_button == 1) {
            RCLCPP_INFO(this->get_logger(), "Botón blanco presionado");
        }
    } 

    
    void control_loop() {
        auto loop_t0 = std::chrono::steady_clock::now();
        
        try {
            // Copiar parámetros dinámicos a locales (para aplicar cambios en caliente)
            Eigen::Vector3i map_pos_local;
            Eigen::Vector3d sign_pos_local;
            Eigen::Vector3i map_rot_local;
            Eigen::Vector3d sign_rot_local;
            {
                std::scoped_lock<std::mutex> lock(dynamic_params_mutex_);
                map_pos_local = map_pos_;
                sign_pos_local = sign_pos_;
                map_rot_local = map_rot_;
                sign_rot_local = sign_rot_;
            }

            if (!pose_inicial_capturada_ || !posicion_inicial_alcanzada_) {
                return;
            }
            if (config_.use_geomagic && capturar_pose_inicial_haptico_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Esperando a que se capture la pose inicial del Geomagic. Presione el botón gris.");
                return;
            }
            // Rama geomagic: seguir referencia del háptico
            if (config_.use_geomagic) {
            cartesian_state_.rotation_matrix_desired = cartesian_state_.rotation_matrix_initial;

            Eigen::Quaterniond dif_orientacion_haptic = haptic_state_.orientation_initial.inverse() * haptic_state_.orientation;
            Eigen::AngleAxisd angle_axis(dif_orientacion_haptic);
            Eigen::Vector3d axis_raw =  angle_axis.axis() * angle_axis.angle();
            double escala = 0.5; // factor de orientación
            Eigen::Vector3d axis_map;
            for(int i=0; i<3; i++) axis_map(i) = axis_raw(map_rot_local(i)) * sign_rot_local(i) * escala;
            if (axis_map.norm() > 1e-6){
                Eigen::Matrix3d R_delta = Eigen::AngleAxisd(axis_map.norm(), axis_map.normalized()).toRotationMatrix();
                cartesian_state_.rotation_matrix_desired = cartesian_state_.rotation_matrix_initial * R_delta;
            }
            
            // dif_orientacion_haptic = Eigen::Quaterniond(Eigen::AngleAxisd(escala * angle_axis.angle(), angle_axis.axis()));
            // cartesian_state_.orientation_desired = cartesian_state_.orientation_initial * dif_orientacion_haptic;

            RCLCPP_DEBUG(this->get_logger(), "Dif orientación háptico (eje): [%.3f, %.3f, %.3f]",
                        dif_orientacion_haptic.vec().x(), dif_orientacion_haptic.vec().y(), dif_orientacion_haptic.vec().z());
            
            for(int i=0; i<3; i++){
                cartesian_state_.position_desired(i) = cartesian_state_.position_initial(i) +
                    (haptic_state_.position(map_pos_local(i)) - haptic_state_.position_initial(map_pos_local(i))) * sign_pos_local(i)*2.5;     
            }
            

            
            // cartesian_state_.position_desired = trayectoria_geomagic(
            // cartesian_state_.position_initial, haptic_state_.position_initial, haptic_state_.position, 2.5);

            cartesian_state_.velocity = haptic_state_.velocity * 2.5; // escala de velocidad
            cartesian_state_.angular_velocity = haptic_state_.angular_velocity; // escala de velocidad
            cartesian_state_.acceleration = haptic_state_.acceleration * 2.5; // escala de aceleración
                
            
        } else {

            // Rama automática: generar trayectoria paramétrica
            if (!trajectory_active_) {
                // Aún no activada (esperando fin de movimiento inicial)
                return;
            }
            auto st = TrajectoryGenerator::calculate(
                cartesian_state_.position_initial,
                config_.traj_A,
                config_.traj_wn,
                config_.traj_c0,
                t_traj_,
                config_.traj_mode
            );
            
            cartesian_state_.position_desired = st.position;
            cartesian_state_.velocity = st.velocity;
            cartesian_state_.angular_velocity = Eigen::Vector4d::Zero(); // sin rot
            cartesian_state_.orientation_desired = cartesian_state_.orientation_initial;
            cartesian_state_.rotation_matrix_desired = cartesian_state_.rotation_matrix_initial;
            cartesian_state_.acceleration = st.acceleration;
            cartesian_state_.angular_acceleration = Eigen::Vector3d::Zero();

            //x_des << -0.03, 0.7, 0.1;
            // Mantener orientación constanteluego agregalo al cmakelist
            cartesian_state_.orientation_desired = cartesian_state_.orientation_initial;
            RCLCPP_DEBUG(this->get_logger(), "x_des: [%.3f, %.3f, %.3f]",
                cartesian_state_.position_desired.x(), cartesian_state_.position_desired.y(), cartesian_state_.position_desired.z());
            RCLCPP_DEBUG(this->get_logger(), "vel_des: [%.3f, %.3f, %.3f]",
                cartesian_state_.velocity.x(), cartesian_state_.velocity.y(), cartesian_state_.velocity.z());
        }
        t_traj_ += 0.005; // t_step = 0.005
            // Medición cartesiana actual (para logging y control si fuera necesario)
            pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
            pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);

            const auto& frame_meas = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
            cartesian_state_.position = frame_meas.translation();
            RCLCPP_DEBUG(this->get_logger(), "x_meas: [%.3f, %.3f, %.3f]",
                cartesian_state_.position.x(), cartesian_state_.position.y(), cartesian_state_.position.z());
        Eigen::Matrix3d R_meas = frame_meas.rotation();
        cartesian_state_.orientation = Eigen::Quaterniond(R_meas);

            Eigen::Matrix3d R_des = cartesian_state_.orientation_desired.toRotationMatrix();
            Eigen::Matrix3d R_err = R_meas.transpose() * R_des;
            double e_R_angle = Eigen::AngleAxisd(R_err).angle();
            // Quaterniones y Euler (roll-pitch-yaw) deseados y medidos
            Eigen::Quaterniond q_des = cartesian_state_.orientation_desired;
            Eigen::Quaterniond q_meas(R_meas);
            Eigen::Vector3d euler_des = R_des.eulerAngles(0, 1, 2); // RPY
            Eigen::Vector3d euler_meas = R_meas.eulerAngles(0, 1, 2); // RPY

            auto ik_t0 = std::chrono::steady_clock::now();

            if (config_.controller == "QP") {
                robot_state_.q_solution = kinematics_solver_->inverseKinematicsQP2(
                    robot_state_.q,
                    cartesian_state_.position_desired,
                    cartesian_state_.rotation_matrix_desired,
                    600,
                    config_.ctrl_hz_
                );
                robot_state_.u_control = kinematics_solver_->inverseKinematicsQP2(
                    robot_state_.q,
                    cartesian_state_.position_desired,
                    cartesian_state_.rotation_matrix_desired,
                    600,
                    config_.ctrl_hz_
                ); // No se calculan esfuerzos en QP, solo posición objetivo
            }
            else if (config_.controller == "IMP") {
                robot_state_.q_solution = impedance_controller_->calculateControlCommand(
                    robot_state_.q,
                    robot_state_.qd,
                    cartesian_state_.position_desired,
                    cartesian_state_.orientation_desired,
                    cartesian_state_.velocity,
                    cartesian_state_.acceleration,
                    config_.Kp,
                    config_.Kd,
                    config_.dt).q_desired;
                robot_state_.u_control = impedance_controller_->calculateControlCommand(
                    robot_state_.q,
                    robot_state_.qd,
                    cartesian_state_.position_desired,
                    cartesian_state_.orientation_desired,
                    cartesian_state_.velocity,
                    cartesian_state_.acceleration,
                    config_.Kp,
                    config_.Kd,
                    config_.dt).tau;
            }
            else if (config_.controller == "SLD") {                
                // Eigen::Matrix<double, 6, 1> lambda = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
                // Eigen::Matrix<double, 6, 1> k = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
                // Eigen::Matrix<double, 6, 1> k2 = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
                // double alpha = 0.01;
                // double damping_factor = 0.01;
                // double dt = 0.01;
                robot_state_.q_solution = sliding_controller_->calculateControlCommand(
                    robot_state_.q,
                    robot_state_.qd,
                    cartesian_state_.position_desired,
                    cartesian_state_.rotation_matrix_desired,
                    cartesian_state_.velocity,
                    Eigen::Vector3d::Zero(), // desired_vel_ori (no control de velocidad angular en este ejemplo)
                    cartesian_state_.acceleration,
                    Eigen::Vector3d::Zero(), // desired_acc_ori
                    config_.lambda,
                    config_.k,
                    config_.k2,
                    config_.alpha,
                    config_.damping_factor,
                    config_.dt).q_desired;
                robot_state_.u_control = sliding_controller_->calculateControlCommand(
                    robot_state_.q,
                    robot_state_.qd,
                    cartesian_state_.position_desired,
                    cartesian_state_.rotation_matrix_desired,
                    cartesian_state_.velocity,
                    Eigen::Vector3d::Zero(), // desired_vel_ori
                    cartesian_state_.acceleration,
                    Eigen::Vector3d::Zero(), // desired_acc_ori
                    config_.lambda,
                    config_.k,
                    config_.k2,
                    config_.alpha,
                    config_.damping_factor,
                    config_.dt).tau;
            }
            else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Controlador desconocido: %s. Usando QP.", config_.controller.c_str());
                config_.controller = "QP";
                robot_state_.q_solution = kinematics_solver_->inverseKinematicsQP2(
                    robot_state_.q,
                    cartesian_state_.position_desired,
                    cartesian_state_.rotation_matrix_desired,
                    600,
                    config_.ctrl_hz_
                );
            }
        
            last_ik_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - ik_t0)
                               .count() / 1000.0;
            RCLCPP_INFO(this->get_logger(), "Tiempo IK: %.3f ms", last_ik_ms_);

            // === INTERPOLACIÓN ADAPTATIVA PARA COMANDOS ALEJADOS ===
            // Inicializar target interpolado en el primer ciclo
            if (!q_cmd_initialized_) {
                q_target_interpolated_ = robot_state_.q;
                q_solution_prev_ = robot_state_.q_solution;
            }
            
            // Detectar cambios bruscos en la solución IK (operador movió háptico nuevamente)
            Eigen::VectorXd ik_change = robot_state_.q_solution - q_solution_prev_;
            double max_ik_change = ik_change.cwiseAbs().maxCoeff();
            bool ik_changed_abruptly = (max_ik_change > max_ik_change_threshold_rad_);
            
            // Calcular error entre posición actual y solución IK
            Eigen::VectorXd q_error = robot_state_.q_solution - robot_state_.q;
            double max_error = q_error.cwiseAbs().maxCoeff();
            
            // Si hay un error grande (movimiento brusco del háptico)
            if (max_error > large_error_threshold_rad_) {
                // Si la solución IK cambió drásticamente, reiniciar interpolación
                if (ik_changed_abruptly) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "Cambio brusco en solución IK detectado (%.3f rad). Reiniciando interpolación.",
                        max_ik_change);
                    q_target_interpolated_ = robot_state_.q;  // Reiniciar desde posición actual
                } else {
                    // Interpolar gradualmente hacia la solución alejada
                    Eigen::VectorXd direction = (robot_state_.q_solution - q_target_interpolated_).normalized();
                    double remaining_distance = (robot_state_.q_solution - q_target_interpolated_).norm();
                    
                    // Avanzar un paso limitado hacia el objetivo
                    double step_size = std::min(max_joint_step_rad_, remaining_distance);
                    q_target_interpolated_ += direction * step_size;
                    
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "Error articular grande detectado: %.3f rad. Interpolando suavemente (restante: %.3f rad)",
                        max_error, remaining_distance);
                }
            } else {
                // Error pequeño: seguir normalmente la solución IK
                q_target_interpolated_ = robot_state_.q_solution;
            }
            
            // Guardar solución IK actual para próximo ciclo
            q_solution_prev_ = robot_state_.q_solution;
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "q_actual: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                robot_state_.q[0], robot_state_.q[1], robot_state_.q[2], robot_state_.q[3], robot_state_.q[4], robot_state_.q[5]);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "q_solution: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                robot_state_.q_solution[0], robot_state_.q_solution[1], robot_state_.q_solution[2], robot_state_.q_solution[3], robot_state_.q_solution[4], robot_state_.q_solution[5]);    
        // --------------------------------------------------------------------------------
        
            // Filtro pasabajo sobre el target interpolado (no sobre q_solution directamente)
            if (!q_cmd_initialized_) {
                q_cmd_filtered_ = q_target_interpolated_;
                q_cmd_initialized_ = true;
            } else {
                q_cmd_filtered_ = q_cmd_alpha_ * q_target_interpolated_ + (1.0 - q_cmd_alpha_) * q_cmd_filtered_;
            }

            // === PUBLICACIÓN CON GARANTÍA ===
            // Siempre publicar para mantener heartbeat con el driver
            try {
                auto position_msg = std_msgs::msg::Float64MultiArray();
                position_msg.data.assign(q_cmd_filtered_.data(), q_cmd_filtered_.data() + q_cmd_filtered_.size());
                joint_position_pub_->publish(position_msg);
                
                // Registro exitoso de publicación
                consecutive_publish_failures_ = 0;
                last_successful_publish_ = std::chrono::steady_clock::now();
            } catch (const std::exception& e) {
                consecutive_publish_failures_++;
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Error publicando comando de posición: %s (fallos consecutivos: %d)",
                    e.what(), consecutive_publish_failures_);
                
                if (consecutive_publish_failures_ >= MAX_CONSECUTIVE_FAILURES) {
                    RCLCPP_FATAL(this->get_logger(),
                        "Demasiados fallos consecutivos de publicación (%d). Revisar conexión ethernet.",
                        consecutive_publish_failures_);
                }
            }

            // Tiempo total del ciclo (hasta después de publicar)
            last_loop_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - loop_t0)
                        .count() / 1000.0;

            // === WATCHDOG: Detectar delays del loop ===
            if (last_loop_ms_ > loop_timeout_ms_) {
                consecutive_loop_delays_++;
                if (consecutive_loop_delays_ >= MAX_CONSECUTIVE_DELAYS) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Loop lento detectado: %.2f ms (esperado <2ms a 500Hz). Fallos consecutivos: %d. "
                        "Posible causa: IK solver, FK, logging CSV, o saturación de CPU.",
                        last_loop_ms_, consecutive_loop_delays_);
                }
            } else {
                consecutive_loop_delays_ = 0;
            }

            // === Logging de timeout ===
            auto now = std::chrono::steady_clock::now();
            double ms_since_last_publish = std::chrono::duration<double, std::milli>(
                now - last_successful_publish_).count();
            if (ms_since_last_publish > 100.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "ADVERTENCIA: %.0f ms sin publicación exitosa. Driver UR podría desconectarse.",
                    ms_since_last_publish);
            }

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Control Loop] ik: %.3f ms | loop: %.3f ms | publish: OK",
                last_ik_ms_, last_loop_ms_);

            // Logging CSV (sin bloqueo, en thread de fondo ideal)
            if (config_.csv_enabled) {
                try {
                    rclcpp::Time now_ros = this->now();
                    double t = (now_ros - start_time_).seconds();
                    double dt = (now_ros - last_log_time_).seconds();
                    last_log_time_ = now_ros;
                    // Calcular errores cartesianos
                    Eigen::Vector3d pos_err = cartesian_state_.position_desired - cartesian_state_.position; // error de posición
                    RCLCPP_DEBUG(this->get_logger(), "pos_err: [%.3f, %.3f, %.3f]", pos_err.x(), pos_err.y(), pos_err.z());
                    // Para orientación: quaternion de error (desired * meas^{-1})
                    Eigen::Quaterniond q_err = q_des * cartesian_state_.orientation.inverse();
                    Eigen::AngleAxisd aa(q_err);
                    Eigen::Vector3d ori_axis = aa.axis();
                    double ori_angle = aa.angle();
                    write_csv_row(t, dt, cartesian_state_.position_desired, cartesian_state_.position, q_des, cartesian_state_.orientation, euler_des, euler_meas, e_R_angle,
                                  pos_err, ori_axis, ori_angle, robot_state_.u_control,
                                  last_ik_ms_, last_loop_ms_);
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Error escribiendo CSV: %s", e.what());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Excepción en control_loop: %s. Continuando con último comando válido.", e.what());
            // Intentar publicar última posición conocida para mantener conexión
            try {
                auto position_msg = std_msgs::msg::Float64MultiArray();
                position_msg.data.assign(q_cmd_filtered_.data(), q_cmd_filtered_.data() + q_cmd_filtered_.size());
                joint_position_pub_->publish(position_msg);
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "No se puede publicar comando. Desconexión probable.");
            }
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR5IKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// ros2 run ur5_controller controller_backup --ros-args -p control_topic:="/scaled_joint_trajectory_controller/joint_trayectory" -p ur:="ur5e" -p nmspace:="ur5e" -p geomagic_topic:="/phantom2/pose" -p geomagic_button_topic:="/phantom2/button" -p csv_log_enable:="true" -p geomagic:="true"




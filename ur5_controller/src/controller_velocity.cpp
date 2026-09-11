

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
#include <fstream>
#include <sstream>

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
            state.position.x() = x_init.x() + r_x * sin_wn_t;
            state.position.y() = x_init.y() + r_y * (cos_wn_t - 1.0);
            state.position.z() = x_init.z();

            state.velocity.x() = r_x * wn * cos_wn_t;
            state.velocity.y() = -r_y * wn * sin_wn_t;
            state.velocity.z() = 0.0;

            state.acceleration.x() = -r_x * wn * wn * sin_wn_t;
            state.acceleration.y() = -r_y * wn * wn * cos_wn_t;
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
        this->declare_parameter<std::string>("robot_description", "");
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
        std::string robot_description_param = this->get_parameter("robot_description").as_string();
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

        // El URDF puede llegar ya resuelto (post-xacro) vía el parámetro estándar
        // 'robot_description', lo que permite que cada robot traiga su propia
        // herramienta definida en su .urdf.xacro. Si no se recibe, se cae al
        // comportamiento legado: leer un archivo URDF desde 'urdf_path'.
        if (!robot_description_param.empty()) {
            urdf_xml_ = robot_description_param;
            urdf_path_ = "<robot_description parameter>";
        } else {
            if (urdf_path_.empty()) {
                urdf_path_ = "/home/david/tesis_ws/src/ur5_simulation/ur5_description/urdf/ur5e.urdf";
            }
            std::ifstream urdf_file(urdf_path_);
            if (!urdf_file.is_open()) {
                throw std::runtime_error("No se pudo abrir el archivo URDF: " + urdf_path_);
            }
            std::stringstream urdf_buffer;
            urdf_buffer << urdf_file.rdbuf();
            urdf_xml_ = urdf_buffer.str();
        }

        // Cargar modelo pinocchio
        RCLCPP_INFO(this->get_logger(), "Cargando modelo URDF desde: %s", urdf_path_.c_str());

        try {
            model_ = std::make_unique<pinocchio::Model>();
            pinocchio::urdf::buildModelFromXML(urdf_xml_, *model_);
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
    std::string urdf_xml_;

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
            pinocchio::urdf::buildModelFromXML(urdf_xml_, *model_);
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
            
            pinocchio::computeFrameJacobian(*model_, *data_, q_, tool_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,jacobian);
            //pinocchio::computeFrameJacobian(*model_, *data_, q_, ee_frame_id, jacobian);
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
            v_ee_desired_.head(3) = Kp_pos * position_error+ Kd_pos * d_position_error;//+ velocity_reference  ;
            v_ee_desired_.tail(3) = Kp_orient * orientation_error + Kd_orient * d_orientation_error;

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
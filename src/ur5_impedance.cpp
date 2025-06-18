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

Eigen::VectorXd computeError(pinocchio::Data& data, 
                             const pinocchio::FrameIndex& tool_frame_id, const pinocchio::SE3& desired_pose) {
    /* const pinocchio::SE3 current_pose = data.oMf[tool_frame_id];
    
    const pinocchio::SE3 error_pose = current_pose.actInv(desired_pose);
    return pinocchio::log6(error_pose).toVector(); // Error en SE(3) */
    const pinocchio::SE3 current_pose = data.oMf[tool_frame_id];

    // Error de posición
    Eigen::Vector3d position_error = desired_pose.translation() - current_pose.translation();

    // Error de orientación usando cuaterniones
    Eigen::Quaterniond current_orientation = Eigen::Quaterniond(current_pose.rotation());
    Eigen::Quaterniond desired_orientation = Eigen::Quaterniond(desired_pose.rotation());

    Eigen::Quaterniond error_quat = desired_orientation * current_orientation.inverse();
    Eigen::Vector3d angular_error;

    // Convertir el cuaternión de error a una representación vectorial del error angular
    if (error_quat.w() < 0) {
        error_quat.w() = -error_quat.w(); // Asegurar la parte escalar positiva para la representación del error más corta
    }
    angular_error = error_quat.vec(); // El vector parte del cuaternión representa el eje de rotación escalado por sin(ángulo/2)

    Eigen::VectorXd error(6);
    error << position_error, angular_error;
    return error;
}




Eigen::VectorXd impedanceControl6D(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const pinocchio::FrameIndex& tool_frame_id,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const pinocchio::SE3& desired_pose,
    const Eigen::Matrix<double,6,1>& dx_d,
    const Eigen::Matrix<double,6,1>& ddx_d,
    const Eigen::Matrix<double,6,6>& Kp,
    const Eigen::Matrix<double,6,6>& Kd,
    const Eigen::Matrix<double,6,1>& F_ext = Eigen::Matrix<double,6,1>::Zero()
) {
    // Cinemática directa y jacobiano
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::updateFramePlacement(model, data, tool_frame_id);
    const pinocchio::SE3& current_pose = data.oMf[tool_frame_id];

    // Error de posición
    Eigen::Vector3d pos_error = desired_pose.translation() - current_pose.translation();

    // Error de orientación (ángulo-eje
    Eigen::Matrix3d R_err = desired_pose.rotation() * current_pose.rotation().transpose();
    Eigen::AngleAxisd aa(R_err);
    Eigen::Vector3d ori_error = aa.axis() * aa.angle();

    // Error total 6D
    Eigen::Matrix<double,6,1> e;
    e.head<3>() = pos_error;
    e.tail<3>() = ori_error;

    // Jacobiano espacial 6xN
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, tool_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

    // Velocidad espacial actual
    Eigen::VectorXd v = J * dq; // 6x1

    // Error de velocidad
    Eigen::Matrix<double,6,1> de = dx_d - v;

    // Dinámica
    pinocchio::computeJointJacobians(model, data, q); // J
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);// J_dot
    pinocchio::crba(model, data, q); // M
    pinocchio::computeCoriolisMatrix(model, data, q, dq); // C
    pinocchio::computeGeneralizedGravity(model, data, q); // g

    Eigen::MatrixXd M = data.M;
    Eigen::VectorXd b = data.nle; // Coriolis + gravedad
    Eigen::MatrixXd M_inv;
    
    // Asegurarse de que M sea invertible
    if (M.determinant() == 0) {
        throw std::runtime_error("Matriz de inercia no invertible");
        //User un pseudo-inverso o manejar el caso de singularidad
        M_inv = M.completeOrthogonalDecomposition().pseudoInverse();
        //return J.transpose() * (J * M_inv * J.transpose()).inverse() * (J * M_inv * b - J * M_inv * data.g);             
    }
    else {
        M_inv = M.inverse(); // Inversa de la matriz de inercia
    }
    // Inercia operacional (Lambda)
    Eigen::MatrixXd Lambda = (J * M_inv * J.transpose()).inverse();

    // Término de acoplamiento (mu) y gravedad operacional (p)
    Eigen::VectorXd mu = Lambda * (J * M_inv * b - J * M_inv * data.g);
    Eigen::VectorXd p = Lambda * (J * M_inv * data.g);

    // Ley de impedancia operacional
    Eigen::Matrix<double,6,1> F_star = ddx_d + Kd * de + Kp * e + F_ext;  
    Eigen::Matrix<double,6,1> F = Lambda * F_star + mu + p;

    // Torque articular: tau = J^T * F
    Eigen::VectorXd tau = J.transpose() * F;

    return tau;
}

class UR5eJointController : public rclcpp::Node {
    public:
        UR5eJointController() : Node("ur5e_joint_controller"), time_elapsed_(0.0) {
            q_ = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero
            q_ << 0,-1.57,1.57,0,0,0; // Asigna los valores deseados

            qd_ = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero
            qd_anterior = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero
            qdd_ = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero

            q_init = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero
            q_init << 0,-1.57,1.57,0,0,0; // Asigna los valores deseados

            x_des = Eigen::Vector3d::Zero(); // Inicializa con 3 elementos, todos a cero
            x_des << 0.5, 0.0, 0.5; // Asigna los valores deseados

            dx_des = Eigen::VectorXd::Zero(6); // Inicializa con 3 elementos, todos a cero
            ddx_des = Eigen::VectorXd::Zero(6); // Inicializa con 3 elementos, todos a cero

            initializeUR5(model, data, tool_frame_id, urdf_path);            // Publicador para enviar trayectorias
            joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(control_topic, 10);
    
            // Suscriptor para leer el estado articular actual
            subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&UR5eJointController::update_joint_positions, this, std::placeholders::_1));            
                // Suscriptor para leer la posición cartesiana del haptic phantom
            
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&UR5eJointController::posicion_inicial, this));
        }
    
    private:
        bool posicion_inicial_alcanzada_ = false; // Bandera para sincronización
        double time_elapsed_=0.0; // Variable

        double r_[3] = {0, 0, 0}; // Posiciones del haptic phantom
        double qt_[4] = {1, 0, 0, 0}; // Orientacion del haptic phantom  
        double qt_inv[4] = {1, 0, 0, 0}; // Orientacion del haptic phantom inverso
        std::ofstream output_file_;
        std::ofstream output_file_2;
        std::ofstream output_file_3;

        Eigen::Vector3d euler_angles;
        Eigen::Matrix3d rotation_matrix;
        Eigen::Quaterniond q_x;        Eigen::Quaterniond q_y;        Eigen::Quaterniond q_z;        
        Eigen::Quaterniond quat_initial_UR5; 
        Eigen::Quaterniond quat_initial_geo; 
        Eigen::Quaterniond quat_real_geo; 
        
        
        Eigen::VectorXd q_;
        Eigen::VectorXd qd_;
        Eigen::VectorXd qd_anterior;
        Eigen::VectorXd qdd_;

        Eigen::VectorXd q_init; // Inicialización de las posiciones articulares del UR50
        double x_init[3]; 
        Eigen::Vector3d x_des; // Posición deseada del UR5
        Eigen::VectorXd dx_des; // Velocidad deseada del UR5 en espacio cartesiano
        Eigen::VectorXd ddx_des;
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
        Eigen::MatrixXd J_anterior= Eigen::MatrixXd::Zero(6, 6); // Inicializar J_anterior como una matriz de ceros
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
            RCLCPP_INFO(this->get_logger(), "Posiciones articulares: %.4f %.4f %.4f %.4f %.4f %.4f",
                        q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
            // Imprimir las velocidades articulares
            RCLCPP_INFO(this->get_logger(), "Velocidades articulares: %.4f %.4f %.4f %.4f %.4f %.4f",
                        qd_[0], qd_[1], qd_[2], qd_[3], qd_[4], qd_[5]);
        }
        
        
        void posicion_inicial() {
            //calcula el jacobiano y la cinemática directa
            pinocchio::forwardKinematics(*model, *data, q_);
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
            Eigen::MatrixXd J = jacobian(*model, *data, tool_frame_id, q_);
            cout << "Jacobian: " << J << endl;
            // Derivada de Jacobiano
            Eigen::MatrixXd J_dot = (J-J_anterior)/0.01; // 0.01 es el tiempo de control en segundos
            cout << "Jacobian Derivative: " << J_dot << endl;
            // aceleración articular
            qdd_ = (qd_-qd_anterior)/0.01; // 0.01 es el tiempo de control en segundos

            

            Eigen::Quaterniond desired_orientation(qt_[0], qt_[1], qt_[2], qt_[3]);//(,,,w)
            desired_orientation.normalize(); // Asegúrate de que el cuaternión esté normalizado
            pinocchio::SE3 desired_pose_eigen(desired_orientation.toRotationMatrix(), x_des);
            Eigen::VectorXd x_e = computeError(*data, tool_frame_id, desired_pose_eigen); //e = {ex,ey,ez,exy,exz,eyz}

           
            // Calcula la velocidad cartesiana actual (vx,vy,vz,wx,wy,wz)
            Eigen::VectorXd dx = J * qd_;  // Velocidad cartesiana actual
            // Calcula la velocidad angular actual
            Eigen::Matrix<double,6,1> de = dx_des - dx;  // e_dot = {ex_dot, ey_dot, ez_dot, exy_dot, exz_dot, eyz_dot}

            pinocchio::computeJointJacobians(*model, *data, q_); // J
            pinocchio::computeJointJacobiansTimeVariation(*model, *data, q_, qd_);// J_dot
            pinocchio::crba(*model, *data, q_); // M
            pinocchio::computeCoriolisMatrix(*model, *data, q_, qd_); // C
            pinocchio::computeGeneralizedGravity(*model, *data, q_); // g

            Eigen::MatrixXd M = (*data).M;
            Eigen::VectorXd b = (*data).nle; // Coriolis + gravedad

            


            Eigen::Matrix<double,6,6> K = Eigen::Matrix<double,6,6>::Identity();
            // Ejemplo de sintonización (ajústalo a tus necesidades)
            // Para los 3 primeros elementos (lineal) y los 3 últimos (angular)
            K.diagonal() << 200.0, 200.0, 200.0, 50.0, 50.0, 50.0;

            Eigen::Matrix<double,6,6> B = Eigen::Matrix<double,6,6>::Identity();
            // Ejemplo de sintonización
            B.diagonal() << 20.0, 20.0, 20.0, 5.0, 5.0, 5.0;


            // Llama a tu función de control de impedancia
            // F_ext_zero es para fuerzas externas, puedes inicializarla a cero si no hay
            Eigen::Matrix<double,6,1> F_ext_zero = Eigen::Matrix<double,6,1>::Zero();

            Eigen::VectorXd tau = impedanceControl6D(
                *model, *data, tool_frame_id, q_, qd_,
                desired_pose_eigen, // pinocchio::SE3
                dx_des,             // Eigen::Matrix<double,6,1> para velocidad cartesiana deseada
                ddx_des,            // Eigen::Matrix<double,6,1> para aceleración cartesiana deseada
                K,
                B,
                F_ext_zero
            );

            // Después de obtener tau, calcula qdd y actualiza q_ y qd_
            // Asegúrate de que *data.M esté actualizada para calcular M_inv
            pinocchio::crba(*model, *data, q_); // Asegura que la matriz M de Pinocchio esté actualizada
            Eigen::MatrixXd M = (*data).M;
            Eigen::MatrixXd M_inv;
            if (M.determinant() == 0) {
                RCLCPP_ERROR(this->get_logger(), "Matriz de inercia no invertible en posicion_inicial(). Usando pseudo-inversa.");
                M_inv = M.completeOrthogonalDecomposition().pseudoInverse();
            } else {
                M_inv = M.inverse();
            }

            Eigen::VectorXd qdd = M_inv * tau; // Aceleración articular

            // Integrar velocidades y posiciones
            qd_ += qdd * 0.01;
            q_ += qd_ * 0.01;
            



            // q_[0] = 0.0 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q1
            // q_[1] = -1.57 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q2
            // q_[2] = 1.57 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q3
            // q_[3] = 0.0 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q4
            // q_[4] = 0.0 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q5
            // q_[5] = 0.0 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q6
            time_elapsed_ += 0.01; // Incrementa el tiempo transcurrido


            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            RCLCPP_INFO(this->get_logger(), "Enviando posición: %.4f %.4f %.4f %.4f %.4f %.4f",
                        q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
            point.positions = {q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]};
            point.time_from_start = rclcpp::Duration::from_seconds(0.1);
            trajectory_msg.points.push_back(point);
            joint_trajectory_pub_->publish(trajectory_msg);
            J_anterior = J; // Actualizar J_anterior con el Jacobiano actual
            qd_anterior = qd_; // Actualizar qd_anterior con la velocidad actual
            
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


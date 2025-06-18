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

Eigen::VectorXd impedanceControlPythonStyle(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const pinocchio::FrameIndex& tool_frame_id,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const pinocchio::SE3& desired_pose, // Para xdes
    const Eigen::VectorXd& dx_des,      // ddxdes
    const Eigen::VectorXd& ddx_des,     // ddxdes
    const Eigen::Matrix<double,6,6>& Kp_task, // Matriz de rigidez (Kd en Python)
    const Eigen::Matrix<double,6,6>& Kd_task, // Matriz de amortiguamiento (Bd en Python)
    double dt,
    Eigen::MatrixXd& J_anterior // Pasado por referencia para actualizar y usar
) {
    // 1. Cinemática directa y Jacobiano actual
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::updateFramePlacement(model, data, tool_frame_id);

    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, tool_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

    // 2. Derivada del Jacobiano (dJ en Python)
    // Se asume que J_anterior se actualiza después de cada llamada a esta función.
    Eigen::MatrixXd dJ = (J - J_anterior) / dt;

    // 3. Obtener x y dx (posición y velocidad cartesiana actual)
    Eigen::Vector3d x_current_pos = data.oMf[tool_frame_id].translation();
    // Para la orientación, necesitamos extraerla de data.oMf[tool_frame_id] y la deseada
    // Usaremos la función computeError para obtener el error completo y las posiciones/orientaciones
    
    // Obtener la velocidad cartesiana actual
    Eigen::VectorXd dx_current_cartesian = J * dq; // (6x6) * (6x1) = 6x1

    // 4. Calcular el error (xdes - x y dxdes - dx)
    // Para el error de posición/orientación, usaremos tu función computeError existente
    // que devuelve un vector 6x1 (pos_error, ang_error)
    Eigen::VectorXd error_pose = computeError(data, tool_frame_id, desired_pose); // (x_d - x)

    // Error de velocidad (dx_des - dx_current_cartesian)
    Eigen::VectorXd error_velocity = dx_des - dx_current_cartesian; // (dx_d - dx)


    // 5. Cálculos de dinámica (M, c, g)
    // Necesitamos recalcular estas para cada paso de tiempo
    pinocchio::computeJointJacobians(model, data, q); // Necesario para crba y otros
    pinocchio::crba(model, data, q); // Calcula la matriz de inercia M
    pinocchio::computeCoriolisMatrix(model, data, q, dq); // Calcula el término de Coriolis
    pinocchio::computeGeneralizedGravity(model, data, q); // Calcula el vector de gravedad g

    Eigen::MatrixXd M = data.M; // Matriz de inercia
    // data.nle ya es (Coriolis + Gravity) en Pinocchio
    Eigen::VectorXd c_term = data.nle - data.g; // Solo el término de Coriolis (c en Python)
    Eigen::VectorXd g_term = data.g; // Solo el término de gravedad (g en Python)

    // 6. Calcular Md (Matriz de inercia deseada en el espacio de tarea)
    // Asegurarse de usar pseudo-inversa como en Python
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse(); // np.linalg.pinv(J)

    // Md = (J_pinv.T) * M * J_pinv
    // Dimensiones: (6x6).transpose() * (6x6) * (6x6) = (6x6) * (6x6) * (6x6) = 6x6
    Eigen::Matrix<double,6,6> Md = J_pinv.transpose() * M * J_pinv;

    // Asegurarse de que Md sea invertible para Md.inverse()
    // En la práctica, Md puede volverse singular si J es singular, aunque la pseudo-inversa lo maneja.
    // Una opción más robusta sería usar la pseudo-inversa de Md si es necesario.
    Eigen::Matrix<double,6,6> Md_inv;
    if (Md.determinant() == 0) {
        // Esto puede ocurrir si J es singular, haciendo Md singular.
        // Se podría usar la pseudo-inversa de Md para mayor robustez
        // Md_inv = Md.completeOrthogonalDecomposition().pseudoInverse();
        // Por simplicidad, por ahora lanzamos un error o manejamos la excepción
        throw std::runtime_error("Matriz Md singular en impedanceControlPythonStyle.");
    } else {
        Md_inv = Md.inverse(); // np.linalg.pinv(Md) si Md es singular
    }


    // 7. Calcular el torque final
    // torque = M@(np.linalg.pinv(J))@(ddxdes - dJ@dq + np.linalg.pinv(Md)@(Bd@(dxdes-dx) + Kd@(xdes - x))) + c + g
    //           ^----------- Termino A -----------^ ^------------------- Termino B -------------------^   ^---- Termino C ----^

    // Término A: M * J_pinv
    Eigen::MatrixXd TermA = M * J_pinv; // (6x6) * (6x6) = 6x6

    // Término B: (ddxdes - dJ@dq + Md_inv@(Kd_task@(error_pose) + Kd_task@(error_velocity)))
    // Nota: Kp_task es Kd en Python, Kd_task es Bd en Python
    Eigen::VectorXd InnerTermB = Kd_task * error_velocity + Kp_task * error_pose; // (6x6)*(6x1) + (6x6)*(6x1) = 6x1
    Eigen::VectorXd TermB_part2 = Md_inv * InnerTermB; // (6x6)*(6x1) = 6x1

    Eigen::VectorXd TermB = ddx_des - (dJ * dq) + TermB_part2; // (6x1) - (6x1) + (6x1) = 6x1

    // Término C: c_term + g_term
    Eigen::VectorXd TermC = c_term + g_term; // (6x1) + (6x1) = 6x1

    // Torque final: TermA * TermB + TermC
    Eigen::VectorXd tau = TermA * TermB + TermC; // (6x6) * (6x1) + (6x1) = 6x1

    // Actualizar J_anterior para la próxima iteración
    J_anterior = J;

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
            
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&UR5eJointController::posicion_inicial, this));
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
            //RCLCPP_INFO(this->get_logger(), "Posiciones articulares: %.4f %.4f %.4f %.4f %.4f %.4f",   q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
            // Imprimir las velocidades articulares
            //RCLCPP_INFO(this->get_logger(), "Velocidades articulares: %.4f %.4f %.4f %.4f %.4f %.4f",  qd_[0], qd_[1], qd_[2], qd_[3], qd_[4], qd_[5]);
        }
        
        
        void posicion_inicial() {
            // //calcula el jacobiano y la cinemática directa
            // pinocchio::forwardKinematics(*model, *data, q_);
            // pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
            // Eigen::MatrixXd J = jacobian(*model, *data, tool_frame_id, q_);
            // cout << "Jacobian: " << J << endl;
            // // Derivada de Jacobiano
            // Eigen::MatrixXd J_dot = (J-J_anterior)/0.01; // 0.01 es el tiempo de control en segundos
            // cout << "Jacobian Derivative: " << J_dot << endl;
            // // aceleración articular
            // qdd_ = (qd_-qd_anterior)/0.01; // 0.01 es el tiempo de control en segundos

            

            // Eigen::Quaterniond desired_orientation(qt_[0], qt_[1], qt_[2], qt_[3]);//(,,,w)
            // desired_orientation.normalize(); // Asegúrate de que el cuaternión esté normalizado
            // pinocchio::SE3 desired_pose_eigen(desired_orientation.toRotationMatrix(), x_des);
            // Eigen::VectorXd x_e = computeError(*data, tool_frame_id, desired_pose_eigen); //e = {ex,ey,ez,exy,exz,eyz}

           
            // // Calcula la velocidad cartesiana actual (vx,vy,vz,wx,wy,wz)
            // Eigen::VectorXd dx = J * qd_;  // Velocidad cartesiana actual
            // // Calcula la velocidad angular actual

            // pinocchio::computeJointJacobians(*model, *data, q_); // J
            // pinocchio::computeJointJacobiansTimeVariation(*model, *data, q_, qd_);// J_dot
            // pinocchio::crba(*model, *data, q_); // M
            // pinocchio::computeCoriolisMatrix(*model, *data, q_, qd_); // C
            // pinocchio::computeGeneralizedGravity(*model, *data, q_); // g

            // Eigen::MatrixXd M = (*data).M;
            // Eigen::VectorXd b = (*data).nle; // Coriolis + gravedad

            // Eigen::Matrix<double,6,6> K = Eigen::Matrix<double,6,6>::Identity();
            // // Ejemplo de sintonización (ajústalo a tus necesidades)
            // // Para los 3 primeros elementos (lineal) y los 3 últimos (angular)
            // K.diagonal() << 20.0, 20.0, 20.0, 50.0, 50.0, 50.0;

            // Eigen::Matrix<double,6,6> B = Eigen::Matrix<double,6,6>::Identity();
            // // Ejemplo de sintonización
            // B.diagonal() << 20.0, 20.0, 20.0, 5.0, 5.0, 5.0;


            // // Llama a tu función de control de impedancia
            // // F_ext_zero es para fuerzas externas, puedes inicializarla a cero si no hay
            // Eigen::Matrix<double,6,1> F_ext_zero = Eigen::Matrix<double,6,1>::Zero();

            // Eigen::VectorXd tau = impedanceControl6D(
            //     *model, *data, tool_frame_id, q_, qd_,
            //     desired_pose_eigen, // pinocchio::SE3
            //     dx_des,             // Eigen::Matrix<double,6,1> para velocidad cartesiana deseada
            //     ddx_des,            // Eigen::Matrix<double,6,1> para aceleración cartesiana deseada
            //     K,
            //     B,
            //     F_ext_zero
            // );

            // // Después de obtener tau, calcula qdd y actualiza q_ y qd_
            // // Asegúrate de que *data.M esté actualizada para calcular M_inv
            // pinocchio::crba(*model, *data, q_); // Asegura que la matriz M de Pinocchio esté actualizada
            
            // Eigen::MatrixXd M_inv;
            // if (M.determinant() == 0) {
            //     RCLCPP_ERROR(this->get_logger(), "Matriz de inercia no invertible en posicion_inicial(). Usando pseudo-inversa.");
            //     M_inv = M.completeOrthogonalDecomposition().pseudoInverse();
            // } else {
            //     M_inv = M.inverse();
            // }

            // Eigen::VectorXd qdd = M_inv * tau; // Aceleración articular

            // // Integrar velocidades y posiciones
            // qd_ += qdd * 0.01;
            // q_ += qd_ * 0.01;
            



            // // q_[0] = 0.0 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q1
            // // q_[1] = -1.57 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q2
            // // q_[2] = 1.57 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q3
            // // q_[3] = 0.0 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q4
            // // q_[4] = 0.0 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q5
            // // q_[5] = 0.0 + 0.5 * sin(2 * M_PI * 0.1 * time_elapsed_); // Movimiento sinusoidal en q6
            // time_elapsed_ += 0.01; // Incrementa el tiempo transcurrido


            // auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            // trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            //                             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            // auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            // RCLCPP_INFO(this->get_logger(), "Enviando posición: %.4f %.4f %.4f %.4f %.4f %.4f",
            //             q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
            // point.positions = {q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]};
            // point.time_from_start = rclcpp::Duration::from_seconds(0.2);
            // trajectory_msg.points.push_back(point);
            // joint_trajectory_pub_->publish(trajectory_msg);
            // J_anterior = J; // Actualizar J_anterior con el Jacobiano actual
            // qd_anterior = qd_; // Actualizar qd_anterior con la velocidad actual

            // ... (parte inicial de la función, incluyendo la obtención de q_ y qd_)

            // 1. Define tu pose deseada (posición y orientación)
            // Para simplificar, la posición deseada podría ser fija o variar sinusoidalmente
            // Aquí usamos x_des definida en el constructor
            // Para la orientación, usas qt_ para el cuaternión. Asegúrate de que qt_ esté bien definido.
            // Si qt_ no cambia, será una orientación fija.
            Eigen::Quaterniond desired_orientation_quat(qt_[0], qt_[1], qt_[2], qt_[3]);
            desired_orientation_quat.normalize();
            pinocchio::SE3 desired_pose_eigen(desired_orientation_quat.toRotationMatrix(), x_des);

            // 2. Define las ganancias Kp y Kd (6x6 matrices, como las de tu Python)
            // Los valores son solo ejemplos, DEBEN SER SINTONIZADOS para tu robot
            Eigen::Matrix<double,6,6> Kp_impedancia_py = Eigen::Matrix<double,6,6>::Identity();
            // Ajusta estos valores según la respuesta deseada (Kd en Python)
            Kp_impedancia_py.diagonal() << 100.0, 100.0, 100.0, 50.0, 50.0, 50.0;

            Eigen::Matrix<double,6,6> Kd_impedancia_py = Eigen::Matrix<double,6,6>::Identity();
            // Ajusta estos valores según la respuesta deseada (Bd en Python)
            Kd_impedancia_py.diagonal() << 10.0, 10.0, 10.0, 5.0, 5.0, 5.0;

            // 3. Llama a la nueva función
            // El dt de tu python es 0.01, que coincide con tu tiempo de control
            double control_dt = 0.01;

            // Asegúrate de que q_ y qd_ están actualizados desde el suscriptor antes de llamar a esto.
            // Si `last_joint_state_` no se ha recibido todavía, q_ y qd_ estarán en sus valores iniciales.
            if (!last_joint_state_) {
                RCLCPP_WARN(this->get_logger(), "No se han recibido joint_states. Usando posiciones/velocidades iniciales.");
                // Podrías añadir un 'return' o esperar hasta tener datos.
                // Para este ejemplo, asumiremos que se reciben rápidamente.
            }

            Eigen::VectorXd tau = impedanceControlPythonStyle(
                *model, *data, tool_frame_id, q_, qd_,
                desired_pose_eigen, // pinocchio::SE3 xdes
                dx_des,             // Eigen::VectorXd dxdes
                ddx_des,            // Eigen::VectorXd ddxdes
                Kp_impedancia_py,   // Kp (Kd en Python)
                Kd_impedancia_py,   // Kd (Bd en Python)
                control_dt,
                J_anterior          // Se pasará por referencia para actualizar J_anterior dentro de la función
                                    // y usarlo en el siguiente ciclo.
            );

            // Para simular el comportamiento del robot con estos torques,
            // necesitamos la dinámica inversa para obtener la aceleración articular:
            // tau = M * qdd + C + G
            // qdd = M_inv * (tau - C - G)
            // Sin embargo, tu Python simplemente calcula el torque y lo envía.
            // Si quieres actualizar q_ y qd_ localmente para predecir el siguiente estado,
            // puedes usar aba (forward dynamics) o rnea (inverse dynamics) con los torques.

            // Opción 1: Usa la ley de impedancia para generar torques y envíalos (como en Python)
            // Aquí, `tau` es el torque calculado.
            // Si tu control real se basa en `tau`, lo envías directamente al controlador de torques.
            // Sin embargo, tu `joint_trajectory_controller` espera posiciones, no torques.
            // Este es un punto de discrepancia importante.

            // Si tu objetivo es controlar por POSICIONES/VELOCIDADES articulares, entonces necesitas
            // usar el `tau` calculado para determinar `qdd`, y luego integrar `qdd` a `qd` y `q`.
            // Esto es lo que estabas haciendo antes:

            pinocchio::crba(*model, *data, q_); // Asegura que la matriz M de Pinocchio esté actualizada
            Eigen::MatrixXd M_current = (*data).M;
            Eigen::MatrixXd M_inv_current;
            if (M_current.determinant() == 0) {
                RCLCPP_WARN(this->get_logger(), "Matriz de inercia singular al final del ciclo. Usando pseudo-inversa.");
                M_inv_current = M_current.completeOrthogonalDecomposition().pseudoInverse();
            } else {
                M_inv_current = M_current.inverse();
            }

            // Calcula los términos no lineales C y G para el cálculo de qdd
            // Necesarios para qdd = M_inv * (tau - (C+G))
            pinocchio::computeCoriolisMatrix(*model, *data, q_, qd_);
            pinocchio::computeGeneralizedGravity(*model, *data, q_);
            Eigen::VectorXd C_plus_G = (*data).nle; // nle = Coriolis + Gravity

            // Calcula la aceleración articular deseada
            Eigen::VectorXd qdd_calculated = M_inv_current * (tau - C_plus_G);

            // Integrar para obtener nuevas velocidades y posiciones articulares
            qd_ += qdd_calculated * control_dt;
            q_ += qd_ * control_dt;
            for (int i = 0; i < 6; ++i) {
                if (q_[i] > 2*M_PI) {
                    q_[i] = 2 * M_PI; // Normaliza a [-pi, pi]
                } else if (q_[i] < -2*M_PI) {
                    q_[i] = -2 * M_PI; // Normaliza a [-pi, pi]
                }
            }

            // ... (el resto del código para publicar la trayectoria con las nuevas q_)

            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            RCLCPP_INFO(this->get_logger(), "Enviando posición: %.4f %.4f %.4f %.4f %.4f %.4f",
                        q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
            point.positions = {q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]};
            point.time_from_start = rclcpp::Duration::from_seconds(control_dt); // El tiempo de control
            trajectory_msg.points.push_back(point);
            joint_trajectory_pub_->publish(trajectory_msg);

            // J_anterior se actualiza dentro de impedanceControlPythonStyle, no aquí
            qd_anterior = qd_; // Actualizar qd_anterior para el cálculo de qdd_ en el próximo ciclo
            
            time_elapsed_ += control_dt; // Incrementa el tiempo transcurrido
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


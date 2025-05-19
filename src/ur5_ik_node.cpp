#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <algorithm> // Para std::clamp
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include <memory>
#include <stdexcept>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fstream> 
#include <ament_index_cpp/get_package_share_directory.hpp>


#define PI 3.14159265358979323846
#define MAX_JOINT_DELTA 0.1

using namespace std;

// variables globales

bool orientacion = false;
bool implementacion = false;
bool geomagic = false;
std::string control_topic = "";


template <typename T, size_t N>
constexpr size_t array_length(const T (&)[N]) {
    return N;
}

std::string get_file_path(const std::string& package_name, const std::string& relative_path) {
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        return package_share_directory + "/" + relative_path;
    } catch (const std::exception& e) {
        throw std::runtime_error("No se pudo encontrar el paquete: " + package_name);
    }
}


void load_values_from_file(const std::string &file_path, double values[], int size, int line_number) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("No se pudo abrir el archivo: " + file_path);
    }

    std::string line;
    int current_line = 0;
    while (std::getline(file, line)) {
        current_line++;
        if (current_line == line_number) {
            std::istringstream iss(line);
            for (int i = 0; i < size; ++i) {
                if (!(iss >> values[i])) {
                    throw std::runtime_error("Error al leer los valores desde la línea " + std::to_string(line_number));
                }
                if (iss.peek() == ',' || iss.peek() == ' ') {
                    iss.ignore(); // Ignorar comas o espacios
                }
            }
            break;
        }
    }

    if (current_line < line_number) {
        throw std::runtime_error("El archivo no contiene suficientes líneas.");
    }

    file.close();
}

Eigen::Vector3d extractEulerAngles(const Eigen::Quaterniond& quaternion) {
    // Convertir el cuaternión en una matriz de rotación
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

    // Extraer los ángulos de Euler (roll, pitch, yaw) en radianes
    double roll = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2)); // Rotación alrededor de X
    double pitch = asin(-rotation_matrix(2, 0));                      // Rotación alrededor de Y
    double yaw = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0)); // Rotación alrededor de Z

    return Eigen::Vector3d(roll, pitch, yaw);
}
// Rotación alrededor del eje X
Eigen::Matrix3d Rotx(double ang) {
    Eigen::Matrix3d T;
    T << 1, 0, 0,
         0, std::cos(ang), -std::sin(ang),
         0, std::sin(ang),  std::cos(ang);
    return T;
}

// Rotación alrededor del eje Y
Eigen::Matrix3d Roty(double ang) {
    Eigen::Matrix3d T;
    T << std::cos(ang), 0, std::sin(ang),
         0, 1, 0,
        -std::sin(ang), 0, std::cos(ang);
    return T;
}

// Rotación alrededor del eje Z
Eigen::Matrix3d Rotz(double ang) {
    Eigen::Matrix3d T;
    T << std::cos(ang), -std::sin(ang), 0,
         std::sin(ang),  std::cos(ang), 0,
         0, 0, 1;
    return T;
}

// initializeUR5: inicializa el modelo UR5 a partir de un archivo URDF con pinocchio
// model: puntero al modelo de pinocchio
// data: puntero a los datos de pinocchio
// tool_frame_id: ID del marco de la herramienta
// urdf_path: ruta al archivo URDF
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

//computar el error combinado (posicion y orientacion) entre la posicion deseada y la posicion actual en SE3
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

Eigen::Vector3d forwardKinematics(const pinocchio::Model& model, pinocchio::Data& data, 
                                  const pinocchio::FrameIndex& tool_frame_id, const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, tool_frame_id);
    return data.oMf[tool_frame_id].translation();
}

Eigen::MatrixXd jacobian(const pinocchio::Model& model, pinocchio::Data& data, 
                         const pinocchio::FrameIndex& tool_frame_id, const Eigen::VectorXd& q) {
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, tool_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
}

Eigen::VectorXd computeQP_IK(const Eigen::MatrixXd& J, const Eigen::VectorXd& x_error) {
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    int n = J.cols();
    int m = J.rows();

    Eigen::SparseMatrix<double> H = Eigen::MatrixXd::Identity(n, n).sparseView();
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n);
    Eigen::SparseMatrix<double> A = J.sparseView();
    Eigen::VectorXd lowerBound = x_error;
    Eigen::VectorXd upperBound = x_error;

    solver.data()->setNumberOfVariables(n);
    solver.data()->setNumberOfConstraints(m);
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    if (!solver.initSolver()) {
        throw std::runtime_error("Failed to initialize QP solver");
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("Failed to solve QP problem");
    }

    return solver.getSolution();
}

Eigen::VectorXd computeQP_IK_WeightedError(const Eigen::MatrixXd& J, const Eigen::VectorXd& x_error,
                                         const Eigen::MatrixXd& W_p, const Eigen::MatrixXd& W_o) {
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    int n = J.cols();
    //int m = J.rows();

    Eigen::MatrixXd J_p = J.block(0, 0, 3, n);
    Eigen::MatrixXd J_o = J.block(3, 0, 3, n);
    Eigen::VectorXd e_p = x_error.head(3);
    Eigen::VectorXd e_o = x_error.tail(3);

    Eigen::SparseMatrix<double> H = Eigen::MatrixXd::Identity(n, n).sparseView();
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n);

    // Formular el problema como min ||W_p * J_p * q_dot - W_p * e_p||^2 + ||W_o * J_o * q_dot - W_o * e_o||^2
    // Esto lleva a un problema de mínimos cuadrados que se puede resolver con QP.
    Eigen::MatrixXd A = (W_p * J_p).transpose() * (W_p * J_p) + (W_o * J_o).transpose() * (W_o * J_o);
    Eigen::VectorXd b = (W_p * J_p).transpose() * (W_p * e_p) + (W_o * J_o).transpose() * (W_o * e_o);
    // A es la matriz Hessiana y b es el vector de gradiente
    Eigen::SparseMatrix<double> H_qp = A.sparseView();
    Eigen::VectorXd g_qp = -b; // El solver de QP minimiza 0.5 * x^T * H * x + g^T * x 
    // x es la variable de decisión (q_dot)
    solver.data()->setNumberOfVariables(n);
    solver.data()->setNumberOfConstraints(0); // Sin restricciones de igualdad por ahora
    solver.data()->setHessianMatrix(H_qp);
    solver.data()->setGradient(g_qp);

    if (!solver.initSolver()) {
        throw std::runtime_error("Failed to initialize QP solver");
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("Failed to solve QP problem");
    }

    return solver.getSolution();
}

Eigen::VectorXd inverseKinematicsQP(const pinocchio::Model& model, pinocchio::Data& data, const pinocchio::FrameIndex& tool_frame_id,    const Eigen::VectorXd& q_init, 
                                    const pinocchio::SE3& desired_pose,     int max_iters,  double alpha) {

    Eigen::VectorXd q = q_init;
    const double joint_limit = PI; // Joint limit for UR5

    for (int i = 0; i < max_iters; ++i) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacement(model, data, tool_frame_id);
        Eigen::VectorXd x_error = computeError(data, tool_frame_id, desired_pose);

        //cout<<"error:\n"<<x_error.norm()<<endl;
        if (x_error.norm() < 0.0002) {
            
            std::cout << "Converged after " << i << " iterations" << std::endl;
            break;
        }
        if (i == max_iters - 1) {
            std::cout << "Max iterations reached" << std::endl;
        }
        

        Eigen::MatrixXd J = jacobian(model, data, tool_frame_id, q);
        double condition_number = J.jacobiSvd().singularValues()(0) / J.jacobiSvd().singularValues().tail(1)(0);
        if (condition_number > 1e6) {
            std::cerr << "El Jacobiano está mal condicionado." << std::endl;
            return q_init;
        }
        //else {   std::cout << "El Jacobiano está bien condicionado." << std::endl;     }
        double w_p = 1; // Peso para la posición
        double w_o = 0.9; // Peso para la orientación
        Eigen::MatrixXd W_p = Eigen::Matrix3d::Identity() * w_p;
        Eigen::MatrixXd W_o = Eigen::Matrix3d::Identity() * w_o;
        Eigen::VectorXd q_dot = computeQP_IK_WeightedError(J, x_error, W_p, W_o);
        //Eigen::VectorXd q_dot = computeQP_IK(J, x_error);
        q += alpha * q_dot;
        for (int j = 1; j < q.size()-1; j++) {
            if (q[j] > joint_limit) {q[j] = joint_limit;}
            else if (q[j] < -joint_limit) {q[j] = -joint_limit;} 
        }
    }
    return q;
}


Eigen::VectorXd Cinematica_Inversa(double q_init[],  double desired_pose[], 
                                   double desired_quat[],  int max_iteraciones,    double alpha,
                                   std::unique_ptr<pinocchio::Model>& model,  std::unique_ptr<pinocchio::Data>& data,
                                   pinocchio::FrameIndex& tool_frame_id) {


    //initializeUR5(model, data, tool_frame_id, path_urdf);

    Eigen::VectorXd q_init_eigen(6);
    for (int i = 0; i < 6; ++i) {
        q_init_eigen(i) = q_init[i];
    }
    //Eigen::Matrix3d orientation = Rotx(desired_quat[0])*Roty(desired_quat[1])*Rotz(desired_quat[2]);//Eigen::Matrix3d::Identity();
   
    //desired_quat = {0.54, -0.84, 0, 0.04}; donde w 
    Eigen::Quaterniond desired_orientation(desired_quat[0], desired_quat[1], desired_quat[2], desired_quat[3]);//(,,,w)
    desired_orientation.normalize(); // Asegúrate de que el cuaternión esté normalizado
    pinocchio::SE3 desired_pose_eigen(desired_orientation.toRotationMatrix(), Eigen::Vector3d(desired_pose[0], desired_pose[1], desired_pose[2]));
    //pinocchio::SE3 desired_pose_eigen(orientation, Eigen::Vector3d(desired_pose[0], desired_pose[1], desired_pose[2]));
    cout<<"desired_pose_eigen: \n"<<desired_pose_eigen<<endl;

    auto start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd q_solution = inverseKinematicsQP(*model, *data, tool_frame_id, q_init_eigen, desired_pose_eigen, max_iteraciones , alpha);
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Tiempo total: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << " ns" << std::endl;
    std::cout << "Solución de IK: " << q_solution.transpose() << std::endl;
    pinocchio::forwardKinematics(*model, *data, q_solution);
    pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
    const pinocchio::SE3& result_pose = data->oMf[tool_frame_id];
    std::cout << "Posición resultante: " << result_pose.translation().transpose() << std::endl;
    std::cout << "Orientación resultante:\n" << result_pose.rotation() << std::endl;
    
    
    return q_solution;

}

void limit_joint_changes(const double current_positions[6], Eigen::VectorXd& new_positions) {
    for (int i = 0; i < 6; ++i) {
        double delta = new_positions[i] - current_positions[i];
        if (std::abs(delta) > MAX_JOINT_DELTA) {
            new_positions[i] = current_positions[i] + std::copysign(MAX_JOINT_DELTA, delta);
        }
    }
}



class UR5eJointController : public rclcpp::Node {
    public:
        UR5eJointController() : Node("ur5e_joint_controller"), time_elapsed_(0.0) {
            output_file_.open(geo_pos, std::ios::out | std::ios::app);
            output_file_2.open(control_pos, std::ios::out | std::ios::app);
            output_file_3.open(ur5_pos, std::ios::out | std::ios::app);

            initializeUR5(model, data, tool_frame_id, urdf_path);
            
            
            try {
                load_values_from_file(config_path, q_init, 6, 7);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            
            if (!output_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo para guardar los datos.");
            }
            // Publicador para enviar trayectorias
            joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(control_topic, 10);
    
            // Suscriptor para leer el estado articular actual
            subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&UR5eJointController::update_joint_positions, this, std::placeholders::_1));            
                // Suscriptor para leer la posición cartesiana del haptic phantom
            subscription_haptic_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/phantom/pose", 10, std::bind(&UR5eJointController::pose_callback, this, std::placeholders::_1));
            subscription_phantom_joints_ = this->create_subscription<sensor_msgs::msg::JointState>("/phantom/joint_states", 10, std::bind(&UR5eJointController::phantom_joint_states_callback, this, std::placeholders::_1));
                
            // Temporizador para calcular y publicar nuevas posiciones articulares
            timer2_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&UR5eJointController::posicion_inicial, this));
            timer_ = this->create_wall_timer( std::chrono::milliseconds(control_loop_time), std::bind(&UR5eJointController::control_loop, this));
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
        Eigen::Quaterniond q_x;        Eigen::Quaterniond q_y;        Eigen::Quaterniond q_z;        Eigen::Quaterniond quat_initial; 
        double q_[6] ;
        double h_[6] ;
        double q_init[6];
        double x_init[3]; 
        double x_des[3];
        double qt_init[4];
        int control_loop_time = 1; 
        int ur5_time = 0.01;
        double max_iteraciones[1];
        double alpha[1];
        
        sensor_msgs::msg::JointState::SharedPtr last_joint_state_;    
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_haptic_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_phantom_joints_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;
        std::unique_ptr<pinocchio::Model> model; // declarar puntero único para el modelo
        std::unique_ptr<pinocchio::Data> data; // declarar puntero único para los datos
        pinocchio::FrameIndex tool_frame_id; 

        std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
        std::string urdf_path = get_file_path("ur5_simulation",   "include/ur5.urdf");

        std::string ur5_pos = get_file_path("ur5_simulation",     "launch/output_data3.txt");
        std::string control_pos = get_file_path("ur5_simulation",     "launch/output_data2.txt");
        std::string geo_pos = get_file_path("ur5_simulation",     "launch/output_data.txt");
        
        
        
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if (!msg) {
                RCLCPP_ERROR(this->get_logger(), "Mensaje nulo recibido en /phantom/pose.");
                return;
            }
            
            r_[0] = msg->pose.position.x;      r_[1] = msg->pose.position.y;       r_[2] = msg->pose.position.z;
            //cout<<"Posicion: "<<r_[0]<<" "<<r_[1]<<" "<<r_[2]<<endl;
            

            //RCLCPP_INFO(this->get_logger(), "Pose received:");
   
            Eigen::Quaterniond quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
            Eigen::Quaterniond quaternion2(0.977,-0.21,0,0); //quaternion inicial
            quaternion.normalize();
            qt_[1] = quaternion.x();
            qt_[2] = quaternion.y();
            qt_[3] = quaternion.z();
            qt_[0] = quaternion.w();
            

            
            rotation_matrix = quaternion.toRotationMatrix(); 
            //Eigen::Vector3d euler_angles = extractEulerAngles(quaternion*quaternion2.inverse());
            //RCLCPP_INFO(this->get_logger(), "  Roll : %f", euler_angles[0]);
                        
        }    
            

        // POSICIONES ARTICULARES DEL UR5
        void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
            last_joint_state_ = msg;
            if (implementacion){
                q_[0] = msg->position[5];
                q_[1] = msg->position[0];
                q_[2] = msg->position[1];
                q_[3] = msg->position[2];
                q_[4] = msg->position[3];
                q_[5] = msg->position[4];
            }
            else{
                for (int i = 0; i < 6; ++i) {
                    q_[i] = msg->position[i];
                }
            }
        }
        
        void phantom_joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Ejemplo: imprimir las posiciones articulares del haptic
            //RCLCPP_INFO(this->get_logger(), "Phantom joint states recibidas:");
            
            
            for (int i = 0; i < 6; ++i) {
                    h_[i] = msg->position[i];
                }
            q_x.w() =  std::cos((h_[4] + 3.1416)/ 4); q_x.x() =  std::sin((h_[4]+3.1416) / 4);q_x.y() =  0; q_x.z() =  0; //eje real: 
            //q_y.w() =  std::cos(euler_angles[1] / 4); q_y.x() =  0; q_y.y() =  std::sin(euler_angles[1] / 4); q_y.z() =  0;
            //q_z.w() =  std::cos(euler_angles[2] / 4); q_z.x() =  0; q_z.y() =  0; q_z.z() =  std::sin(euler_angles[2] / 4);
            
        }
    
        void posicion_inicial() {
            
            if (posicion_inicial_alcanzada_) {
                // Si ya se alcanzó la posición inicial, no hacer nada
                return;
            }

            

            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            RCLCPP_INFO(this->get_logger(), "Enviando posición inicial: %.2f %.2f %.2f %.2f %.2f %.2f",
                        q_init[0], q_init[1], q_init[2], q_init[3], q_init[4], q_init[5]);
            point.positions = {q_init[0], q_init[1], q_init[2], q_init[3], q_init[4], q_init[5]};
            point.time_from_start = rclcpp::Duration::from_seconds(1);
            trajectory_msg.points.push_back(point);
            joint_trajectory_pub_->publish(trajectory_msg);

            // Verificar si la posición actual coincide con la posición inicial
            bool posicion_correcta = true;
            for (int i = 0; i < 6; ++i) {
                cout<<"posicion del ur5"<<q_[0]<<" "<<q_[1]<<" "<<q_[2]<<" "<<q_[3]<<" "<<q_[4]<<" "<<q_[5]<<endl;
                cout<<"diferencia articular: "<<q_[i] - q_init[i]<<endl;
                if (std::abs(q_[i] - q_init[i]) > 0.01) { // Tolerancia de 0.01 radianes
                    posicion_correcta = false;
                    break;
                }
            }

            if (posicion_correcta) {
                RCLCPP_INFO(this->get_logger(), "Posición inicial alcanzada.");
                posicion_inicial_alcanzada_ = true;
            }
        }

        
        
        /// @brief Bucle de control que calcula y publica nuevas posiciones articulares.
        void control_loop() {

            if (!posicion_inicial_alcanzada_) {
                // Si no se ha alcanzado la posición inicial, no ejecutar el bucle de control
                return;
            }
            // variables modificables desde config.txt
            try {
                load_values_from_file(config_path, max_iteraciones, 1, 15);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            try {
                load_values_from_file(config_path, alpha, 1, 13);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            
            pinocchio::forwardKinematics(*model, *data, Eigen::Map<Eigen::VectorXd>(q_init, 6));
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
            x_init[0] = data->oMf[tool_frame_id].translation()[0];
            x_init[1] = data->oMf[tool_frame_id].translation()[1];
            x_init[2] = data->oMf[tool_frame_id].translation()[2];
            cout<<"x_init: "<<x_init[0]<<" "<<x_init[1]<<" "<<x_init[2]<<endl;

            qt_init[0] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).w();
            qt_init[1] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).x();
            qt_init[2] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).y();
            qt_init[3] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).z();
            cout<<"qt_init: "<<qt_init[0]<<" "<<qt_init[1]<<" "<<qt_init[2]<<" "<<qt_init[3]<<endl;

            cout<<"x_init: "<<x_init[0]<<" "<<x_init[1]<<" "<<x_init[2]<<endl;
            cout<<"r_init: "<<r_[0]<<" "<<r_[1]<<" "<<r_[2]<<endl;
            time_elapsed_ += 0.01; // Incremento de 100 ms
            if (geomagic) {
                int escala = 1.5;
                // Si se está usando el haptic phantom, actualizar las posiciones
                x_des[0] = -r_[0]*escala + x_init[0]; // -r_[0]*2+0.647514
                x_des[1] = (x_init[1]- (r_[1]-0.0881142)*escala);
                x_des[2] = (r_[2]+0.0655108)*escala + x_init[2];
            }else {
                
                x_des[0] = x_init[0] + 0.1 * cos(2 * PI * 0.5 * time_elapsed_); // X_BASE + AMPLITUDE * cos(2 * PI * FREQUENCY * time_elapsed_)
                x_des[1] = x_init[1] + 0.1 * sin(2 * PI * 0.5 * time_elapsed_);                                           // Y_CONST
                x_des[2] = x_init[2];//.5 + 0.1 * sin(2 * PI * 0.5 * time_elapsed_)                                          // Z_CONST
     
            }
            
            //x_init[0] = -0.1152; x_init[1] = 0.493; x_init[2] =  0.293;
            
            
            

            
            cout<<"x_des: "<<x_des[0]<<" "<<x_des[1]<<" "<<x_des[2]<<endl;
            
            
            
            
            // Coordenadas deseadas (xd)
            
            // guarda la posicion del haptico en el archivo
            if (output_file_.is_open()) {
                output_file_ << "Haptic Position: " << r_[0] << " " << r_[1] << " " << r_[2] << " ";
                output_file_ << "Haptic Orientation (quaternion): " << qt_[0] << " " << qt_[1] << " " << qt_[2] << " " << qt_[3] << "\n";
            }
            // guarda la posicion del ur5 en el archivo
            if (output_file_3.is_open()) {
                output_file_3 << "Joint Positions ur: ";
                for (int i = 0; i < 6; ++i) {
                    output_file_3 << q_[i] << " ";
                }
                output_file_3 << "\n";
            }

            

            //quat_initial.w() = -0.26707; quat_initial.x() = 0.962872; quat_initial.y() = 0.010197; quat_initial.z() = -0.03804;
            quat_initial.w() = qt_init[0]; quat_initial.x() = qt_init[1]; quat_initial.y() = qt_init[2]; quat_initial.z() = qt_init[3];
            Eigen::Quaterniond current_orientation;

            if (orientacion) {
                current_orientation = quat_initial;//*q_x*q_x;// * q_y * q_z;
    
            } else {
                current_orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
            }
            
            double rot_des[4] = {current_orientation.w(), current_orientation.x(), current_orientation.y(), current_orientation.z()};
          

            Eigen::VectorXd q_solution = Cinematica_Inversa(q_, x_des,rot_des, max_iteraciones[0], alpha[0], model, data, tool_frame_id);
            // Verificar si la solución es válida
            if (!q_solution.allFinite()) {
                //
                RCLCPP_ERROR(this->get_logger(), "La solución de IK no es válida.");
                return;
            }
            Eigen::VectorXd q_init_eigen(6);
            for (int i = 0; i < 6; ++i) {
                q_init_eigen(i) = q_[i];
            }
            

            if ((q_init_eigen-q_solution).norm() > 1) {
                RCLCPP_ERROR(this->get_logger(), "La diferencia entre la posición inicial y la solución es demasiado grande.");
                q_solution = q_init_eigen;
                
            }
            if ((q_init_eigen-q_solution).norm() < 0.001) {               
                q_solution = q_init_eigen;
                
            }
            cout<<"q_enviado"<<q_solution[0]<<" "<<q_solution[1]<<" "<<q_solution[2]<<" "<<q_solution[3]<<" "<<q_solution[4]<<" "<<q_solution[5]<<endl;

            //limit_joint_changes(q_, q_solution);
            if (output_file_2.is_open()) {
                output_file_2 << "Joint Positions: ";
                for (int i = 0; i < 6; ++i) {
                    output_file_2 << q_solution[i] << " ";
                }
                output_file_2 << "\n";
            }

            // Publicar las nuevas posiciones articulares
            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            point.positions = {q_solution[0], q_solution[1], q_solution[2], q_solution[3], q_solution[4], q_solution[5]};

            point.time_from_start = rclcpp::Duration::from_seconds(ur5_time); // Tiempo para alcanzar la posición
            trajectory_msg.points.push_back(point);
            joint_trajectory_pub_->publish(trajectory_msg);
        }
    };


class JointTrajectoryPublisher : public rclcpp::Node
    {
    public:
        JointTrajectoryPublisher() : Node("joint_trajectory_publisher")
        {
            publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                control_topic, 10);

            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&JointTrajectoryPublisher::publish_trajectory, this));
        }

    private:
        void publish_trajectory()
        {
            // Leer las posiciones articulares desde el archivo config.txt
            std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
            double joint_positions[6];
            cout<<array_length(joint_positions)<<endl;
            try {
                load_values_from_file(config_path, joint_positions, array_length(joint_positions),7); // Leer la 7ma línea
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Error al leer las posiciones articulares desde el archivo: %s", e.what());
                return;
            }
            cout<<"joint_positions: "<<joint_positions[0]<<" "<<joint_positions[1]<<" "<<joint_positions[2]<<" "<<joint_positions[3]<<" "<<joint_positions[4]<<" "<<joint_positions[5]<<endl;

            // Crear y publicar el mensaje de trayectoria
            auto message = trajectory_msgs::msg::JointTrajectory();
            message.joint_names = {
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {joint_positions[0], joint_positions[1], joint_positions[2],
                            joint_positions[3], joint_positions[4], joint_positions[5]};
            point.time_from_start = rclcpp::Duration::from_seconds(4.0);

            message.points.push_back(point);

            RCLCPP_INFO(this->get_logger(), "Publicando posición articular...");
            publisher_->publish(message);
        }

        
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
        std::string urdf_path = get_file_path("ur5_simulation",   "include/ur5.urdf");
    };
        


int main(int argc, char **argv) {
    int l;int l3;
    cout<<"Simulacion o Implementacion? 1.-Simulacion 2.-Implementacion"<<endl;cin>>l3;
    if (l3==1){
        implementacion = false;
        control_topic = "/joint_trajectory_controller/joint_trajectory";
    }else if (l3 ==2){
        implementacion = true;
        control_topic = "/scaled_joint_trajectory_controller/joint_trajectory";
    }
    
    cout<<"Elija:\n1.-Control con Geomagic\n2.-Trayectoria\n3.-Solo Inversa\n4.-Movimiento Articualar"<<endl;cin>> l;
    if (l == 1) {

        int l2;       
        
        cout<<"Con orientacion? 1.-si 2.-no"<<endl;cin>>l2;
        geomagic = true;
        if (l2 == 1) {      orientacion = true;    cout<<"Con orientacion"<<endl;  } 
        else if (l2 == 2) { orientacion = false;   cout<<"Sin orientacion"<<endl;  } 
        else {                                     cout<<"Opcion no valida"<<endl;    return 0; }
        cout<<"Control con Geomagic"<<endl;
        rclcpp::init(argc, argv);
        auto node = std::make_shared<UR5eJointController>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } else if (l == 2) {
        geomagic = false;
        orientacion = true;
        cout<<"Control de Trayectoria"<<endl;
        rclcpp::init(argc, argv);
        auto node = std::make_shared<UR5eJointController>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } else if (l == 3) {
        cout<<"Solo Inversa"<<endl;
        std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
        std::string urdf_path = get_file_path("ur5_simulation",   "include/ur5.urdf");

        double q_init[6];        
        double desired_pose[3];        
        double rot_des[4];
        double max_iteraciones[1];
        double alpha[1];

        double quat[4] = {0,0,1,0};
        Eigen::Matrix3d rot_matrix = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix(); 
        cout<<"rot_matrix: \n"<<rot_matrix<<endl;

        try {
                load_values_from_file(config_path, max_iteraciones, 1, 15);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            try {
                load_values_from_file(config_path, alpha, 1, 13);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
        
        try {
            load_values_from_file(config_path, q_init, 6, 1);       // Leer la 1ra línea para q_init
            load_values_from_file(config_path, desired_pose, 3, 2); // Leer la 2da línea para desired_pose
            load_values_from_file(config_path, rot_des, 4, 3);      // Leer la 3ra línea para rot_des
        } catch (const std::exception &e) {
            cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;
            return 1;
        }
        cout<<"q_init: \n"<<q_init[0]<<" "<<q_init[1]<<" "<<q_init[2]<<" "<<q_init[3]<<" "<<q_init[4]<<" "<<q_init[5]<<endl;
        cout<<"desired_pose: \n"<<desired_pose[0]<<" "<<desired_pose[1]<<" "<<desired_pose[2]<<endl;
        cout<<"rot_des: \n"<<rot_des[0]<<" "<<rot_des[1]<<" "<<rot_des[2]<<" "<<rot_des[3]<<endl;

        std::unique_ptr<pinocchio::Model> model;    
        std::unique_ptr<pinocchio::Data> data; // declarar puntero único para los datos
        pinocchio::FrameIndex tool_frame_id;
        initializeUR5(model, data, tool_frame_id, urdf_path);
        Eigen::VectorXd q_result = Cinematica_Inversa(q_init, desired_pose,rot_des, max_iteraciones[0], alpha[0], model, data, tool_frame_id);
        cout << "Resultado de la cinemática inversa: " << q_result.transpose() << endl; 
    } else if (l == 4) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
        rclcpp::shutdown();
        return 0;
    }
    
    else {
        cout<<"Opcion no valida"<<endl;
        return 0;
    }
    
} 

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
//dinamica
#include <pinocchio/algorithm/crba.hpp>      // Para la matriz de inercia
#include <pinocchio/algorithm/rnea.hpp>      // Para Coriolis y gravedad
#include <pinocchio/algorithm/aba.hpp>  

#include <OsqpEigen/OsqpEigen.h>
#include <memory>
#include <stdexcept>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
#include <fstream> 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modbus/modbus.h>
#include <unistd.h>
#include <iomanip>
#include <cstdlib>  


using namespace std;

#define PI 3.14159265358979323846
#define MAX_JOINT_DELTA 0.1
// Parámetros del gripper
#define CMD_REG_START 0x03E8
#define STATUS_REG    0x07D0
#define SLAVE_ID      0x09




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





// gripper control

// Muestra bytes en hexadecimal
void printHex(const uint8_t* data, int len, const std::string& label) {
    std::cout << label;
    for (int i = 0; i < len; ++i)
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
    std::cout << std::dec << std::endl;
}

// Calcula CRC16-Modbus
uint16_t crc16_modbus(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < length; pos++) {
        crc ^= data[pos];
        for (int i = 0; i < 8; i++) {
            if ((crc & 0x0001)) {
                crc >>= 1;
                crc ^= 0xA001;
            } else crc >>= 1;
        }
    }
    return crc;
}

// Enviar comando raw con CRC automático
bool sendRawWithCRC(modbus_t* ctx, uint8_t* request, int len_wo_crc) {
    uint16_t crc = crc16_modbus(request, len_wo_crc);
    request[len_wo_crc] = crc & 0xFF;
    request[len_wo_crc + 1] = (crc >> 8) & 0xFF;

    printHex(request, len_wo_crc + 2, "Request: ");
    uint8_t rsp[MODBUS_RTU_MAX_ADU_LENGTH];

    if (modbus_send_raw_request(ctx, request, len_wo_crc + 2) == -1 ||
        modbus_receive_confirmation(ctx, rsp) == -1) {
        std::cerr << "Error en comunicación Modbus\n";
        return false;
    }
    printHex(rsp, rsp[2] + 5, "Respuesta: ");
    return true;
}

// Activa el gripper si no está activado aún
bool activateGripper(modbus_t* ctx) {
    uint16_t status;
    if (modbus_read_registers(ctx, STATUS_REG, 1, &status) != 1)
        return false;

    if ((status & 0x3100) == 0x3100) return true;  // Ya activado

    // RESET
    uint8_t reset[] = { SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    sendRawWithCRC(ctx, reset, sizeof(reset) - 2);
    usleep(500000);

    // ACTIVAR
    uint8_t activate[] = { SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
                           0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    sendRawWithCRC(ctx, activate, sizeof(activate) - 2);

    // Esperar hasta activación
    for (int i = 0; i < 20; ++i) {
        usleep(200000);
        if (modbus_read_registers(ctx, STATUS_REG, 1, &status) == 1 &&
            (status & 0x3100) == 0x3100)
            return true;
    }
    return false;
}

// Controla el gripper (posición: 0-255, fuerza: 0-255)
void moveGripper(modbus_t* ctx, uint8_t position, uint8_t force) {
    if (!activateGripper(ctx)) {
        std::cerr << "No se pudo activar el gripper\n";
        return;
    }

    uint8_t cmd[] = {
        SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
        0x09, 0x00, 0x00,       // rACT=1, rGTO=0, rATR=0
        position,               // rPR
        0xFF,                   // rSP (velocidad máxima)
        force,                  // rFR
        0x00, 0x00              // CRC
    };
    sendRawWithCRC(ctx, cmd, sizeof(cmd) - 2);
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

    // Error de orientación (ángulo-eje)
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
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
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
#ifndef UR5_IMPEDANCE_HPP
#define UR5_IMPEDANCE_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody.hpp>
// #include <pinocchio/multibody/data.hpp>

#include <Eigen/Dense>
#include <string>
#include <memory>

namespace ur5_impedance {

/**
 * @brief Estructura para almacenar la salida del control
 */
struct ControlOutput {
    Eigen::VectorXd q_desired;      ///< Posiciones articulares deseadas [6x1]
    Eigen::VectorXd qd_desired;     ///< Velocidades articulares deseadas [6x1]
    Eigen::VectorXd tau;             ///< Pares de control calculados [6x1]
    Eigen::VectorXd q_ddot_desired;  ///< Aceleraciones articulares deseadas [6x1]
};

/**
 * @brief Clase para control por modo deslizante (Sliding Mode Control) del UR5
 * 
 * Esta clase implementa un controlador SMC en el espacio de trabajo cartesiano
 * para seguimiento de trayectorias del robot UR5.
 */
class UR5Impedance {
public:
    // El constructor carga el modelo del robot.
    explicit UR5Impedance(const std::string& urdf_path);
    
    // El método principal que calcula la siguiente posición articular.
    ControlOutput calculateControlCommand(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& dq,
        const Eigen::Vector3d& desired_pos,
        const Eigen::Matrix3d& desired_orient,
        const Eigen::Vector3d& desired_vel,
        const Eigen::Vector3d& desired_vel_orient,
        const Eigen::Vector3d& desired_acc,
        const Eigen::Vector3d& desired_acc_orient,
        const Eigen::Matrix<double, 6, 1>& Kp_task_diag,
        const Eigen::Matrix<double, 6, 1>& Kd_task_diag,
        double dt
    );

private:
    // Miembros privados para el modelo y los datos de Pinocchio.
    std::unique_ptr<pinocchio::Model> model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex tool_frame_id_;

    // Jacobiano de la iteración anterior para calcular la derivada.
    Eigen::MatrixXd J_previous_;

    // Métodos auxiliares privados.
    Eigen::MatrixXd computeFullJacobian(const Eigen::VectorXd& q);
    Eigen::MatrixXd computeFullJacobianQuaternion(        
        const Eigen::VectorXd& q,
        double delta = 1e-8);
};

}  // namespace ur5_impedance

#endif // UR5_IMPEDANCE_HPP
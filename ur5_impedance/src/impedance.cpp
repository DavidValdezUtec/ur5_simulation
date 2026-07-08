#include "ur5_impedance/impedance.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial.hpp>
#include <stdexcept>
#include <iostream>

#define M_PI 3.14159265358979323846

namespace ur5_impedance {

UR5Impedance::UR5Impedance(const std::string& urdf_path) {
    model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_path, *model_);
    data_ = std::make_unique<pinocchio::Data>(*model_);    
    tool_frame_id_ = model_->getFrameId("tool0");
     
    if (!model_->existFrame("tool0")) {
        throw std::runtime_error("El frame 'tool0' no existe en el modelo URDF.");
    }

    J_previous_ = Eigen::MatrixXd::Zero(6, model_->nv);
    std::cout << "Modelo de impedancia cargado correctamente." << std::endl;
}

/**
ControlOutput UR5Impedance::calculateControlCommand(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const Eigen::Vector3d& desired_pos,
    const Eigen::Matrix3d& desired_orient, //ya es matriz 3x3
    const Eigen::Vector3d& desired_vel,
    const Eigen::Vector3d& desired_vel_orient,
    const Eigen::Vector3d& desired_acc,
    const Eigen::Vector3d& desired_acc_orient,
    const Eigen::Matrix<double, 6, 1>& Kp_task_diag,
    const Eigen::Matrix<double, 6, 1>& Kd_task_diag,
    double dt)
{
    // 1. Cinemática y Jacobiano
    pinocchio::forwardKinematics(*model_, *data_, q, dq);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model_->nv); //6x6 para el UR5
    Eigen::MatrixXd dJ = Eigen::MatrixXd::Zero(6, model_->nv);
    pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_, 
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);

    pinocchio::getFrameJacobianTimeVariation(*model_, *data_, tool_frame_id_,
                                            pinocchio::LOCAL_WORLD_ALIGNED, dJ);
    
    Eigen::VectorXd dx_current_cartesian = J * dq; // (6x6) * (6x1) = 6x1

    Eigen::Vector3d vel_ori_d = desired_vel_orient;//Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_ori_d = desired_acc_orient;//Eigen::Vector3d::Zero();


    Eigen::Matrix<double,6,6> Kp_task, Kd_task;
    Kp_task.setZero();
    Kd_task.setZero();
    Kp_task.diagonal() = Kp_task_diag;
    Kd_task.diagonal() = Kd_task_diag;

    // 2. Calcular errores:
    //pose actual
    const auto& current_pose = data_->oMf[tool_frame_id_];

    //error de posicion:
    Eigen::Vector3d current_pos = current_pose.translation();
    Eigen::Vector3d error_pos = current_pos - desired_pos;


    // Error de orientación en SO(3) usando mapa logarítmico
    Eigen::Matrix3d error_rot = current_pose.rotation() * desired_orient.transpose();
    Eigen::Vector3d angular_error = pinocchio::log3(error_rot);


    Eigen::Matrix<double, 6, 1> error_pose;
    error_pose << error_pos, angular_error;

    // Error de velocidad
    
    Eigen::VectorXd desired_vel_6D(6);
    desired_vel_6D << desired_vel, vel_ori_d;
    
    // Eigen::VectorXd current_vel_6D(6);
    // current_vel_6D << current_vel_6D.head(3), vel_ori_d;

    Eigen::VectorXd error_vel = dx_current_cartesian - desired_vel_6D; // (6x1) - (6x1) = 6x1

    // 3. Dinámica del robot
    pinocchio::computeJointJacobians(*model_, *data_, q);
    pinocchio::crba(*model_, *data_, q); // Calcula Matriz de Inercia M
    pinocchio::computeCoriolisMatrix(*model_, *data_, q, dq);
    pinocchio::computeGeneralizedGravity(*model_, *data_, q);
    
    Eigen::MatrixXd M = data_->M;
    Eigen::VectorXd nle = data_->nle; // Coriolis + Gravedad

    // 4. Ley de control de impedancia
    // Replicando la fórmula: tau = M*J_pinv * (dvel_des - Kp*err_p - Kd*err_v - J*dq) + nle
    // Nota: La fórmula original tiene términos que se cancelan o son redundantes.
    // Una formulación más estándar es: tau = J^T * F + nle, donde F es la fuerza cartesiana deseada.
    
    Eigen::VectorXd desired_acc_6D(6);
    desired_acc_6D << desired_acc, acc_ori_d;

    // Fuerza cartesiana deseada (6D: posición + orientación)
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd tau  = M*J_pinv * (desired_acc_6D - Kp_task * error_pose - Kd_task * error_vel - dJ * dq)+ nle;
    Eigen::VectorXd qdd  = J_pinv * (desired_acc_6D - Kp_task * error_pose - Kd_task * error_vel - dJ * dq);
    J_previous_ = J; // Actualizar para la siguiente iteración


    // 5. Integración para obtener la siguiente posición (esto debería hacerlo el controlador del robot, no la ley de control)
    // La ley de control debe devolver un TORQUE (tau) o una ACELERACIÓN (qdd).
    // Devolver la siguiente posición (q_solution) mezcla la ley de control con la simulación.
    // Es mejor devolver el torque y dejar que el nodo principal lo maneje.
    // Sin embargo, para replicar tu código, calcularemos q_solution.
    
    
    Eigen::VectorXd qd_next = dq + qdd * dt;
    Eigen::VectorXd q_next = q + qd_next * dt;
    for (int i = 0; i < 6; ++i) {
        if (q_next[i] > M_PI) {
            q_next[i] = M_PI; // Normaliza a [-pi, pi]
        } else if (q_next[i] < -M_PI) {
            q_next[i] = -M_PI; // Normaliza a [-pi, pi]
        }
    }
    ControlOutput output;
    output.q_desired =q_next;
    output.tau = tau;
    output.q_ddot_desired = qdd;
    return output;
}

}  // namespace ur5_impedance

**/
ControlOutput UR5Impedance::calculateControlCommand(
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
    double dt)
{
    // 1. Cinemática y Jacobiano
    pinocchio::forwardKinematics(*model_, *data_, q, dq);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);

    Eigen::MatrixXd J(6, model_->nv);
    pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_, 
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);

    Eigen::VectorXd dx_current = J * dq; 

    // 2. Errores de Pose y Velocidad
    const auto& current_pose = data_->oMf[tool_frame_id_];
    Eigen::Vector3d error_pos = current_pose.translation() - desired_pos;
    
    // Error de rotación: R_curr * R_des^T -> Log3 para vector de error
    Eigen::Vector3d angular_error = pinocchio::log3(current_pose.rotation() * desired_orient.transpose());

    Eigen::Matrix<double, 6, 1> error_pose;
    error_pose << error_pos, angular_error;

    Eigen::VectorXd desired_vel_6D(6);
    desired_vel_6D << desired_vel, desired_vel_orient;
    Eigen::VectorXd error_vel = dx_current - desired_vel_6D;

    // 3. Dinámica Articular (Siciliano: B(q), nle)
    pinocchio::crba(*model_, *data_, q); // Calcula M (o B en Siciliano)
    pinocchio::nonLinearEffects(*model_, *data_, q, dq); // Calcula Coriolis + Gravedad (nle)
    
    Eigen::MatrixXd M = data_->M;
    Eigen::VectorXd nle = data_->nle;

    // --- NUEVA LEY DE CONTROL: ESPACIO OPERACIONAL ---

    // 4.1 Inercia Operacional: H_x = (J * M^-1 * J^T)^-1
    // Usamos descomposición para evitar problemas cerca de singularidades
    Eigen::MatrixXd M_inv = M.inverse();
    Eigen::MatrixXd Lambda_inv = J * M_inv * J.transpose();
    
    // Matriz de Inercia en Espacio de Tarea (H_x o Lambda)
    Eigen::MatrixXd H_x = Lambda_inv.completeOrthogonalDecomposition().pseudoInverse();

    // 4.2 Definición de Masa Deseada (Md), Amortiguamiento (Dd) y Rigidez (Kd)
    // En este esquema, Md suele ser H_x para "cancelar" la dinámica, 
    // pero Siciliano permite definir una Md virtual. 
    // Aquí usamos Md = H_x para simplificar a Impedancia con torque computado.
    
    Eigen::VectorXd desired_acc_6D(6);
    desired_acc_6D << desired_acc, desired_acc_orient;

    // 4.3 Cálculo de 'y' (Aceleración de referencia según Siciliano)
    // y = acc_d - Md^-1 * (D_d * error_vel + K_d * error_pose)
    // Nota: Como queremos que u sea torque, calculamos la fuerza operacional F_x
    
    Eigen::VectorXd F_x = H_x * (desired_acc_6D - Kp_task_diag.asDiagonal() * error_pose - Kd_task_diag.asDiagonal() * error_vel);

    // 4.4 Compensación de Coriolis y Gravedad en Espacio Operacional (nle_x)
    // nle_x = J^-T * nle - H_x * dJ * dq
    Eigen::MatrixXd dJ(6, model_->nv);
    pinocchio::getFrameJacobianTimeVariation(*model_, *data_, tool_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
    
    Eigen::MatrixXd J_pinv_T = J.completeOrthogonalDecomposition().pseudoInverse().transpose();
    Eigen::VectorXd nle_x = J_pinv_T * nle - H_x * dJ * dq;

    // 4.5 Ley de Control Final (Torque Articular)
    // u = J^T * (F_x + nle_x)
    Eigen::VectorXd tau = J.transpose() * (F_x + nle_x);

    // --- FIN DE LA LEY DE CONTROL ---

    // 5. Cálculo de aceleración articular para integración (opcional)
    Eigen::VectorXd qdd = M.inverse() * (tau - nle);

    Eigen::VectorXd qd_next = dq + qdd * dt;
    Eigen::VectorXd q_next = q + qd_next * dt;

    ControlOutput output;
    output.q_desired = q_next;
    output.qd_desired = qd_next;
    output.tau = tau;
    output.q_ddot_desired = qdd;
    return output;
}
}
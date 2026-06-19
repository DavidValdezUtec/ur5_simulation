#include "ur5_kinematics/kinematics.hpp"
#include <iostream>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

UR5Kinematics::UR5Kinematics(const std::string& urdf_path) {
    model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_path, *model_);
    data_ = std::make_unique<pinocchio::Data>(*model_);
    tool_frame_id_ = model_->getFrameId("tool0");

    if (!model_->existFrame("tool0")) {
        throw std::runtime_error("El frame 'tool0' no existe en el modelo URDF."); 
    }
    std::cout << "Modelo cinemático cargado correctamente desde " << urdf_path << std::endl;
}

pinocchio::SE3 UR5Kinematics::forwardKinematics(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    return data_->oMf[tool_frame_id_];
}


Eigen::Matrix<double, 6, 1> UR5Kinematics::computePoseError2(const pinocchio::SE3& desired_pose) {
    const pinocchio::SE3 current_pose = data_->oMf[tool_frame_id_];

    // Error de posición
    Eigen::Vector3d position_error = desired_pose.translation() - current_pose.translation();

    // Error de orientación usando transpuesta de matrices y antisimetricas
    Eigen::Matrix3d R_err = desired_pose.rotation() * current_pose.rotation().transpose();
    Eigen::Vector3d angular_error;
    angular_error << R_err(2,1) - R_err(1,2),
                     R_err(0,2) - R_err(2,0),
                     R_err(1,0) - R_err(0,1);
    angular_error *= 0.5; // Pequeña aproximación para ángulos pequeños

    Eigen::VectorXd error(6);
    error << position_error, angular_error;
    return error;

    // // Calcula el error espacial exacto en el frame LOCAL_WORLD_ALIGNED
    // pinocchio::Motion error_motion = pinocchio::log6(desired_pose.inverse() * current_pose);
    
    // // Pinocchio devuelve un objeto Motion, lo pasamos a VectorXd
    // return error_motion.toVector();
}

Eigen::VectorXd UR5Kinematics::solveQPIK_Velocity(
    const Eigen::MatrixXd& J, 
    const Eigen::VectorXd& x_dot_des, 
    const Eigen::VectorXd& current_q,
    double dt) 
{
    static OsqpEigen::Solver solver;
    static bool initialized = false;
    static int last_n = -1;

    const int n = static_cast<int>(J.cols()); // 6 para el UR5

    // 1. Formular la función objetivo: min 0.5 * dq^T * H * dq + g^T * dq
    // H = J^T * J + lambda * I
    Eigen::MatrixXd A_hessian = J.transpose() * J;
    const double lambda = 1e-6; // Regularización de Tikhonov
    A_hessian.noalias() += lambda * Eigen::MatrixXd::Identity(n, n);
    
    Eigen::SparseMatrix<double> H_qp = A_hessian.sparseView();
    // g = -J^T * x_dot_des
    Eigen::VectorXd g_qp = -J.transpose() * x_dot_des;

    // 2. Formular las restricciones (A_qp * dq)
    // lower_bound <= dq <= upper_bound
    Eigen::SparseMatrix<double> A_qp = Eigen::MatrixXd::Identity(n, n).sparseView();
    
    Eigen::VectorXd lower_bound(n);
    Eigen::VectorXd upper_bound(n);
    
    // Límites operativos del UR5
    const double joint_limit = 2.0 * M_PI; // Límite de posición física
    const double dq_max_motor = 0.5;      // Límite de velocidad máxima (rad/s)

    for(int j = 0; j < n; ++j) {
        // Velocidad máxima permitida en este dt para NO exceder el límite físico
        double dq_limit_down = (-joint_limit - current_q(j)) / dt;
        double dq_limit_up   = ( joint_limit - current_q(j)) / dt;

        // El solver respetará el límite más restrictivo entre el motor y la posición
        lower_bound(j) = std::max(-dq_max_motor, dq_limit_down);
        upper_bound(j) = std::min( dq_max_motor, dq_limit_up);
    }

    // 3. Inicializar o actualizar el solver OSQP
    if (!initialized || last_n != n) {
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(n);
        solver.data()->setNumberOfConstraints(n); // Ahora informamos que hay 'n' restricciones
        solver.data()->setHessianMatrix(H_qp);
        solver.data()->setGradient(g_qp);
        solver.data()->setLinearConstraintsMatrix(A_qp);
        solver.data()->setLowerBound(lower_bound);
        solver.data()->setUpperBound(upper_bound);
        
        if (!solver.initSolver()) {
            throw std::runtime_error("Failed to initialize QP solver");
        }
        initialized = true;
        last_n = n;
    } else {
        // En tiempo real, solo actualizamos los valores numéricos, no la estructura
        if (!solver.updateHessianMatrix(H_qp)) throw std::runtime_error("Failed to update Hessian");
        if (!solver.updateGradient(g_qp)) throw std::runtime_error("Failed to update Gradient");
        if (!solver.updateBounds(lower_bound, upper_bound)) throw std::runtime_error("Failed to update Bounds");
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("Failed to solve QP problem");
    }

    // El resultado devuelto es estrictamente una VELOCIDAD (rad/s)
    return solver.getSolution(); 
}

Eigen::VectorXd UR5Kinematics::solveQPIK(
    const Eigen::MatrixXd& J, 
    const Eigen::Matrix<double, 6, 1>& error, 
    const Eigen::VectorXd& current_q,
    const Eigen::MatrixXd& W) 
{
    // Reutilizar una instancia estática de OSQP para minimizar overhead por llamada
    static OsqpEigen::Solver solver;
    static bool initialized = false;
    static int last_n = -1;
    

    const int n = static_cast<int>(J.cols()); // Número de articulaciones (variables)
    const Eigen::MatrixXd J_w = W*J;
    const Eigen::VectorXd e_w = W*error;

    // Formular el problema como min ||W_p * (J_p * dq - e_p)||^2 + ||W_o * (J_o * dq - e_o)||^2 + 
    // Equivalente a: min 0.5 dq^T H dq + g^T dq
    Eigen::MatrixXd A = (W * J_w).transpose() * (W * J_w);
    Eigen::VectorXd g_qp = -(W * J_w).transpose() * (W * e_w);

    // Regularización para asegurar definida positiva, evita riesgos en singularidades o colapsos del jacobiano
    const double lambda = 1e-6;
    A.noalias() += lambda * Eigen::MatrixXd::Identity(n, n);

    Eigen::SparseMatrix<double> H_qp = A.sparseView();

    // 3. Restricciones: Límite de paso iterativo y límites articulares
    Eigen::SparseMatrix<double> A_qp = Eigen::MatrixXd::Identity(n, n).sparseView();
    Eigen::VectorXd lower_bound(n);
    Eigen::VectorXd upper_bound(n);
    
    const double joint_limit = 2.0 * M_PI; // Límite físico del robot
    const double max_step = 0.5;           // Límite de paso por iteración (para linealidad del Jacobiano)

    for(int j = 0; j < n; ++j) {
        // ¿Cuánto margen de giro me queda antes de chocar con el límite físico?
        double limit_down = -joint_limit - current_q(j);
        double limit_up   =  joint_limit - current_q(j);

        // El solver restringe el paso \Delta q al mínimo entre el paso máximo permitido (max_step) 
        // y la distancia restante hasta el límite articular físico.
        lower_bound(j) = std::max(-max_step, limit_down);
        upper_bound(j) = std::min( max_step, limit_up);
    }



    if (!initialized || last_n != n) {
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(n);
        solver.data()->setNumberOfConstraints(n); 
        solver.data()->setHessianMatrix(H_qp);
        solver.data()->setGradient(g_qp);
        solver.data()->setLinearConstraintsMatrix(A_qp);
        solver.data()->setLowerBound(lower_bound);
        solver.data()->setUpperBound(upper_bound);
        
        if (!solver.initSolver()) { throw std::runtime_error("Failed to initialize QP solver"); }
        initialized = true;
        last_n = n;
    } else {
        if (!solver.updateHessianMatrix(H_qp)) {throw std::runtime_error("Failed to update Hessian matrix"); }
        if (!solver.updateGradient(g_qp)) {throw std::runtime_error("Failed to update gradient"); }
        if (!solver.updateLowerBound(lower_bound)) {throw std::runtime_error("Failed to update lower bound"); }
        if (!solver.updateUpperBound(upper_bound)) {throw std::runtime_error("Failed to update upper bound"); }
    }
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("Failed to solve QP problem");
    }

    return solver.getSolution();
}


Eigen::VectorXd UR5Kinematics::computeVelocityControlStep(
    const Eigen::VectorXd& q_real,
    const Eigen::Vector3d& desired_pos,
    const Eigen::Matrix3d& desired_orient,
    double Kp_pos,
    double Kp_orient,
    double dt) 
{
    // 1. Cinemática Directa con el estado actual del robot
    pinocchio::forwardKinematics(*model_, *data_, q_real);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    const pinocchio::SE3 current_pose = data_->oMf[tool_frame_id_];

    // 2. Calcular el error espacial exacto
    // Pinocchio devuelve un vector 6D (3 traslación, 3 rotación) en el frame de alineación mundial
    pinocchio::SE3 desired_pose(desired_orient, desired_pos);
    pinocchio::Motion error_motion = pinocchio::log6(current_pose.inverse() * desired_pose);
    Eigen::VectorXd error = error_motion.toVector(); 

    // Opcional: Condición de parada suave si el dispositivo externo deja de enviar objetivos
    if (error.norm() < 1e-5) {
        return Eigen::VectorXd::Zero(model_->nv);
    }

    // 3. Ley de Control Proporcional (Convertir error geométrico en velocidad cartesiana)
    Eigen::VectorXd x_dot_des(6);
    x_dot_des.head(3) = Kp_pos * error.head(3);
    x_dot_des.tail(3) = Kp_orient * error.tail(3);

    // 4. Calcular el Jacobiano
    pinocchio::Data::Matrix6x J(6, model_->nv);
    J.setZero();
    pinocchio::computeFrameJacobian(*model_, *data_, q_real, tool_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    // 5. Resolver el problema cuadrático para obtener las velocidades articulares (dq en rad/s)
    Eigen::VectorXd dq_command = solveQPIK_Velocity(J, x_dot_des, q_real, dt);
    std::cout << "dq: " << dq_command.transpose() << " norm: " << dq_command.norm() << std::endl;
    return dq_command;
}


Eigen::VectorXd UR5Kinematics::inverseKinematicsQP2(
    const Eigen::VectorXd& q_initial,
    const Eigen::Vector3d& desired_pos,
    const Eigen::Matrix3d& desired_orient,
    int max_iterations,
    double alpha,
    double weight_pos,
    double weight_orient)
{
    // Versión simplificada: única tarea cartesiana con OSQP, sin nivel secundario
    Eigen::VectorXd q = q_initial;
    Eigen::VectorXd dq_externo = Eigen::VectorXd::Zero(q.size());

    pinocchio::SE3 desired_pose(desired_orient, desired_pos);
    const int iter_cap = std::min(max_iterations, 150);

    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(6, 6);
    W.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * weight_pos;
    W.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity() * weight_orient;

    for (int i = 0; i < iter_cap; ++i) {
        pinocchio::forwardKinematics(*model_, *data_, q);
        pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);

        const Eigen::Matrix<double, 6, 1> error = computePoseError2(desired_pose);
        if (error.norm() < 1e-4) {
            std::cout << "dq: " << dq_externo.transpose() << " norm: " << dq_externo.norm() << std::endl;
            std::cout << "Convergencia alcanzada en " << i << " iteraciones." << std::endl;
            return q;
        }

        pinocchio::Data::Matrix6x J(6, model_->nv);  J.setZero();
        pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);


        Eigen::VectorXd dq = solveQPIK(J, error, q, W);
        const double nrm = dq.norm();
        dq_externo = dq; // Para logging o análisis externo
        q.noalias() += dq;//alpha_eff * dq;
    }
    std::cout << "dq_limite: " << dq_externo.transpose() << " norm: " << dq_externo.norm() << std::endl;
    return q;
}

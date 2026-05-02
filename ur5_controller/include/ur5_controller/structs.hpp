#ifndef UR5_CONTROLLER__STRUCTS_HPP_
#define UR5_CONTROLLER__STRUCTS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <memory>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace ur5_controller
{

// Estado del robot (articular)
struct RobotState
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);      // Posiciones articulares actuales
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(6);     // Velocidades articulares actuales
  Eigen::VectorXd q_init = Eigen::VectorXd::Zero(6); // Posiciones articulares iniciales
  Eigen::VectorXd q_solution = Eigen::VectorXd::Zero(6); // Solución del controlador
  Eigen::VectorXd u_control = Eigen::VectorXd::Zero(6); // Esfuerzo de control calculado (si se usa control de torque)
};

// Estado Cartesiano del efector final
struct CartesianState
{
  // estado actual
  Eigen::Vector3d position = Eigen::Vector3d::Zero();      // Posición cartesiana actual
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // Orientación cartesiana actual
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity(); // Matriz de rotación actual
  
  // estado deseado
  Eigen::Vector3d position_desired = Eigen::Vector3d::Zero(); // Posición cartesiana deseada
  Eigen::Quaterniond orientation_desired = Eigen::Quaterniond::Identity(); // Orientación cartesiana deseada
  Eigen::Matrix3d rotation_matrix_desired = Eigen::Matrix3d::Identity(); // Matriz de rotación deseada
  
  //Estado Inicial
  Eigen::Vector3d position_initial = Eigen::Vector3d::Zero(); // Posición cartesiana inicial
  Eigen::Quaterniond orientation_initial = Eigen::Quaterniond::Identity(); // Orientación inicial
  Eigen::Matrix3d rotation_matrix_initial = Eigen::Matrix3d::Identity(); // Matriz de rotación inicial

  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();      // Velocidad cartesiana actual
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero(); // Velocidad angular actual quaternionica
  Eigen::Vector3d position_last = Eigen::Vector3d::Zero(); // Posición cartesiana en el último paso
  Eigen::Quaterniond orientation_last = Eigen::Quaterniond::Identity(); // Orientación en
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();  // Aceleración cartesiana actual
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero(); // Aceleración angular actual
  
};

// Estado del dispositivo háptico (Geomagic)
struct HapticState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();           // Posición actual
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // Orientación actual quaterniones
  Eigen::Vector3d position_initial = Eigen::Vector3d::Zero();      // Posición inicial capturada
  Eigen::Quaterniond orientation_initial = Eigen::Quaterniond::Identity(); // Orientación inicial capturada
  Eigen::Vector3d position_last = Eigen::Vector3d::Zero(); // Posición cartesiana en el último paso
  Eigen::Quaterniond orientation_last = Eigen::Quaterniond::Identity(); // Orientación en el último paso

  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();      // Velocidad cartesiana actual 
  Eigen::Vector3d velocity_last = Eigen::Vector3d::Zero(); // Velocidad en el último paso
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero(); // Velocidad angular actual x,y,z
  Eigen::Vector3d angular_velocity_last = Eigen::Vector3d::Zero(); // Velocidad angular en el último paso

  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();  // Aceleración cartesiana actual
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero(); // Aceleración angular actual
  
};

// Configuración y parámetros del nodo
struct NodeConfig
{
  // Estados derivados para control teleoperado
  Eigen::Vector3d position_last = Eigen::Vector3d::Zero();         // Última posición (para derivada)
  Eigen::Quaterniond orientation_last = Eigen::Quaterniond::Identity(); // Última orientación
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();              // Velocidad cartesiana estimada
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();      // Velocidad angular estimada (parte imaginaria del delta quaternion / dt)
  // ------------------------------------------------------------------
  std::string control_topic = "/forward_velocity_controller/commands"; // Tópico de control articular
  std::string operation_mode = "teleop";   // "teleop" o "trajectory"
  std::string ur_model = "ur5";            // Modelo: ur5 / ur5e
  std::string nmspace = "";                // Namespace (prefijo de joints y tópicos)
  std::string urdf_path;                    // Ruta absoluta al URDF (se completa en runtime si queda vacío)
  std::string controller="QP";
  bool use_geomagic = false;                // Modo teleoperado háptico

  // ------------------------------------------------------------------
  // Frecuencia y lazo de control
  // ------------------------------------------------------------------
  double ctrl_hz_ = 100.0;         // Hz (referencial)
  double control_loop_time = 0.01;          // Segundos para time_from_start de cada punto

  // ------------------------------------------------------------------
  // Movimiento articular inicial (similar a nodo ur5_pos)
  // ------------------------------------------------------------------
  bool use_ur5_pos_init = true;             // Activar secuencia inicial hacia q_target
  std::vector<double> q_target {1.57, -1.90771733, 1.57, -1.777, -1.57, 0.0}; // Objetivo articular inicial
  double q_target_time = 2.0;               // Tiempo para alcanzar objetivo inicial

  // ------------------------------------------------------------------
  // Trayectoria automática (cuando use_geomagic = false)
  // Parámetros leídos de ROS params: traj_A, traj_wn, traj_c0, traj_mode
  // ------------------------------------------------------------------
  Eigen::Vector3d traj_A {0.3, 0.2, 0.2}; // Amplitudes en metros
  double traj_wn = 0.5;                  // Frecuencia natural
  double traj_c0 = 0.1;                      // Factor de decaimiento
  int traj_mode = 1;                         // Modo (1 sinusoidal, 2 exponencial, etc.)
  std::string trayectoria = "cicloide creciente"; // Descripción textual (legacy)

  // ------------------------------------------------------------------
  // Waypoints de trayectoria personalizada (no implementado aún)
  // ------------------------------------------------------------------
  std::vector<double> trajectory_waypoints;  // Reservado para futuras trayectorias definidas por puntos
  double trajectory_duration = 5.0;          // Duración total de trayectoria de waypoints

  // ------------------------------------------------------------------
  // CSV Logging
  // ------------------------------------------------------------------
  bool csv_enabled = false;                 // Activar log CSV
  std::string csv_path = "";               // Directorio destino (se completa si vacío)
  std::string csv_prefix = "ur5_log";       // Prefijo de nombre de archivo
  Eigen::Matrix<double, 7, 1> Kp = Eigen::Matrix<double, 7, 1>::Constant(100.0); // Ganancias proporcionales para control PD
  Eigen::Matrix<double, 7, 1> Kd = Eigen::Matrix<double, 7, 1>::Constant(10.0);  // Ganancias derivativas para control PD
  Eigen::Matrix<double, 6, 1> lambda = Eigen::Matrix<double, 6, 1>::Constant(0.5); // Ganancias de control (para QP)
  Eigen::Matrix<double, 6, 1> k = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
  Eigen::Matrix<double, 6, 1> k2 = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
  double alpha = 0.01;
  double damping_factor = 0.01;
  double dt = 0.01;
};

// Recursos de Pinocchio
struct PinocchioResources
{
    std::unique_ptr<pinocchio::Model> model;
    std::unique_ptr<pinocchio::Data> data;
    pinocchio::FrameIndex base_frame_id;  // Frame de la base del robot
    pinocchio::FrameIndex tool_frame_id;  // Frame del efector final
};

} // namespace ur5_controller

#endif // UR5_CONTROLLER__STRUCTS_HPP_

/*
1.57294, -1.72583,  2.05493, -1.90003,  -1.5709, -3.13933,
-0.11, 0.4, 0.301
0, 0,1,0


//CONFIGURACION UR5: 
1.509709803, -1.63697, 1.68547, 0.86656, 1.57, 0.0


//CONFIGURACION GEOMAGIG:
0.977166, -0.210257, -0.002090905, -0.00986305
   w           x          y             z  

//CONFIGURACION SOLVER: 
---ALFA---
0.1
---NUMERO DE ITERACIONES MAX
600
/////////////////////////////////////////  Control impedancia   ////////////////////////////////////////
//Tiempo de entre bucle (ms)
20
//K
1500, 1500, 2500, 1500, 1500, 1500
//B 
30,30, 30,10,10,10
// x des
0.55566, 0.42846, 1.0782
// quaternion deseado
0.49603, 0.11866, 0.20079, 0.83639 
    w         x        y       z



//CONTROLADOR    1-imoednacia 2-optmizador
2



ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, joint_names: ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], points: [{positions: [0.0, -1.57, 1.07, 0.80, 1.57, 0.0], velocities: [], accelerations: [], time_from_start: {sec: 2, nanosec: 0}}]}'

ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=192.168.0.101 target_filename:="${HOME}/my_robot_calibration_ur5.yaml"

ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=192.168.10.103 target_filename:="${HOME}/my_robot_calibration_ur5e.yaml"

ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur5   robot_ip:=192.168.0.101   kinematics_params_file:="${HOME}/my_robot_calibration_ur5.yaml"
ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur5e   robot_ip:=192.168.10.103   kinematics_params_file:="${HOME}/my_robot_calibration_ur5e.yaml"


*/
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
#include "functions.hpp"


class UR5eJointController : public rclcpp::Node {
    public:
        UR5eJointController() : Node("ur5e_joint_controller"), time_elapsed_(0.0) {
            output_file_.open(geo_pos, std::ios::out | std::ios::app);
            output_file_2.open(control_pos, std::ios::out | std::ios::app);
            output_file_3.open(ur5_pos, std::ios::out | std::ios::app);

            initializeUR5(model, data, tool_frame_id, urdf_path);
            
            //llama constantes de config
            try {
                load_values_from_file(config_path, q_init, 6, 7);       // Leer la 7ma línea para q_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            try {
                load_values_from_file(config_path, qt_init_geo, 4, 11);       // Leer la 1va línea para qt_init del GeomagicTouch
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
        Eigen::Quaterniond q_x;        Eigen::Quaterniond q_y;        Eigen::Quaterniond q_z;        
        Eigen::Quaterniond quat_initial_UR5; 
        Eigen::Quaterniond quat_initial_geo; 
        Eigen::Quaterniond quat_real_geo; 
        double q_[6] ;
        double h_[6] ;
        double q_init[6];
        double x_init[3]; 
        double x_des[3];
        double qt_init_ur5[4];
        double qt_init_geo[4];
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

   
            quat_real_geo.w() = msg->pose.orientation.w;
            quat_real_geo.x() = msg->pose.orientation.x;
            quat_real_geo.y() = msg->pose.orientation.y;
            quat_real_geo.z() = msg->pose.orientation.z;

            
                        
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

            quat_initial_geo.w() = qt_init_geo[0]; quat_initial_geo.x() = qt_init_geo[1]; quat_initial_geo.y() = qt_init_geo[2]; quat_initial_geo.z() = qt_init_geo[3];
            if (!posicion_inicial_alcanzada_) {
                // Si no se ha alcanzado la posición inicial, no ejecutar el bucle de control
                return;
            }
            // variables modificables desde config.txt
            try {
                load_values_from_file(config_path, max_iteraciones, 1, 18);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            try {
                load_values_from_file(config_path, alpha, 1, 16);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            
            pinocchio::forwardKinematics(*model, *data, Eigen::Map<Eigen::VectorXd>(q_init, 6));
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);

            x_init[0] = data->oMf[tool_frame_id].translation()[0];
            x_init[1] = data->oMf[tool_frame_id].translation()[1];
            x_init[2] = data->oMf[tool_frame_id].translation()[2];
            cout<<"x_init: "<<x_init[0]<<" "<<x_init[1]<<" "<<x_init[2]<<endl;

            qt_init_ur5[0] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).w();
            qt_init_ur5[1] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).x();
            qt_init_ur5[2] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).y();
            qt_init_ur5[3] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).z();
            cout<<"qt_init: "<<qt_init_ur5[0]<<" "<<qt_init_ur5[1]<<" "<<qt_init_ur5[2]<<" "<<qt_init_ur5[3]<<endl;

            Eigen::Quaterniond quat_trans_geo;
            cout<<"quat_real_geo: "<<quat_real_geo.w()<<" "<<quat_real_geo.x()<<" "<<quat_real_geo.y()<<" "<<quat_real_geo.z()<<endl;
            cout<<"quat_initial_geo: "<<quat_initial_geo.w()<<" "<<quat_initial_geo.x()<<" "<<quat_initial_geo.y()<<" "<<quat_initial_geo.z()<<endl;    
            quat_trans_geo = quat_real_geo * quat_initial_geo.inverse();
            cout<<"quat_trans_geo: "<<quat_trans_geo.w()<<" "<<quat_trans_geo.x()<<" "<<quat_trans_geo.y()<<" "<<quat_trans_geo.z()<<endl;
            
            Eigen::AngleAxisd aa(quat_trans_geo);
            double angle = aa.angle();
            Eigen::Vector3d axis = aa.axis();

            // Escalar el ángulo (por ejemplo, a la mitad)
            double escala = 0.5;
            Eigen::AngleAxisd aa_escalado(angle * escala, axis);

            // Reconstruir el cuaternión escalado
            quat_trans_geo = Eigen::Quaterniond(aa_escalado);


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
            quat_initial_UR5.w() = qt_init_ur5[0]; quat_initial_UR5.x() = qt_init_ur5[1]; quat_initial_UR5.y() = qt_init_ur5[2]; quat_initial_UR5.z() = qt_init_ur5[3];
            Eigen::Quaterniond current_orientation;

            if (orientacion) {
                current_orientation = quat_initial_UR5*quat_trans_geo;//*q_x*q_x;// * q_y * q_z;
    
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
    modbus_t* ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
    if (!ctx || modbus_set_slave(ctx, SLAVE_ID) == -1 || modbus_connect(ctx) == -1) {
        std::cerr << "Error al configurar Modbus\n";
    }

    // Cambiar aquí la posición (0-255) y fuerza (0-255)
    sleep(2);
    moveGripper(ctx, 255, 255);  // Mitad cerrado con fuerza alta
    sleep(5);
    moveGripper(ctx, 0, 150);    // Abrir con fuerza media

    modbus_close(ctx);
    modbus_free(ctx);
    // Inicializar ROS2
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
                load_values_from_file(config_path, max_iteraciones, 1, 18);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            try {
                load_values_from_file(config_path, alpha, 1, 16);       // Leer la 6ta línea para x_init del UR5
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


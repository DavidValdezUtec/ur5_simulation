# Estado interno de controller_node en cada ciclo publicado: la misma
# informacion que la fila del CSV (csv_logger.cpp). Se publica en
# /<robot_id>/controller_state (lo grafica ur5_panel, modo Desarrollo).

std_msgs/Header header
string controller_type             # QP / SLD / IMP

# Articular (rad), orden shoulder_pan .. wrist_3
float64[6] q_meas                  # medida (joint_states)
float64[6] q_cmd                   # deseada (solucion del controlador)
float64[6] u_control               # esfuerzo de control

# Cartesiano: punta de la herramienta en el frame base (m / rad)
geometry_msgs/Point x_des
geometry_msgs/Point x_meas
geometry_msgs/Quaternion quat_des
geometry_msgs/Quaternion quat_meas
geometry_msgs/Vector3 euler_des    # roll, pitch, yaw
geometry_msgs/Vector3 euler_meas

# Errores
geometry_msgs/Vector3 pos_err      # x_des - x_meas
float64 e_r_angle          
float64 ori_err_angle              # angulo de quat_des * quat_meas^-1

# Tiempos del lazo (ms)
float64 ik_ms
float64 loop_ms

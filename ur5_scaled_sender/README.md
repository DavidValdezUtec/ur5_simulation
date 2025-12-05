# UR5 Scaled Sender Package

Paquete ROS 2 para teleoperación del robot UR5e con dispositivo háptico Phantom Omni Touch.

## Características

- ✅ Teleoperación en tiempo real con Phantom Omni
- ✅ Control de posición y orientación (6 DOF)
- ✅ Cinemática inversa con Pinocchio
- ✅ Control adaptativo de velocidad
- ✅ Logging detallado en CSV para análisis
- ✅ Soporte para robot real y simulación
- ✅ Detección de saturación de velocidad
- ✅ Mapeo configurable de ejes

## Dependencias

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# Pinocchio (cinemática)
sudo apt install ros-humble-pinocchio

# UR Robot Driver
sudo apt install ros-humble-ur-robot-driver

# Eigen3
sudo apt install libeigen3-dev

# Phantom Omni (haptic package - debe estar en tu workspace)
```

## Instalación

```bash
# Clonar el repositorio
cd ~/ros2_ws/src
git clone https://github.com/DavidValdezUtec/ur5_simulation.git
cd ..

# Instalar dependencias
rosdep install --from-paths src --ignore-src -r -y

# Compilar
colcon build --packages-select ur5_scaled_sender --symlink-install

# Sourcer
source install/setup.bash
```

## Uso

### 1. Robot Real con Teleoperación

```bash
# Terminal 1: Lanzar driver del robot real
ros2 launch ur5_scaled_sender real_robot.launch.py robot_ip:=192.168.10.103

# Terminal 2: Lanzar teleoperación con Phantom Omni
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py
```

### 2. Configuración de Parámetros

```bash
# Ajustar escalas de movimiento
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
    haptic_scale_pos:=0.5 \
    haptic_scale_rot:=0.5

# Cambiar frecuencia de control
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
    ctrl_hz:=125.0

# Ajustar velocidad máxima de juntas
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
    max_joint_vel:=2.5

# Mapeo de ejes (ej: robot rotado 180°)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
    sign_x:=-1.0 \
    sign_y:=-1.0
```

### 3. Visualización en RViz

```bash
ros2 launch ur5_scaled_sender view_both_robots.launch.py
```

## Estructura del Paquete

```
ur5_scaled_sender/
├── src/
│   ├── opt_teleop_haptic.cpp          # Nodo principal de teleoperación
│   ├── joint_trajectory_publisher.cpp  # Publicador de trayectorias
│   └── guia_control_funccional.cpp     # Ejemplo de control
├── launch/
│   ├── opt_teleop_real.launch.py      # Launch para robot real
│   ├── real_robot.launch.py           # Launch del driver UR
│   ├── send_trajectory.launch.py      # Envío de trayectorias
│   └── view_both_robots.launch.py     # Visualización RViz
├── rviz/
│   └── config.rviz                     # Configuración RViz
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Nodos

### opt_teleop_haptic

Nodo principal que realiza teleoperación del UR5 con Phantom Omni.

**Publicadores:**
- `/scaled_joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)

**Suscriptores:**
- `/phantom1/phantom/pose` (omni_msgs/OmniButtonEvent) - Pose del Phantom Omni
- `/joint_states` (sensor_msgs/JointState) - Estado actual del robot

**Parámetros:**
- `urdf_path` (string): Ruta al archivo URDF del robot
- `ctrl_hz` (double): Frecuencia de control en Hz (default: 125.0)
- `haptic_scale_pos` (double): Escala de movimiento posicional (default: 1.0)
- `haptic_scale_rot` (double): Escala de movimiento rotacional (default: 0.5)
- `filter_gain` (double): Ganancia del filtro paso-bajo (default: 0.6)
- `max_joint_vel` (double): Velocidad máxima de juntas en rad/s (default: 2.5)
- `sign_x`, `sign_y`, `sign_z` (double): Inversión de ejes (±1.0)
- `map_x`, `map_y`, `map_z` (int): Remapeo de ejes (0=X, 1=Y, 2=Z)

## Logging y Análisis

El nodo genera archivos CSV con datos de teleoperación en:
```
/tmp/opt_teleop_YYYYMMDD_HHMMSS.csv
```

Contenido del CSV:
- Tiempo
- Posiciones comandadas (q_cmd)
- Posiciones actuales (q_actual)
- Posición cartesiana deseada (p_des)
- Time from start
- Velocidades requeridas por junta
- Velocidad máxima
- Error de seguimiento
- Flag de saturación

## Troubleshooting

### Robot no se mueve

1. Verificar que el teach pendant esté en modo remoto
2. Comprobar conexión de red: `ping 192.168.10.103`
3. Verificar que el controlador esté activo:
   ```bash
   ros2 control list_controllers
   ```

### Movimiento muy lento o interrumpido

- El problema suele ser que cada comando interrumpe al anterior
- El código usa `time_from_start` fijo de 100ms para interpolación suave
- Ajustar frecuencia si es necesario: `ctrl_hz:=125.0`

### Errores de velocidad saturada

- Reducir escalas: `haptic_scale_pos:=0.5`
- Aumentar `max_joint_vel` si es seguro
- Revisar logs CSV para análisis detallado

### Orientación incorrecta

- Verificar mapeo de ejes con parámetros `sign_*` y `map_*`
- El código usa rotación local: `R_des = R_start * R_delta`
- Probar inversión de ejes según orientación de montaje

## Seguridad

⚠️ **IMPORTANTE**:
- Siempre tener el botón de parada de emergencia accesible
- Comenzar con escalas bajas (`haptic_scale_pos:=0.3`)
- Usar teach pendant al 40-100% según experiencia
- Mantener distancia de seguridad del robot
- Verificar workspace limits antes de operar

## Contribución

Este paquete fue desarrollado para investigación en teleoperación robótica.

## Licencia

Apache 2.0

## Contacto

- Repositorio: https://github.com/DavidValdezUtec/ur5_simulation
- Universidad: UTEC

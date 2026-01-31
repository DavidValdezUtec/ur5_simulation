# Simulación y Teleoperación de UR5 con Geomagic Touch

Este repositorio contiene el software necesario para teleoperar dos robots UR5/UR5e utilizando los dispositivos hápticos Geomagic Touch en un entorno ROS 2 Humble.

## Requisitos Previos

*   **Sistema Operativo:** Ubuntu 22.04
*   **Plataforma ROS:** ROS 2 Humble
*   **RAM:** Se recomienda un mínimo de 8 GB de RAM para la compilación.

## Guía de Instalación

Sigue estos pasos en orden para configurar tu entorno de desarrollo.

### 1. Instalación de ROS 2 Humble

Estos comandos instalarán la distribución de escritorio de ROS 2 Humble y las herramientas de desarrollo.

```bash
# Configurar la codificación de caracteres a UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen es_ES es_ES.UTF-8
sudo update-locale LC_ALL=es_ES.UTF-8 LANG=es_ES.UTF-8
export LANG=es_ES.UTF-8

# Habilitar los repositorios 'universe' y 'multiverse'
sudo apt install software-properties-common
sudo add-apt-repository universe

# Añadir la clave GPG y el repositorio de ROS 2
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Actualizar e instalar ROS 2 y herramientas de desarrollo
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop ros-dev-tools

# Añadir el script de configuración al .bashrc para cargarlo automáticamente
echo '
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    # Forzar X11 para evitar errores con Wayland en algunas interfaces gráficas
    export QT_QPA_PLATFORM=xcb
fi' >> ~/.bashrc
```
**Importante:** Cierra y vuelve a abrir tu terminal para que los cambios surtan efecto.

### 2. Instalación de Dependencias Adicionales

Instala todas las dependencias de paquetes de ROS y del sistema con un solo comando.

```bash
sudo apt update
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-ros2-control \
  ros-humble-controller-manager \
  ros-humble-ur \
  ros-humble-ur-simulation-gz\
  ros-humble-pinocchio \
  libeigen3-dev \
  libgoogle-glog-dev \
  libmodbus-dev
```

### 3. Instalación de Dependencias desde Código Fuente

`OSQP` y `Osqp-Eigen` se compilan desde el código fuente para asegurar la compatibilidad.

1.  **Instalar OSQP:**
    ```bash
    git clone --recursive https://github.com/osqp/osqp.git
    cd osqp
    git checkout v0.6.3
    git submodule update --init --recursive
    mkdir build && cd build
    cmake .. && make
    sudo make install
    cd ../..
    ```

2.  **Instalar Osqp-Eigen:**
    ```bash
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen
    git checkout v0.8.1
    mkdir build && cd build
    cmake .. && make
    sudo make install
    cd ../..
    ```
    **Nota:** Actualiza el caché del enlazador después de instalar librerías manualmente.
    ```bash
    sudo ldconfig
    ```

### 4. Compilación del Workspace

Una vez instaladas todas las dependencias, clona este repositorio y los paquetes adicionales en tu workspace de ROS 2 y compila.

```bash
# Crea y navega a tu workspace
mkdir -p ~/tesis_ws/src
cd ~/tesis_ws/src

# Clona los repositorios necesarios
git clone https://github.com/DavidValdezUtec/ur5_simulation.git
git clone https://github.com/stevens-armlab/Geomagic_Touch_ROS2.git
# Aquí iría el 'git clone' para el driver del UR si es necesario

# Regresa a la raíz del workspace y compila
cd ~/tesis_ws
colcon build --symlink-install
```
**Nota sobre la compilación:** Si encuentras un error de tipo `killed` o `Terminado`, significa que te has quedado sin memoria RAM. Intenta compilar de nuevo usando un solo núcleo:
`colcon build --parallel-workers 1`

### 5. Uso

Para ejecutar la simulación, asegúrate de haber "sourceado" tu workspace y luego utiliza el archivo de lanzamiento correspondiente.

```bash
# Carga la configuración de tu workspace (haz esto en cada nueva terminal)
source ~/tesis_ws/install/setup.bash

# Lanza el nodo de simulación (reemplaza con el nombre real de tu launch file)
ros2 launch ur5_simulation ur5_simulation.launch.py
```

Para teleoperar los robots con los Geomagic Touch se debe renombrar los dispositivos como "phantom2" para el brazo izquierdo y "phantom3" para el derecho, luego calibrarlos por separado, desconectarlos y conectarlos de la forma, primero izquierdo y luego derecho. Finalmente, lanzar el nodo de teleoperación:

```bash
ros2 launch omni_common dual_omni_state.launch.py 
```
Para lanzar el driver de los 2 robots conectar ambos a un mismo router, en ambos configurar la IP de urcaps: 192.168.10.101 y en el equipo mantener IP estatica IPV4 con el mismo valor y la mascara 255.255.255.0, las IPs de cada robot son: 192.168.10.103 y 192.168.10.104 luego lanzar el siguiente archivo de launch: 
```bash
ros2 launch ur_robot_driver dual_control.launch.py r1_type:=ur5e r2_type:=ur5e
```
Una vez los robots esten posicionados correctamente se lanza los controladores de la forma:
```bash
ros2 run ur5_controller controller_backup --ros-args -p control_topic:="/scaled_joint_trajectory_controller/joint_trajectory" -p ur:="ur5e" -p nmspace:="r1" -p geomagic_topic:="/phantom3/pose" -p geomagic_button_topic:="/phantom3/button" -p csv_log_enable:="true" -p geomagic:="true"
```

```bash
ros2 run ur5_controller controller_backup --ros-args -p control_topic:="/scaled_joint_trajectory_controller/joint_trajectory" -p ur:="ur5e" -p nmspace:="r2" -p geomagic_topic:="/phantom2/pose" -p geomagic_button_topic:="/phantom2/button" -p csv_log_enable:="true" -p geomagic:="true"
```


Dependencias de python:

#!/bin/bash
set -e  # Salir si hay error

echo "==================================="
echo "  Instalando dependencias Python"
echo "==================================="

# 1. Actualizar pip
echo "[1/5] Actualizando pip..."
python3 -m pip install --upgrade pip

# 2. Instalar paquetes del sistema (recomendado para PyQt5)
echo "[2/5] Instalando paquetes del sistema..."
sudo apt-get update
sudo apt-get install -y \
    python3-pyqt5 \
    python3-pyqt5.qtsvg \
    python3-opencv \
    python3-pip

# 3. Instalar versiones específicas con pip (si necesitas versiones exactas)
echo "[3/5] Instalando paquetes Python específicos..."
# Opción A: Versiones exactas (recomendado para reproducibilidad)
pip3 install \
    PyQt5==5.15.6 \
    PyQt5-sip==12.9.1 \
    opencv-python-headless==4.13.0.90

# Opción B: Rango de versiones (más flexible)
# pip3 install \
#     "PyQt5>=5.15.0,<5.16.0" \
#     "PyQt5-sip>=12.9.0,<13.0.0" \
#     "opencv-python-headless>=4.5.0,<5.0.0"

# Opción C: Últimas versiones compatibles (menos recomendado)
# pip3 install PyQt5 PyQt5-sip opencv-python-headless

# 4. ROS2 Humble ya provee cv-bridge y rclpy, no instalar con pip
echo "[4/5] Verificando paquetes ROS2..."
echo "cv-bridge y rclpy se instalan automáticamente con ROS2 Humble"

# 5. Verificar instalación
echo "[5/5] Verificando instalación..."
python3 -c "import PyQt5; print(f'PyQt5: {PyQt5.QtCore.PYQT_VERSION_STR}')"
python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}')"
python3 -c "import rclpy; print('rclpy: OK')"
python3 -c "from cv_bridge import CvBridge; print('cv_bridge: OK')"

echo "✅ Instalación completada exitosamente"

# ==========================================
# INSTALAR VERSIÓN ESPECÍFICA
# ==========================================
pip3 install PyQt5==5.15.6
pip3 install opencv-python-headless==4.13.0.90

# ==========================================
# BAJAR DE VERSIÓN (DOWNGRADE)
# ==========================================
pip3 install --force-reinstall PyQt5==5.15.6
pip3 install --force-reinstall opencv-python==4.5.4

# ==========================================
# ACTUALIZAR A ÚLTIMA VERSIÓN
# ==========================================
pip3 install --upgrade PyQt5
pip3 install --upgrade opencv-python-headless

# ==========================================
# INSTALAR DESDE requirements.txt
# ==========================================
pip3 install -r requirements.txt

# ==========================================
# DESINSTALAR
# ==========================================
pip3 uninstall PyQt5 PyQt5-sip
pip3 uninstall opencv-python-headless

# ==========================================
# LISTAR PAQUETES INSTALADOS
# ==========================================
pip3 list
pip3 list | grep PyQt
pip3 freeze > requirements.txt  # Guardar versiones actuales

# ==========================================
# VERIFICAR VERSIÓN
# ==========================================
python3 -c "import PyQt5.QtCore; print(PyQt5.QtCore.PYQT_VERSION_STR)"
python3 -c "import cv2; print(cv2.__version__)"
pip3 show PyQt5

# requirements.txt para ur5_panel
PyQt5==5.15.6
PyQt5-sip==12.9.1
opencv-python-headless==4.13.0.90
# cv-bridge y rclpy vienen con ROS2 Humble, NO instalar con pip

# En tu workspace:
# 1. Construir paquetes
colcon build --packages-select ur5_interfaz_library ur5_panel

# 2. Ejecutar install.sh automáticamente (opción)
# Agregar en CMakeLists.txt o package.xml un post-build hook
# O ejecutar manualmente:
./install.sh

# 3. Source el workspace
source install/setup.bash

# 4. Ejecutar
ros2 run ur5_panel prueba2
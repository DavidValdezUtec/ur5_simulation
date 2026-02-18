#!/bin/bash
#
# Script de instalación para el entorno de teleoperación de UR5 con Geomagic Touch.
# Este script automatiza los pasos descritos en el archivo README.md.

# Detener la ejecución si un comando falla
set -e

# Función para imprimir mensajes informativos
info() {
    echo "================================================================================"
    echo "INFO: $1"
    echo "================================================================================"
}

# --- 1. Creación del Workspace y Clonación de Repositorios ---
info "Creando workspace y clonando repositorios..."
mkdir -p ~/tesis_ws/src
cd ~/tesis_ws/src

# Clonar el repositorio principal de la simulación
if [ ! -d "ur5_simulation" ]; then
    git clone https://github.com/DavidValdezUtec/ur5_simulation.git
else
    info "El repositorio ur5_simulation ya existe. Omitiendo clonación."
fi
# (Aquí se podrían clonar otros repositorios si fuera necesario)
cd ~/tesis_ws


# --- 2. Instalación de ROS 2 Humble ---
if [ -f "/opt/ros/humble/setup.bash" ]; then
    info "ROS 2 Humble ya está instalado. Omitiendo instalación."
else
    info "Instalando ROS 2 Humble y herramientas de desarrollo..."

    # Configurar la codificación de caracteres a UTF-8
    sudo apt update && sudo apt install -y locales
sudo locale-gen es_ES es_ES.UTF-8
sudo update-locale LC_ALL=es_ES.UTF-8 LANG=es_ES.UTF-8
export LANG=es_ES.UTF-8

# Habilitar los repositorios 'universe' y 'multiverse'
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# Añadir la clave GPG y el repositorio de ROS 2
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Actualizar e instalar ROS 2 y herramientas de desarrollo
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools

# Añadir el script de configuración al .bashrc para cargarlo automáticamente
# Se verifica si el bloque de configuración ya existe antes de añadirlo
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    info "Añadiendo la configuración de ROS 2 a ~/.bashrc"
    echo '
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    # Forzar X11 para evitar errores con Wayland en algunas interfaces gráficas
    export QT_QPA_PLATFORM=xcb
fi' >> ~/.bashrc
else
    info "La configuración de ROS 2 ya existe en ~/.bashrc. Omitiendo."
fi

# Cargar la configuración de ROS 2 para la sesión actual
source /opt/ros/humble/setup.bash
fi

# --- 3. Instalación de Drivers de Geomagic Touch ---
info "Instalando drivers de Geomagic Touch..."

# Crear un directorio temporal para las descargas
TEMP_DRIVER_DIR=$(mktemp -d)
cd "$TEMP_DRIVER_DIR"

# Descargar e instalar los drivers de Open Haptic
curl -L -o TouchDriver_2024_09_19.tgz https://s3.us-east-1.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2024_09_19.tgz
tar -xzvf TouchDriver_2024_09_19.tgz
cd TouchDriver_2024_09_19
set +e
sudo bash install_haptic_driver
INSTALL_HAPTIC_STATUS=$?
set -e

# En Distrobox, udevadm puede fallar aunque los archivos se copien correctamente.
LIB_PHANTOM_IO=/usr/lib/libPhantomIOLib42.so
LIB_PHANTOM_MANAGER=/usr/lib/libPhantomManagerLite.so
UDEV_RULES_DIR=/etc/udev/rules.d
DRIVER_RULES_DIR="$TEMP_DRIVER_DIR/TouchDriver_2024_09_19/rules.d"
RULES_REQUIRED=0
RULES_INSTALLED=0

if [[ -d "$DRIVER_RULES_DIR" ]]; then
    for rule_path in "$DRIVER_RULES_DIR"/*.rules; do
        [[ -e "$rule_path" ]] || continue
        RULES_REQUIRED=$((RULES_REQUIRED + 1))
        rule_name=$(basename "$rule_path")
        if [[ -f "$UDEV_RULES_DIR/$rule_name" ]]; then
            RULES_INSTALLED=$((RULES_INSTALLED + 1))
        fi
    done
fi

if [[ -f "$LIB_PHANTOM_IO" && -f "$LIB_PHANTOM_MANAGER" ]] && [[ $RULES_REQUIRED -eq 0 || $RULES_INSTALLED -eq $RULES_REQUIRED ]]; then
    info "Drivers detectados en el sistema. Continuando la instalación."
else
    if [[ $INSTALL_HAPTIC_STATUS -ne 0 ]]; then
        echo "ERROR: El instalador de Geomagic Touch falló y no se encontraron los archivos esperados."
        exit 1
    fi
    echo "ERROR: No se encontraron los archivos esperados tras instalar los drivers."
    exit 1
fi
cd ..

# Descargar e instalar OpenHaptics Developer Edition
curl -L -o openhaptics_developer.tar.gz "https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz"
tar -xzvf openhaptics_developer.tar.gz
cd openhaptics_3.4-0-developer-edition-amd64
echo "ATENCIÓN: La siguiente instalación es interactiva."
echo "Deberás presionar 'y' para continuar y 'q' para cancelar el reinicio al final."
read -p "Presiona Enter para iniciar la instalación de OpenHaptics..."
sudo ./install
cd ..

# Copiar los binarios de calibración a una ruta local del usuario
info "Copiando binarios de calibración a ~/.local/share/geomagic/bin..."
LOCAL_TOUCH_BIN_DIR="$HOME/.local/share/geomagic/bin"
DRIVER_BIN_DIR="$TEMP_DRIVER_DIR/TouchDriver_2024_09_19/bin"
mkdir -p "$LOCAL_TOUCH_BIN_DIR"

if compgen -G "$DRIVER_BIN_DIR/*" > /dev/null; then
    install -m 0755 "$DRIVER_BIN_DIR"/* "$LOCAL_TOUCH_BIN_DIR/"
else
    echo "ERROR: No se encontraron binarios en $DRIVER_BIN_DIR"
    exit 1
fi


# Limpiar los archivos descargados
info "Limpiando archivos de instalación de drivers..."
cd ~
rm -rf "$TEMP_DRIVER_DIR"

# Instalar paquetes para los programas gráficos del driver
info "Instalando dependencias gráficas para los drivers..."
sudo apt update
sudo apt install -y build-essential libncurses5-dev freeglut3-dev zlib1g-dev libncurses5

# --- 4. Instalación de Dependencias Adicionales de ROS ---
info "Instalando dependencias adicionales de ROS..."
sudo apt update
sudo apt install -y \
  ros-humble-v4l2-camera \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-ros2-control \
  ros-humble-controller-manager \
  ros-humble-ur \
  ros-humble-ur-simulation-gz \
  ros-humble-pinocchio \
  libeigen3-dev \
  libgoogle-glog-dev \
  libmodbus-dev

# --- 5. Instalación de Dependencias desde Código Fuente (OSQP y Osqp-Eigen) ---
info "Compilando e instalando OSQP y Osqp-Eigen desde el código fuente..."

# Crear un directorio temporal y navegar a él
mkdir -p /tmp/source_deps && cd /tmp/source_deps

# --- Instalar OSQP ---
info "Instalando OSQP..."
git clone --recursive https://github.com/osqp/osqp.git
cd osqp
git checkout v0.6.3
git submodule update --init --recursive
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
cd ../..

# --- Instalar Osqp-Eigen ---
info "Instalando Osqp-Eigen..."
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
git checkout v0.8.1
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
cd ../..

# --- Limpieza ---
info "Limpiando archivos de compilación de dependencias..."
cd ~
rm -rf /tmp/source_deps

# Actualizar el caché del enlazador
sudo ldconfig

# --- 6. Compilación del Workspace ---
info "Compilando el workspace de ROS 2..."
cd ~/tesis_ws

# Nota sobre permisos de Geomagic Touch
info "Nota sobre permisos de Geomagic Touch"
echo "Si el Geomagic Touch aparece como /dev/ttyACM0 y tienes errores de permisos,"
echo "es posible que debas añadir tu usuario al grupo 'dialout' en el sistema anfitrión (host)."
echo "En entornos con Distrobox, este paso no debe ejecutarse dentro del contenedor."
echo "Consulta la sección 'Troubleshooting' del README.md. Este cambio requiere reiniciar el host."

# Compilar paquetes en el orden especificado
colcon build --symlink-install --packages-select omni_msgs ur5_description omni_description ur5_impedance ur5_kinematics ur5_sliding ur5_scaled_sender ur5_bringup
colcon build --symlink-install --packages-select omni_common griper_control geomagic_interface
colcon build --symlink-install --packages-select ur5_controller
colcon build --symlink-install --packages-select ur5_interfaz_library ur5_panel

info "¡Instalación completada con éxito!"
echo "Recuerda cerrar y volver a abrir tu terminal o ejecutar 'source ~/.bashrc' para que todos los cambios surtan efecto."

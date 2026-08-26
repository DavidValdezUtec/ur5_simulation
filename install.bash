#!/bin/bash
#
# Script de instalación para el entorno de teleoperación de UR5 con Geomagic Touch.
# Este script es ejecutado por bootstrap.sh y respeta la ruta del workspace elegida.

set -e

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
    exit 1
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

section() {
    echo ""
    echo -e "${BLUE}================================================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================================================================${NC}"
}

# --- Parsear argumentos ---
WORKSPACE_PATH="$HOME/tesis_ws"  # Por defecto

while [[ $# -gt 0 ]]; do
    case $1 in
        --workspace-path)
            WORKSPACE_PATH="$2"
            shift 2
            ;;
        *)
            error "Opción desconocida: $1"
            ;;
    esac
done

info "Ruta del workspace: $WORKSPACE_PATH"
SRC_PATH="$WORKSPACE_PATH/src"
DOWNLOADS_DIR="$WORKSPACE_PATH/.downloads"

# --- 0. Detección y Configuración del Entorno (Host vs Distrobox) ---
section "Verificar entorno"

INSIDE_DISTROBOX=false

if [ ! -f /run/.containerenv ]; then
    # No estamos en un contenedor
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        info "Sistema detectado: $PRETTY_NAME"
    fi

    if [[ "$ID" != "ubuntu" || "$VERSION_ID" != "22.04" ]]; then
        warn "Sistema operativo distinto a Ubuntu 22.04 detectado."
        warn "Algunos paquetes pueden no estar disponibles o tener versiones incompatibles."
    fi
else
    INSIDE_DISTROBOX=true
    info "Ejecución dentro de un contenedor Distrobox detectada."
    info "Usando archivos pre-descargados desde: $DOWNLOADS_DIR"
fi

# --- 1. Localizar y usar el repositorio descargado ---
section "Verificar repositorio"

REPO_PATH="$SRC_PATH/ur5_simulation"

if [ ! -d "$REPO_PATH" ]; then
    error "No se encontró el repositorio en: $REPO_PATH"
fi

info "Usando repositorio en: $REPO_PATH"
cd "$REPO_PATH"

# --- 2. Instalación de ROS 2 Humble ---
section "Instalar ROS 2 Humble"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    info "ROS 2 Humble ya está instalado."
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

    # Añadir el script de configuración al .bashrc
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        info "Añadiendo la configuración de ROS 2 a ~/.bashrc"
        echo '
# ROS 2 Humble configuration
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    # Forzar X11 para evitar errores con Wayland en algunas interfaces gráficas
    export QT_QPA_PLATFORM=xcb
fi' >> ~/.bashrc
    else
        info "La configuración de ROS 2 ya existe en ~/.bashrc."
    fi

    # Cargar la configuración de ROS 2 para la sesión actual
    source /opt/ros/humble/setup.bash
    
    info "ROS 2 Humble instalado correctamente"
fi

# --- 3. Instalación de Drivers de Geomagic Touch ---
section "Instalar drivers de Geomagic Touch"

# Prerequisitos
info "Añadiendo usuario al grupo dialout para permisos de puerto..."
sudo usermod -a -G dialout "$USER"

info "Instalando dependencias gráficas para los drivers..."
sudo apt update
sudo apt install -y build-essential libncurses5-dev freeglut3-dev zlib1g-dev libncurses5 cmake git

# Crear un directorio temporal para la instalación
TEMP_DRIVER_DIR=$(mktemp -d)
cd "$TEMP_DRIVER_DIR"

# Usar archivo pre-descargado
if [ -f "$DOWNLOADS_DIR/TouchDriver_2024_09_19.tgz" ]; then
    info "Usando Touch Driver pre-descargado..."
    cp "$DOWNLOADS_DIR/TouchDriver_2024_09_19.tgz" .
else
    info "Descargando Touch Driver 2024-09-19... (no estaba pre-descargado)"
    curl -L -o TouchDriver_2024_09_19.tgz https://s3.us-east-1.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2024_09_19.tgz
fi

tar -xzvf TouchDriver_2024_09_19.tgz
cd TouchDriver_2024_09_19

set +e
sudo bash install_haptic_driver
INSTALL_HAPTIC_STATUS=$?
set -e

# Verificar instalación de drivers
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
    info "Drivers de Geomagic Touch detectados en el sistema."
else
    if [[ $INSTALL_HAPTIC_STATUS -ne 0 ]]; then
        error "El instalador de Geomagic Touch falló y no se encontraron los archivos esperados."
    fi
    error "No se encontraron los archivos esperados tras instalar los drivers."
fi
cd ..

# Usar archivo pre-descargado
if [ -f "$DOWNLOADS_DIR/openhaptics_developer.tar.gz" ]; then
    info "Usando OpenHaptics pre-descargado..."
    cp "$DOWNLOADS_DIR/openhaptics_developer.tar.gz" .
else
    info "Descargando OpenHaptics Developer Edition... (no estaba pre-descargado)"
    curl -L -o openhaptics_developer.tar.gz "https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz"
fi

tar -xzvf openhaptics_developer.tar.gz
cd openhaptics_3.4-0-developer-edition-amd64

info "Instalando OpenHaptics de forma desatendida..."
set +e
# Pasamos 'y' (aceptar) y luego 'q' (cancelar reinicio) automáticamente
echo -e "y\nq\n" | sudo ./install
OPENHAPTICS_INSTALL_STATUS=$?
set -e

# Verificar instalación de OpenHaptics
OPENHAPTICS_MARKERS=(
    "/usr/lib/libHD.so"
    "/usr/lib/libHL.so"
    "/usr/lib/libHD.so.3.4.0"
    "/usr/lib/libHL.so.3.4.0"
    "/usr/lib/x86_64-linux-gnu/libHD.so"
    "/usr/lib/x86_64-linux-gnu/libHL.so"
    "/usr/include/HD/hd.h"
    "/usr/include/HL/hl.h"
)
OPENHAPTICS_INSTALLED=0
for marker in "${OPENHAPTICS_MARKERS[@]}"; do
    if [[ -f "$marker" ]]; then
        OPENHAPTICS_INSTALLED=1
        break
    fi
done

if [[ $OPENHAPTICS_INSTALL_STATUS -ne 0 && $OPENHAPTICS_INSTALLED -eq 1 ]]; then
    info "OpenHaptics detectado. Se ignora el código de salida al cancelar reinicio con 'q'."
elif [[ $OPENHAPTICS_INSTALL_STATUS -ne 0 ]]; then
    error "La instalación de OpenHaptics falló (código $OPENHAPTICS_INSTALL_STATUS)."
elif [[ $OPENHAPTICS_INSTALLED -eq 0 ]]; then
    error "El instalador finalizó, pero no se detectaron archivos de OpenHaptics."
fi
cd ..

# Copiar los binarios de calibración (preservando estructura de directorios)
info "Copiando binarios de calibración..."
LOCAL_TOUCH_BIN_DIR="$HOME/.local/share/geomagic/bin"
DRIVER_BIN_DIR="$TEMP_DRIVER_DIR/TouchDriver_2024_09_19/bin"
mkdir -p "$LOCAL_TOUCH_BIN_DIR"

if compgen -G "$DRIVER_BIN_DIR/*" > /dev/null; then
    # Copia recursiva manteniendo estructura (ej: carpeta fonts)
    cp -r "$DRIVER_BIN_DIR"/* "$LOCAL_TOUCH_BIN_DIR/"
    # Asegurar permisos de ejecución recursivamente
    chmod -R +x "$LOCAL_TOUCH_BIN_DIR"
    info "Binarios de calibración copiados y permisos asignados"
else
    error "No se encontraron binarios en $DRIVER_BIN_DIR"
fi

# Limpiar archivos de instalación
info "Limpiando archivos temporales..."
cd ~
rm -rf "$TEMP_DRIVER_DIR"

info "Drivers de Geomagic Touch instalados correctamente"

# --- 4. Instalación de Dependencias Adicionales de ROS ---
section "Instalar dependencias de ROS 2"

info "Instalando paquetes de ROS 2..."
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

info "Dependencias de ROS 2 instaladas"

# --- 5. Instalación de Dependencias desde Código Fuente (OSQP y Osqp-Eigen) ---
section "Instalar OSQP y Osqp-Eigen desde código fuente"

mkdir -p /tmp/source_deps && cd /tmp/source_deps

info "Compilando OSQP v0.6.3..."
if [ -d "$DOWNLOADS_DIR/osqp" ]; then
    info "Usando OSQP pre-descargado..."
    cp -r "$DOWNLOADS_DIR/osqp" .
else
    info "Clonando OSQP... (no estaba pre-descargado)"
    git clone --recursive https://github.com/osqp/osqp.git
    cd osqp
    git checkout v0.6.3
    git submodule update --init --recursive
    cd ..
fi

cd osqp
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
cd ../..

info "Compilando Osqp-Eigen v0.8.1..."
if [ -d "$DOWNLOADS_DIR/osqp-eigen" ]; then
    info "Usando Osqp-Eigen pre-descargado..."
    cp -r "$DOWNLOADS_DIR/osqp-eigen" .
else
    info "Clonando Osqp-Eigen... (no estaba pre-descargado)"
    git clone https://github.com/robotology/osqp-eigen.git
fi

cd osqp-eigen
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
cd ../..

info "Limpiando archivos de compilación..."
cd ~
rm -rf /tmp/source_deps

sudo ldconfig
info "OSQP y Osqp-Eigen instalados correctamente"

# --- 6. Compilación del Workspace ---
section "Compilar workspace de ROS 2"

cd "$WORKSPACE_PATH"

# Agregar setup del workspace al .bashrc
if ! grep -q "source.*$WORKSPACE_PATH/install/setup.bash" ~/.bashrc; then
    info "Añadiendo setup del workspace a ~/.bashrc"
    echo "
# Workspace setup
if [ -f \"$WORKSPACE_PATH/install/setup.bash\" ]; then
    source \"$WORKSPACE_PATH/install/setup.bash\"
fi" >> ~/.bashrc
fi

info "Compilando paquetes del workspace..."
info "Esto puede tomar varios minutos..."
echo ""

colcon build --symlink-install --packages-select omni_msgs ur5_description omni_description ur5_impedance ur5_kinematics ur5_sliding ur5_scaled_sender ur5_bringup
colcon build --symlink-install --packages-select omni_common griper_control geomagic_interface
colcon build --symlink-install --packages-select ur5_controller
colcon build --symlink-install --packages-select ur5_interfaz_library ur5_panel

info "Workspace compilado correctamente"

# --- 7. Crear accesos directos (.desktop) para Geomagic Touch ---
section "Crear accesos directos de Geomagic Touch"

APPS_DIR="$HOME/.local/share/applications"
mkdir -p "$APPS_DIR"

# Detectar si estamos dentro de Distrobox
if [ -f /run/.containerenv ]; then
    DISTROBOX_MODE=true
    EXEC_CMD="distrobox enter ubuntu22 -- $HOME/.local/share/geomagic/bin/TouchCheckup"
    EXEC_CMD_SETUP="distrobox enter ubuntu22 -- $HOME/.local/share/geomagic/bin/Touch_HeadlessSetup"
else
    DISTROBOX_MODE=false
    EXEC_CMD="$HOME/.local/share/geomagic/bin/TouchCheckup"
    EXEC_CMD_SETUP="$HOME/.local/share/geomagic/bin/Touch_HeadlessSetup"
fi

# Crear acceso directo para interfaz de prueba
DESKTOP_FILE="$APPS_DIR/Geomagic_Touch.desktop"
cat <<EOF > "$DESKTOP_FILE"
[Desktop Entry]
Encoding=UTF-8
Version=1.0
Type=Application
Terminal=false
Exec=$EXEC_CMD
Name=Geomagic Touch
Categories=Application;
Comment=Interfaz gráfica de Geomagic Touch para pruebas
Icon=applications-other
EOF

# Crear acceso directo para configuración
DESKTOP_FILE_SETUP="$APPS_DIR/Geomagic_Touch_Setup.desktop"
cat <<EOF > "$DESKTOP_FILE_SETUP"
[Desktop Entry]
Encoding=UTF-8
Version=1.0
Type=Application
Terminal=true
Exec=$EXEC_CMD_SETUP
Name=Geomagic Touch Setup
Categories=Application;
Comment=Configurar y calibrar dispositivo Geomagic Touch
Icon=applications-other
EOF

chmod +x "$DESKTOP_FILE" "$DESKTOP_FILE_SETUP"
info "Accesos directos creados en: $APPS_DIR"

# Copiar al Escritorio si existe
if command -v xdg-user-dir >/dev/null 2>&1; then
    DESKTOP_DIR=$(xdg-user-dir DESKTOP)
elif [ -d "$HOME/Escritorio" ]; then
    DESKTOP_DIR="$HOME/Escritorio"
else
    DESKTOP_DIR="$HOME/Desktop"
fi

if [ -n "$DESKTOP_DIR" ] && [ -d "$DESKTOP_DIR" ]; then
    cp "$DESKTOP_FILE" "$DESKTOP_DIR/"
    cp "$DESKTOP_FILE_SETUP" "$DESKTOP_DIR/"
    chmod +x "$DESKTOP_DIR/Geomagic_Touch.desktop" "$DESKTOP_DIR/Geomagic_Touch_Setup.desktop"
    info "Accesos directos también copiados a: $DESKTOP_DIR"
fi

# --- Nota sobre permisos de Geomagic Touch ---
section "Información importante"

echo ""
echo -e "${YELLOW}⚠ Cambios de permisos y reinicio requeridos:${NC}"
echo ""
echo "Se han realizado los siguientes cambios:"
echo "  ✓ Tu usuario fue añadido al grupo 'dialout'"
echo "  ✓ Los drivers de Geomagic Touch fueron instalados"
echo "  ✓ Se crearon accesos directos en el menú de aplicaciones"
echo ""
echo -e "${YELLOW}IMPORTANTE:${NC} Estos cambios requieren reiniciar el sistema para tomar efecto."
echo ""
if [ "$DISTROBOX_MODE" = true ]; then
    echo "Notas para Distrobox:"
    echo "  - Los permisos del grupo 'dialout' afectan al HOST"
    echo "  - Debes reiniciar el HOST, no solo el contenedor"
    echo "  - Los accesos directos funcionarán desde el escritorio del host"
fi
echo ""

# --- Prompt de reinicio ---
section "Reinicio del sistema"

echo ""
read -p "¿Deseas reiniciar el equipo ahora para aplicar todos los cambios? [s/N]: " REBOOT_RESPONSE

if [[ "$REBOOT_RESPONSE" =~ ^[SsYy]$ ]]; then
    echo ""
    info "Reiniciando el equipo en 10 segundos... (Ctrl+C para cancelar)"
    sleep 10
    sudo reboot
else
    echo ""
    warn "Recuerda reiniciar el equipo manualmente para que todos los cambios surtan efecto."
fi

# --- Mensaje final ---
section "¡Instalación completada exitosamente!"

echo ""
echo "Próximos pasos (después de reiniciar):"
echo ""
echo "  1. Cierra y vuelve a abrir tu terminal"
echo "  2. Navega al workspace:"
echo "     cd $WORKSPACE_PATH"
echo "  3. Inicia el entorno:"
echo "     source install/setup.bash"
echo ""
echo "Geomagic Touch:"
echo "  - Encontrarás accesos directos en tu escritorio"
echo "  - Abre 'Geomagic Touch Setup' para calibrar el dispositivo"
echo "  - Abre 'Geomagic Touch' para probar el dispositivo"
echo ""
if [ -f /run/.containerenv ]; then
    echo "Nota: Estás en un contenedor Distrobox."
    echo "      Para volver al host: exit"
    echo "      Para entrar de nuevo: distrobox enter ubuntu22"
fi

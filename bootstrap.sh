#!/bin/bash
# Bootstrap installer para UR5 Geomagic Touch
# Uso: curl -fsSL https://repo/bootstrap.sh | bash
# O ejecutar localmente: bash bootstrap.sh

set -e

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# --- PASO 0: Detectar ubicación del script (si existe localmente) ---
SCRIPT_DIR=""
if [ -n "${BASH_SOURCE[0]}" ] && [ -f "${BASH_SOURCE[0]}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi

section "Seleccionar ubicación de instalación"

echo "¿Dónde deseas instalar el workspace?"
echo ""
if [ -n "$SCRIPT_DIR" ]; then
    echo "  [1] En la ubicación del script: $SCRIPT_DIR"
fi
echo "  [2] En el home (por defecto): \$HOME"
echo ""

if [ -n "$SCRIPT_DIR" ]; then
    read -p "Selecciona [1 o 2] (por defecto: 2): " install_choice
    install_choice="${install_choice:-2}"
else
    install_choice="2"
fi

if [[ "$install_choice" == "1" && -n "$SCRIPT_DIR" ]]; then
    BASE_PATH="$SCRIPT_DIR"
    info "Se instalará en: $BASE_PATH"
else
    BASE_PATH="$HOME"
    info "Se instalará en: $HOME"
fi

# Ruta completa del workspace
WORKSPACE_PATH="$BASE_PATH/tesis_ws"
SRC_PATH="$WORKSPACE_PATH/src"

info "Estructura final:"
echo "  $SRC_PATH/"
echo "  └── ur5_simulation/"

# --- PASO 1: Verificar y instalar git si es necesario ---
section "Verificar git"

if ! command -v git &> /dev/null; then
    warn "git no está instalado. Intentando instalar..."
    
    if command -v apt &> /dev/null; then
        sudo apt update && sudo apt install -y git
        info "git instalado correctamente"
    else
        error "No se puede instalar git automáticamente en este sistema."
    fi
else
    info "git ya está instalado"
fi

# --- PASO 2: Crear estructura de directorios ---
section "Crear estructura de workspace"

mkdir -p "$SRC_PATH"
cd "$SRC_PATH"
info "Directorio creado: $SRC_PATH"

# --- PASO 3: Descargar repositorio ---
section "Descargar repositorio"

REPO_URL="https://github.com/DavidValdezUtec/ur5_simulation.git"
REPO_NAME="ur5_simulation"
REPO_PATH="$SRC_PATH/$REPO_NAME"

if [ ! -d "$REPO_PATH" ]; then
    info "Descargando repositorio desde: $REPO_URL"
    git clone "$REPO_URL" "$REPO_PATH"
    info "Repositorio descargado correctamente"
else
    warn "El repositorio ya existe en: $REPO_PATH"
    read -p "¿Deseas actualizarlo? [y/n] (por defecto: n): " update_repo
    if [[ "$update_repo" =~ ^[Yy]$ ]]; then
        cd "$REPO_PATH"
        git pull origin main
        cd "$SRC_PATH"
        info "Repositorio actualizado"
    fi
fi

# --- PASO 4: Detectar entorno (Host vs Distrobox) ---
section "Detectar entorno del sistema"

# Determinar si se usará distrobox ANTES de descargas
WILL_USE_DISTROBOX=false

INSIDE_DISTROBOX=false
if [ -f /run/.containerenv ]; then
    info "Se detectó ejecución dentro de un contenedor Distrobox"
    INSIDE_DISTROBOX=true
else
    info "Se detectó ejecución en el host"
    
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        info "Sistema detectado: $PRETTY_NAME"
    fi
    
    if [[ "$ID" != "ubuntu" || "$VERSION_ID" != "22.04" ]]; then
        warn "Este sistema no es Ubuntu 22.04"
        warn "Se recomienda usar Distrobox con Ubuntu 22.04 para garantizar compatibilidad"
        echo ""
        read -p "¿Usar Distrobox? [y/n] (por defecto: y): " use_distrobox
        use_distrobox="${use_distrobox:-y}"
        
        if [[ "$use_distrobox" =~ ^[Yy]$ ]]; then
            WILL_USE_DISTROBOX=true
        fi
    fi
fi

# --- PASO 5: Pre-descargar archivos (antes de distrobox si aplica) ---
if [ "$WILL_USE_DISTROBOX" = true ] || [ "$INSIDE_DISTROBOX" = false ]; then
    section "Pre-descargar archivos de instalación"
    
    DOWNLOADS_DIR="$WORKSPACE_PATH/.downloads"
    mkdir -p "$DOWNLOADS_DIR"
    
    info "Descargando archivos de Geomagic Touch..."
    
    # Geomagic Touch Driver
    if [ ! -f "$DOWNLOADS_DIR/TouchDriver_2024_09_19.tgz" ]; then
        curl -L -o "$DOWNLOADS_DIR/TouchDriver_2024_09_19.tgz" \
            https://s3.us-east-1.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2024_09_19.tgz
        info "Touch Driver descargado"
    else
        info "Touch Driver ya descargado"
    fi
    
    # OpenHaptics Developer Edition
    if [ ! -f "$DOWNLOADS_DIR/openhaptics_developer.tar.gz" ]; then
        curl -L -o "$DOWNLOADS_DIR/openhaptics_developer.tar.gz" \
            "https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz"
        info "OpenHaptics descargado"
    else
        info "OpenHaptics ya descargado"
    fi
    
    info "Descargando repositorios de código fuente..."
    
    # OSQP
    if [ ! -d "$DOWNLOADS_DIR/osqp" ]; then
        git clone --recursive https://github.com/osqp/osqp.git "$DOWNLOADS_DIR/osqp"
        cd "$DOWNLOADS_DIR/osqp"
        git checkout v0.6.3
        git submodule update --init --recursive
        cd "$WORKSPACE_PATH"
        info "OSQP descargado"
    else
        info "OSQP ya descargado"
    fi
    
    # Osqp-Eigen
    if [ ! -d "$DOWNLOADS_DIR/osqp-eigen" ]; then
        git clone https://github.com/robotology/osqp-eigen.git "$DOWNLOADS_DIR/osqp-eigen"
        cd "$DOWNLOADS_DIR/osqp-eigen"
        git checkout v0.8.1
        cd "$WORKSPACE_PATH"
        info "Osqp-Eigen descargado"
    else
        info "Osqp-Eigen ya descargado"
    fi
    
    info "Todos los archivos han sido pre-descargados en: $DOWNLOADS_DIR"
fi

# --- PASO 6: Entrar a Distrobox si es necesario ---
if [ "$WILL_USE_DISTROBOX" = true ]; then
    # Verificar Distrobox
    if ! command -v distrobox &> /dev/null; then
        error "Distrobox no está instalado."
        echo ""
        echo "Instálalo con:"
        echo "  curl -s https://raw.githubusercontent.com/89luca89/distrobox/main/install | sudo sh"
        exit 1
    fi
    
    # Crear/entrar a contenedor
    if ! distrobox list | grep -q 'ubuntu22'; then
        section "Crear contenedor Distrobox"
        info "Creando contenedor Distrobox 'ubuntu22' con Ubuntu 22.04..."
        distrobox create --name ubuntu22 --image ubuntu:22.04
        info "Contenedor creado"
    else
        info "Contenedor Distrobox 'ubuntu22' ya existe"
    fi
    
    section "Ejecutar instalación en Distrobox"
    info "Entrando al contenedor y ejecutando instalador..."
    info "Los archivos ya fueron descargados en el host"
    echo ""
    
    # Pasar la ruta del workspace al instalador dentro del contenedor
    distrobox enter ubuntu22 -- bash "$REPO_PATH/install.bash" --workspace-path "$WORKSPACE_PATH"
    
    exit_code=$?
    if [ $exit_code -eq 0 ]; then
        section "Instalación completada"
        info "Setup finalizado exitosamente en el contenedor"
        echo ""
        echo "Próximos pasos:"
        echo "  1. Cierra y vuelve a abrir la terminal"
        echo "  2. O ejecuta: source ~/.bashrc"
        echo "  3. Luego: cd $WORKSPACE_PATH"
        echo "  4. Para entrar al contenedor: distrobox enter ubuntu22"
    fi
    exit $exit_code
fi

# --- PASO 7: Ejecutar install.bash ---
section "Ejecutar instalador principal"

if [ -f "$REPO_PATH/install.bash" ]; then
    info "Ejecutando instalador en el host..."
    bash "$REPO_PATH/install.bash" --workspace-path "$WORKSPACE_PATH"
    
    exit_code=$?
    if [ $exit_code -eq 0 ]; then
        section "Instalación completada"
        echo ""
        echo "Próximos pasos:"
        echo "  1. Cierra y vuelve a abrir la terminal"
        echo "  2. O ejecuta: source ~/.bashrc"
        echo "  3. Luego: cd $WORKSPACE_PATH"
    fi
    exit $exit_code
else
    error "No se encontró install.bash en el repositorio descargado"
fi

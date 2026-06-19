#!/bin/bash
# --- 3. Instalación de Drivers de Geomagic Touch ---
sudo apt install -y build-essential cmake git curl
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
set +e
sudo ./install
OPENHAPTICS_INSTALL_STATUS=$?
set -e

# El instalador puede devolver salida no-cero al cancelar el reinicio con 'q'.
# Verificamos archivos instalados para decidir si se puede continuar.
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
    echo "ERROR: La instalación de OpenHaptics falló (código $OPENHAPTICS_INSTALL_STATUS)."
    exit 1
elif [[ $OPENHAPTICS_INSTALLED -eq 0 ]]; then
    echo "ERROR: El instalador finalizó, pero no se detectaron archivos de OpenHaptics."
    exit 1
fi
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

# --- 4. Crear accesos directos (.desktop) ---
info "Creando accesos directos de la interfaz gráfica..."

APPS_DIR="$HOME/.local/share/applications"
mkdir -p "$APPS_DIR"

# Verificamos si estamos dentro de Distrobox (revisando si existe .containerenv)
if [ -f /run/containerenv ]; then
    EXEC_CMD="/usr/bin/distrobox enter ubuntu22 -- $HOME/.local/share/geomagic/bin/TouchCheckup"
else
    EXEC_CMD="$HOME/.local/share/geomagic/bin/TouchCheckup"
fi

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
Comment=Interfaz grafica de Geomagic Touch
EOF

chmod +x "$DESKTOP_FILE"

# Identificar la carpeta del Escritorio
if command -v xdg-user-dir >/dev/null 2>&1; then
    DESKTOP_DIR=$(xdg-user-dir DESKTOP)
elif [ -d "$HOME/Escritorio" ]; then
    DESKTOP_DIR="$HOME/Escritorio"
else
    DESKTOP_DIR="$HOME/Desktop"
fi

# Copiar al Escritorio si el directorio existe
if [ -n "$DESKTOP_DIR" ] && [ -d "$DESKTOP_DIR" ]; then
    cp "$DESKTOP_FILE" "$DESKTOP_DIR/"
    chmod +x "$DESKTOP_DIR/Geomagic_Touch.desktop"
    info "Acceso directo creado en: $DESKTOP_DIR"
fi



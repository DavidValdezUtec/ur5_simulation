# UR5e Simulation con Geomagic Touch

Software de teleoperación de dos robots **UR5e** mediante dos dispositivos hápticos **Geomagic Touch**, en ROS 2 Humble. Incluye instalador automatizado (host Ubuntu 22.04 o Distrobox), simulación en Gazebo y driver para los robots reales.

---

## 📋 Requisitos Previos

### Sistema Operativo
- **Ubuntu 22.04 LTS** (instalación nativa)
- **Cualquier distro con Distrobox** (el instalador lo detecta y lo configura solo)

### Hardware
- 2x dispositivo **Geomagic Touch** (opcional para simulación pura; requerido para teleoperación háptica)
- 2x robot **UR5e** real (opcional; solo para operar hardware físico)
- Mínimo 8 GB RAM (recomendado para que `colcon build` no falle por falta de memoria)
- ~10 GB de espacio libre en disco

### Software (se instala automáticamente)
- ROS 2 Humble Desktop + herramientas de desarrollo
- OpenHaptics SDK v3.4 + Touch Driver
- OSQP v0.6.3 + Osqp-Eigen v0.8.1
- `ros-humble-ur`, `ros-humble-ur-simulation-gz`, `ros-humble-pinocchio`, etc.

---

## 🚀 Instalación

### ⚡ Opción 1: Un solo comando (Recomendada)

```bash
curl -fsSL https://raw.githubusercontent.com/DavidValdezUtec/ur5_simulation/main/bootstrap.sh | bash
```

**Qué hace automáticamente:**
1. Verifica/instala `git`
2. Clona este repositorio en `~/tesis_ws/src/ur5_simulation`
3. Detecta tu sistema: si no es Ubuntu 22.04, ofrece usar **Distrobox** (crea el contenedor `ubuntu22` si hace falta)
4. Pre-descarga los archivos pesados (Touch Driver, OpenHaptics, OSQP, Osqp-Eigen) **antes** de entrar al contenedor, para no repetir descargas
5. Instala ROS 2 Humble, drivers de Geomagic Touch y dependencias
6. Compila el workspace con `colcon build`
7. Crea accesos directos `.desktop` para Geomagic Touch (menú y escritorio)
8. Pregunta si deseas reiniciar el equipo (necesario para que el grupo `dialout` tome efecto)

> Al ejecutarse por `curl | bash` no hay un archivo local que detectar, así que instala directo en `~/tesis_ws` sin preguntar la ubicación. Durante el proceso puede pedirte la contraseña de `sudo` y confirmar la instalación de los drivers Geomagic.

---

### 📝 Opción 2: Manual (paso a paso)

Para tener control total o revisar cada paso antes de ejecutarlo:

```bash
# 1. Crear el workspace y clonar el repositorio en la ruta que espera install.bash
mkdir -p ~/tesis_ws/src
cd ~/tesis_ws/src
git clone https://github.com/DavidValdezUtec/ur5_simulation.git
cd ur5_simulation
```

**En Ubuntu 22.04 nativo**, ejecuta el instalador directamente:

```bash
bash install.bash --workspace-path ~/tesis_ws
```

**En cualquier otra distro**, usa `bootstrap.sh` (te preguntará la ubicación y configurará Distrobox):

```bash
bash bootstrap.sh
```

Ambos caminos terminan compilando el workspace y dejándote el mismo resultado que la Opción 1; la diferencia es que aquí ves cada paso y puedes cancelar en cualquier punto (`set -e`: el script se detiene ante el primer error).

---

## 📁 Estructura del Repositorio

```
~/tesis_ws/
├── src/
│   └── ur5_simulation/
│       ├── bootstrap.sh              ← Instalador (detecta host/Distrobox)
│       ├── install.bash              ← Instalador principal (llamado por bootstrap.sh)
│       ├── Geomagic_Touch_ROS2/       ← Driver Geomagic Touch
│       ├── geomagic_interface/
│       ├── griper_control/
│       ├── ur5_bringup/              ← Paquete de bringup (legado, un solo robot)
│       ├── ur5e_bringup/             ← Bringup actual: 2 robots (r1, r2), Gazebo + driver real
│       ├── ur5_controller/           ← Nodo de control (controller_backup)
│       ├── ur5_description/          ← URDF/xacro (soporta ur5 y ur5e vía config)
│       ├── ur5_impedance/
│       ├── ur5_interfaz/
│       ├── ur5_kinematics/
│       ├── ur5_sliding/
│       └── ur5_torque/
├── build/ install/ log/              ← Generados por colcon
└── .downloads/                       ← Cache de archivos pre-descargados
```

Los dos robots (`r1`, `r2`) y sus IPs/tipo se configuran en `ur5e_bringup/config/config.json`.

---

## 🎮 Uso

### 1. Cargar el workspace

En cada terminal nueva (o agrégalo a `~/.bashrc`, el instalador ya lo hace por ti):

```bash
source ~/tesis_ws/install/setup.bash
```

### 2. Simulación en Gazebo (los 2 robots UR5e)

```bash
ros2 launch ur5e_bringup multi_ur5e_sim.launch.py

# Con GUI de Gazebo visible y sin sensor de fuerza/torque:
ros2 launch ur5e_bringup multi_ur5e_sim.launch.py gui:=true ft_sensor:=false
```

> ⚠️ **Nota:** si instalaste con una versión de `install.bash` anterior a este cambio, `ur5e_bringup` puede no haberse compilado (no está en la lista de `--packages-select`). Si el `launch` falla con "package not found", compílalo manualmente:
> ```bash
> cd ~/tesis_ws && colcon build --symlink-install --packages-select ur5e_bringup
> ```

### 3. Teleoperación con Geomagic Touch

Antes de conectar los dispositivos, renómbralos como `phantom2` (brazo izquierdo) y `phantom3` (brazo derecho), calíbralos por separado, y conéctalos en orden: primero izquierdo, luego derecho. Luego:

```bash
ros2 launch omni_common dual_omni_state.launch.py
```

### 4. Driver de los robots reales

Conecta ambos UR5e al mismo router (IP de las urcaps: `192.168.10.101`; el equipo con IP estática en la misma red, máscara `255.255.255.0`). Las IPs de los robots son `192.168.10.103` y `192.168.10.104`.

```bash
ros2 launch ur_robot_driver dual_control.launch.py r1_type:=ur5e r2_type:=ur5e
```

Con los robots ya posicionados, lanza los controladores (uno por robot):

```bash
ros2 run ur5_controller controller_backup --ros-args \
  -p control_topic:="/scaled_joint_trajectory_controller/joint_trajectory" \
  -p ur:="ur5e" -p nmspace:="r1" \
  -p geomagic_topic:="/phantom3/pose" -p geomagic_button_topic:="/phantom3/button" \
  -p csv_log_enable:="true" -p geomagic:="true"
```

```bash
ros2 run ur5_controller controller_backup --ros-args \
  -p control_topic:="/scaled_joint_trajectory_controller/joint_trajectory" \
  -p ur:="ur5e" -p nmspace:="r2" \
  -p geomagic_topic:="/phantom2/pose" -p geomagic_button_topic:="/phantom2/button" \
  -p csv_log_enable:="true" -p geomagic:="true"
```

---

## 🔍 Troubleshooting

### "Permiso denegado" en `/dev/ttyACM0`

Los dispositivos serie se asignan al grupo `dialout`. El instalador ya te agrega a ese grupo, pero si necesitas hacerlo manualmente:

```bash
sudo usermod -a -G dialout $USER
```

**Debes reiniciar el equipo** para que el cambio de grupo tome efecto. En Distrobox, este cambio se aplica en el **host**, no dentro del contenedor.

### Geomagic Touch no se detecta

```bash
lsusb | grep -i sensable
```

Si aparece pero sigue sin funcionar, revisa que los drivers y las reglas `udev` se hayan instalado (`install.bash` lo verifica automáticamente y falla con un mensaje claro si algo no quedó bien).

### `colcon build` termina con "Killed" o "Terminado"

Te quedaste sin RAM durante la compilación. Compila con un solo núcleo:

```bash
colcon build --symlink-install --parallel-workers 1
```

### Distrobox no está instalado

```bash
curl -s https://raw.githubusercontent.com/89luca89/distrobox/main/install | sudo bash
```

---

## ⚠️ Notas Importantes

- **Reinicio obligatorio:** tras la instalación, reinicia el equipo para que los permisos de `dialout` y los drivers de Geomagic Touch queden activos.
- **Pre-descargas:** `bootstrap.sh` descarga todo lo pesado (drivers, OSQP) en el host **antes** de entrar a Distrobox, para no repetir descargas si necesitas reinstalar.
- **`ur5e_bringup` es el paquete vigente** para simular y lanzar los 2 robots; `ur5_bringup` se mantiene por compatibilidad con lanzadores anteriores de un solo robot.

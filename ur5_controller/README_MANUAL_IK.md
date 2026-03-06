# Manual IK Node - Documentación

## Descripción

Este nodo permite mover el robot UR5/UR5e desde una pose inicial (actual o definida) hasta una pose objetivo usando uno de los tres controladores disponibles:

- **QP**: Cinemática inversa con optimización cuadrática (publica cada iteración del proceso)
- **IMP**: Control de impedancia (publica a frecuencia fija)
- **SLD**: Control por modo deslizante (publica a frecuencia fija)

## Características Principales

### Controlador QP
- Publica **cada iteración** del proceso de optimización (hasta 150 iteraciones máximo)
- Ideal para ver el proceso de convergencia paso a paso
- No usa bucle externo, ejecuta todas las iteraciones y luego termina

### Controladores IMP y SLD
- Publican a la frecuencia definida en `control_rate` (por defecto 100 Hz)
- Cada publicación es un paso de control independiente
- Usan un bucle de control continuo hasta alcanzar el objetivo

## Uso

### 1. Iniciar el driver del robot UR5

```bash
# Para UR5
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5 \
  robot_ip:=192.168.0.101 \
  kinematics_params_file:="${HOME}/my_robot_calibration_ur5.yaml"

# Para UR5e
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.10.103 \
  kinematics_params_file:="${HOME}/my_robot_calibration_ur5e.yaml"
```

### 2. Activar el controlador del robot

Asegúrate de que el robot esté en modo `forward_position_controller`:

```bash
ros2 control load_controller --set-state active forward_position_controller
```

### 3. Ejecutar el nodo con el controlador deseado

#### Opción A: Usar controlador QP (publica cada iteración)

```bash
ros2 run ur5_controller manual_ik_node --ros-args \
  --params-file src/ur5_simulation/ur5_controller/config/manual_ik_qp.yaml
```

#### Opción B: Usar controlador IMP (Impedancia)

```bash
ros2 run ur5_controller manual_ik_node --ros-args \
  --params-file src/ur5_simulation/ur5_controller/config/manual_ik_imp.yaml
```

#### Opción C: Usar controlador SLD (Sliding Mode)

```bash
ros2 run ur5_controller manual_ik_node --ros-args \
  --params-file src/ur5_simulation/ur5_controller/config/manual_ik_sld.yaml
```

## Parámetros de Configuración

### Parámetros Comunes

| Parámetro | Tipo | Descripción | Valor por defecto |
|-----------|------|-------------|-------------------|
| `use_current_pose` | bool | Si true, usa la pose actual como inicial; si false usa `initial_pose` | true |
| `initial_pose` | vector[3] | Posición inicial [x,y,z] en metros (solo si use_current_pose=false) | [0, 0, 0] |
| `initial_orientation_quat` | vector[4] | Orientación inicial [x,y,z,w] | [0, 0, 0, 1] |
| `target_pose` | vector[3] | Posición objetivo [x,y,z] en metros | [0.55566, 0.42846, 0.5] |
| `target_orientation_quat` | vector[4] | Orientación objetivo [x,y,z,w] | [0.49603, 0.11866, 0.20079, 0.83639] |
| `controller` | string | Controlador a usar: "QP", "IMP" o "SLD" | "QP" |
| `namespace` | string | Namespace del robot (dejar vacío si no usa) | "" |
| `ur_model` | string | Modelo: "ur5" o "ur5e" | "ur5" |
| `control_rate` | double | Frecuencia de control para IMP/SLD en Hz | 100.0 |

### Parámetros del Controlador QP

| Parámetro | Tipo | Descripción | Valor por defecto |
|-----------|------|-------------|-------------------|
| `qp_max_iterations` | int | Máximo de iteraciones (limitado a 150) | 600 |
| `qp_control_hz` | double | Factor de escala de control | 100.0 |

### Parámetros del Controlador IMP

| Parámetro | Tipo | Descripción | Valor por defecto |
|-----------|------|-------------|-------------------|
| `imp_kp` | vector[7] | Ganancias proporcionales | [100.0, ...] |
| `imp_kd` | vector[7] | Ganancias derivativas | [10.0, ...] |
| `imp_dt` | double | Paso de tiempo | 0.01 |

### Parámetros del Controlador SLD

| Parámetro | Tipo | Descripción | Valor por defecto |
|-----------|------|-------------|-------------------|
| `sld_lambda` | vector[6] | Ganancias lambda | [0.1, ...] |
| `sld_k` | vector[6] | Ganancias k | [50.0, ...] |
| `sld_k2` | vector[6] | Ganancias k2 | [10.0, ...] |
| `sld_alpha` | double | Alpha | 0.01 |
| `sld_damping_factor` | double | Factor de amortiguamiento | 0.01 |
| `sld_dt` | double | Paso de tiempo | 0.01 |

## Comportamiento del Nodo

### Flujo de Ejecución

1. **Inicialización**: El nodo carga los parámetros y espera el primer mensaje de `/joint_states`

2. **Captura de Pose Inicial**:
   - Si `use_current_pose = true`: Captura la pose actual del robot
   - Si `use_current_pose = false`: Usa la pose definida en los parámetros

3. **Ejecución del Control**:
   - **QP**: Ejecuta todas las iteraciones y publica cada una con una pausa de 10ms entre iteraciones
   - **IMP/SLD**: Crea un timer que ejecuta el control a la frecuencia especificada

4. **Terminación**: El nodo termina cuando:
   - **QP**: Converge (error < 1e-4) o alcanza el máximo de iteraciones
   - **IMP/SLD**: El error de posición es menor a 1mm

## Tópicos

### Suscribe
- `/joint_states` (o `/<namespace>/joint_states`): Estado actual de las articulaciones

### Publica
- `/forward_position_controller/commands` (o `/<namespace>/forward_position_controller/commands`): Comandos de posición articular

## Ejemplos

### Ejemplo 1: Mover desde pose actual con QP

```bash
ros2 run ur5_controller manual_ik_node --ros-args \
  --params-file src/ur5_simulation/ur5_controller/config/manual_ik_qp.yaml \
  -p use_current_pose:=true \
  -p target_pose:="[0.5, 0.3, 0.4]"
```

### Ejemplo 2: Mover desde pose definida con IMP

```bash
ros2 run ur5_controller manual_ik_node --ros-args \
  --params-file src/ur5_simulation/ur5_controller/config/manual_ik_imp.yaml \
  -p use_current_pose:=false \
  -p initial_pose:="[0.6, 0.2, 0.3]" \
  -p target_pose:="[0.5, 0.4, 0.5]"
```

### Ejemplo 3: Usar con namespace para múltiples robots

```bash
ros2 run ur5_controller manual_ik_node --ros-args \
  --params-file src/ur5_simulation/ur5_controller/config/manual_ik_sld.yaml \
  -p namespace:="robot1"
```

## Notas Importantes

1. **Controlador QP**: Verás una publicación cada 10ms aproximadamente mientras converge. El proceso puede tomar entre 1-15 segundos dependiendo de la distancia al objetivo.

2. **Controladores IMP y SLD**: Se ejecutan continuamente hasta alcanzar el objetivo. El tiempo depende de las ganancias configuradas.

3. **Seguridad**: Siempre verifica que la pose objetivo sea alcanzable y esté dentro del espacio de trabajo del robot antes de ejecutar.

4. **Monitoreo**: Puedes ver los logs en tiempo real para seguir el progreso:
   ```bash
   ros2 topic echo /forward_position_controller/commands
   ```

## Troubleshooting

### El robot no se mueve
- Verifica que el controlador `forward_position_controller` esté activo
- Revisa que el topic `/joint_states` esté publicando datos
- Asegúrate de que la pose objetivo sea válida

### El controlador QP no converge
- Aumenta `qp_max_iterations`
- Verifica que la orientación objetivo sea válida (cuaternión normalizado)
- La pose objetivo podría estar fuera del espacio de trabajo

### Los controladores IMP/SLD son inestables
- Reduce las ganancias proporcionales
- Aumenta las ganancias derivativas
- Ajusta el `control_rate` (prueba con 50 Hz o 200 Hz)

# Final Challenge — Manual de operación

Manual paso a paso para la **Demostración del Prototipo** (100 pts, 10/6/2026)
en el Puzzlebot físico. Cubre los 3 criterios del PDF de evaluación oficial:

| # | Criterio | Pts |
|---|---|---|
| 1 | **Localización con odometría** — `/odom` publicado, elipsoide creciendo con desplazamiento | 20 |
| 2 | **Filtro de Kalman** — odometría + visión integrada, elipsoide se reduce al ver ArUco | 30 |
| 3 | **Navegación autónoma** — robot llega a 4-5 waypoints (dictados por profesores) con error <20 cm | 50 |
| | **Total** | **100** |

**Penalización por colisión**: −5 pts por cada una; >3 colisiones = descalificación.

**Tolerancia de precisión por waypoint**:

| Error vs waypoint | Puntaje |
|---|---|
| <5 cm | 100% |
| 5–10 cm | 90% |
| 10–15 cm | 70% |
| 15–20 cm | 50% |
| >20 cm | 0% |

**Dinámica**: profesores entregan **posición inicial + 4-5 waypoints**.
Robot se detiene en cada uno y **espera señal del profesor** para continuar.
La pausa se usa para medir el error.

---

## Tabla de contenidos

- [0. Arquitectura del sistema](#0-arquitectura-del-sistema)
- [1. Pre-flight checklist](#1-pre-flight-checklist)
- [2. Levantar el robot (Jetson)](#2-levantar-el-robot-jetson)
- [3. Verificar conexión desde la PC](#3-verificar-conexión-desde-la-pc)
- [4. Escena 1 — EKF + Teleop (Part 1)](#4-escena-1--ekf--teleop-part-1)
- [5. Escena 2 — Navegación autónoma (Part 2)](#5-escena-2--navegación-autónoma-part-2)
- [6. Escena 3 — Tests de robustez (opcional)](#6-escena-3--tests-de-robustez-opcional)
- [7. Reproducir bags para grabar el video](#7-reproducir-bags-para-grabar-el-video)
- [8. Estructura del video](#8-estructura-del-video)
- [9. Troubleshooting](#9-troubleshooting)
- [10. Mapeo de requisitos del PDF al código](#10-mapeo-de-requisitos-del-pdf-al-código)
- [11. Preguntas técnicas que pueden hacer los profesores](#11-preguntas-técnicas-que-pueden-hacer-los-profesores)

---

## 0. Arquitectura del sistema

```
              ┌───────────────────────────── Jetson (en el robot) ─────┐
              │                                                        │
              │  micro_ros_agent  ──/cmd_vel──→  firmware Hackerboard  │
              │       ▲                                  │             │
              │       │                                  ▼             │
              │       │                              motores DC        │
              │       │                                  │             │
              │       │                                  ▼             │
              │       │                            encoders            │
              │       │                                  │             │
              │       │                                  ▼             │
              │       │                          /VelocityEncL,R       │
              │       │                                                │
              │   aruco_jetson:                                        │
              │      video_source (CSI 320x240)                        │
              │      camera_info_publisher (jetson_cam.yaml)           │
              │      aruco_ros marker_publisher                        │
              │           │                                            │
              │           ▼                                            │
              │   /marker_publisher/markers (aruco_msgs/MarkerArray)   │
              │                          │                             │
              └──────────────────────────┼─────────────────────────────┘
                                         │  WiFi (solo poses, no imagen)
                                         ▼
              ┌──────────────────────── PC (tu computadora) ───────────┐
              │                                                        │
              │   aruco_ros_bridge:                                    │
              │      filtra IDs validos (drop falsos positivos)        │
              │      convierte aruco_msgs -> puzzlebot_msgs            │
              │                          │                             │
              │                          ▼                             │
              │                  /aruco_detections                     │
              │                          │                             │
              │                          ▼                             │
              │   ekf_localisation:  predict (encoders) + correct      │
              │                      (ArUco con mapa conocido)         │
              │                      publica /odom con covarianza      │
              │                          │                             │
              │     ┌────────────────────┼─────────────────────┐       │
              │     ▼                    ▼                     ▼       │
              │  multi_point_nav   covariance_visualizer    RViz       │
              │  ──/pre_cmd_vel─→ /pose_covariance_marker              │
              │           │                                            │
              │           ▼                                            │
              │  obstacle_avoidance ──/cmd_vel──→ (via WiFi al Jetson) │
              │                                                        │
              └────────────────────────────────────────────────────────┘
```

### Decisiones de diseño (para el video)

| Decisión | Razón |
|---|---|
| Detección ArUco en el **Jetson** y no en la PC | No hay que transmitir imagen por WiFi → no se cae con saturación de red. Solo los `Pose` (≈150 bytes) viajan. |
| EKF en la **PC** | Tiene más cómputo y permite RViz local sin latencia. |
| Filtro `allowed_ids` en el bridge | aruco_ros mete falsos positivos (un 249 encima de un 702 real) cuando el marker se desenfoca. Filtrar deja solo los IDs del mapa. |
| `observation_frame: base` en EKF | aruco_ros con `reference_frame=base` ya hace la TF camera_optical → base_footprint en el Jetson. Reusarlo evita hacerlo dos veces. |
| Bug 0 reactivo en `obstacle_avoidance` | Cumple el requisito "circumnavigate obstacles" sin necesidad de mapa SLAM, que está fuera del alcance del challenge. |
| `joint_state_publisher` con default ceros | El firmware no publica `/joint_states`; sin esto, las TFs de las llantas no resuelven y el RobotModel se ve "bugeado" en RViz. |

---

## 1. Pre-flight checklist

Antes de prender nada verifica:

- [ ] Pista 3 m × 3 m armada (pasillos 60 cm, paredes ≥25 cm de alto, grosor 1-2 cm).
- [ ] 8 markers ArUco de 10 cm impresos del diccionario **Original ArUco** (https://chev.me/arucogen/) en sus posiciones del mapa.
- [ ] Robot físicamente en el **punto verde** del mapa (≈ x=1.3, y=0.5) **mirando hacia la izquierda del mapa** (= -X en world frame).
- [ ] Puzzlebot prendido (LED de la Jetson encendido y de la Hackerboard también).
- [ ] PC en la misma red WiFi que el Jetson.
- [ ] Calibración de cámara `jetson_cam.yaml` ya cargada (viene del OneNote, una sola vez).
- [ ] En el launch del Jetson `aruco_jetson.launch.py`:
  - [ ] `marker_size: 0.10`
  - [ ] `dictionary: "ARUCO"`

---

## 2. Levantar el robot (Jetson)

Necesitas **DOS** terminales SSH al Jetson, en este orden.

### Terminal Jetson #1 — micro-ROS agent

Hace de puente entre el firmware (Hackerboard, micro-ROS) y ROS 2. **Sin
esto, `/cmd_vel` no mueve los motores**.

```bash
ssh puzzlebot@10.42.0.1
ros2 launch puzzlebot_ros micro_ros_agent.launch.py
```

Espera ver `Agent up` o similar. Deja esta terminal abierta.

### Terminal Jetson #2 — cámara + ArUco

```bash
ssh puzzlebot@10.42.0.1
ros2 launch puzzlebot_ros aruco_jetson.launch.py
```

Espera ver:

```
[marker_publisher-3] Successfully setup the marker publisher!
```

### Verificación rápida (Jetson, tercera terminal SSH)

```bash
ros2 topic info /cmd_vel
# Subscription count: debe ser >= 1 (el firmware esta escuchando)

ros2 topic hz /marker_publisher/markers
# con un marker enfrente: ~25-30 Hz; sin marker: nada
```

Si las dos verifican → robot listo.

---

## 3. Verificar conexión desde la PC

```bash
source ~/mc6_real_env.sh

# Debes ver todos estos topics expuestos por el Jetson:
ros2 topic list | grep -E 'cmd_vel|marker_publisher|VelocityEnc|scan|video_source'
```

Si falta alguno → revisa Phase 0 del Jetson.

Si ves todo: estás listo para los demos.

---

## 4. Escena 1 — EKF + Teleop (Part 1)

> **Lo que vas a demostrar**: que el EKF corrige la pose con ArUco. Visualmente: la elipsoide de covarianza CRECE cuando no ves markers y ENCOGE cuando los ves. Eso es Part 1 del PDF.

### 4.1 Pega 1 marker en posición estratégica

Cualquiera de tus 8 markers, en su lugar de la pared. Pero que puedas **taparlo con la mano** durante el demo cuando quieras.

### 4.2 Lanza el stack del EKF (sin navegación)

**Terminal PC #1**:

```bash
source ~/mc6_real_env.sh
ros2 launch puzzlebot_sim real_robot_launch.py enable_navigation:=false
```

Espera estos 3 logs (todos juntos confirman que todo arrancó bien):

```
[ekf_localisation-3] EKF iniciado: r=0.05, L=0.19, dt=0.020s,
                     init=(1.30,0.50,3.14),
                     markers=[70, 706, 75, 701, 703, 705, 708, 702]
[aruco_ros_bridge-6] aruco_ros_bridge: /marker_publisher/markers -> /aruco_detections
                     (... filtrando a ids [70, 75, 701, 702, 703, 705, 706, 708])
[covariance_visualizer-N] CovarianceVisualizer: sigma_scale=2.0
```

RViz se abre con estos displays activos:

- **RobotModel** — modelo 3D del Puzzlebot
- **EKF Odometry** — flechas azules con la trayectoria estimada
- **Covariance Ellipse** — cilindro azul semi-transparente, el ELIPSOIDE
- **LaserScan** — puntos del LiDAR
- **ArUco Debug (real)** — imagen de la cámara con los markers marcados con cuadros verdes (topic `/marker_publisher/result`)

### 4.3 Empieza el bag

**Terminal PC #2**:

```bash
source ~/mc6_real_env.sh
ros2 bag record -o partA_ekf_teleop_$(date +%H%M%S) \
    /odom /aruco_detections /marker_publisher/markers \
    /marker_publisher/result \
    /cmd_vel /scan /pose_covariance_marker /tf /tf_static
```

### 4.4 Teleop

**Terminal PC #3**:

```bash
source ~/mc6_real_env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Al abrirse, **presiona `q` unas 8-10 veces** para subir la velocidad a
~0.12 m/s (el deadband del motor real no deja arrancar por abajo de 0.08).

### 4.5 Secuencia de 5 movimientos a grabar

| Tiempo | Acción FÍSICA | Lo que ves en RViz |
|---|---|---|
| **0:00 – 0:20** | Robot quieto, cámara apuntando a **una pared sin markers** | Cilindro azul **crece lentamente** (ruido del modelo) |
| **0:20 – 0:50** | Avanzar ~50 cm con teleop, sin markers a la vista | Cilindro **crece más rápido** (predict acumula error de encoders) |
| **0:50 – 1:10** | Gira el robot hasta que la cámara apunte al marker físico | En **ArUco Debug** ves el marker con cuadro verde + ID. Cilindro **encoge bruscamente** (corrección) |
| **1:10 – 1:30** | Tapa el marker con la mano | Cuadro verde desaparece. Cilindro **vuelve a crecer** |
| **1:30 – 1:50** | Destapa el marker | Cuadro verde reaparece. Cilindro **encoge otra vez** |

**Esto es Part 1**. Esos 5 cambios visibles del cilindro son la evidencia.

### 4.6 Apaga la grabación

```bash
# Ctrl+C en teleop
# Ctrl+C en ros2 bag record
# Deja el launch corriendo para Escena 2
```

Verifica que el bag se guardó:

```bash
ls -lh partA_ekf_teleop_*
ros2 bag info partA_ekf_teleop_HHMMSS
```

Debe pesar varios MB.

---

## 5. Día de evaluación — Navegación interactiva con RViz

> **Lo que vas a demostrar**: el profesor te dicta una pose inicial y waypoints. Tú **NO** editas yaml — clickeas con RViz directamente en el mapa. El robot navega con Bug 0 reactivo + EKF y muestras cómo la covarianza se reduce cuando ve ArUcos.

### Flujo "click-y-anda" usando RViz

| Acción | Herramienta de RViz | Topic publicado | Quién lo recibe |
|---|---|---|---|
| Marcar dónde está el robot al inicio | **2D Pose Estimate** | `/initialpose` | EKF (resetea su pose) |
| Dar el siguiente goal | **2D Goal Pose** | `/goal_pose` | point_generator → multi_point_nav |

Las herramientas están **en la barra superior de RViz** (los botones azules con flecha y cuadradito).

### 5.1 Antes de empezar (una sola vez)

Edita `real_robot_params.yaml` para asegurar el modo correcto:

```yaml
point_generator:
  ros__parameters:
    manual_advance: false      # NO necesario en modo interactivo
    loop_trajectory: false     # NO loop
    waypoints_x: [0.0]         # placeholder; los goals los das con RViz
    waypoints_y: [0.0]
```

**No necesitas cambiarlo cada vez** — el modo interactivo ignora la lista. Lo que importa es que `manual_advance` esté en `false` para que los goals de RViz no se queden esperando una señal manual extra.

### 5.2 Flujo recomendado el día del demo (modo interactivo con RViz)

**Terminal PC #1** — Lanza el stack completo:

```bash
source ~/mc6_real_env.sh
ros2 launch puzzlebot_sim real_robot_launch.py
```

Espera el log:
```
[map_server-N] Configuring
[map_server-N] Activating
```

→ RViz se abre con:
- El **mapa del laberinto** dibujado en gris/negro/blanco (paredes negras, libre gris)
- El robot model en el origen
- LaserScan, EKF Odometry, Covariance Ellipse, ArUco Debug

**Terminal PC #2** — bag para el video / evidencia:

```bash
ros2 bag record -o eval_$(date +%H%M%S) \
    /odom /aruco_detections /marker_publisher/markers /marker_publisher/result \
    /cmd_vel /pre_cmd_vel /scan /current_goal /planned_path \
    /pose_covariance_marker /initialpose /goal_pose /tf /tf_static
```

**En RViz, durante la evaluación:**

1. **Pose inicial** — click en **"2D Pose Estimate"** en la barra de herramientas de RViz, click sobre el mapa donde físicamente está el robot, arrastra para apuntar a +X y suelta.
   - En la terminal del launch verás: `EKF reset por /initialpose: x=0.00 y=0.00 theta=0.0deg`

2. **Primer goal** — click en **"2D Goal Pose"**, click sobre el mapa en el waypoint que dictó el profesor, arrastra y suelta.
   - El log dice: `Nuevo goal recibido de RViz (2D Goal Pose): (X.XX, Y.YY)`
   - El robot empieza a moverse
   - En RViz ves la elipse encogerse cada vez que cruza un ArUco

3. **Robot llega** — `multi_point_nav` publica `/goal_reached`. El robot se detiene.

4. **Profesor mide error con cinta** vs tu posición declarada en RViz.

5. **Profesor dice "siguiente"** — click en "2D Goal Pose" otra vez con el siguiente punto.

6. Repite hasta terminar todos los waypoints.

7. Ctrl+C en el bag cuando termines.

### Ventajas de este flujo vs el modo "lista del yaml"

- No editas yaml en vivo (menos errores)
- Los profesores pueden dictar puntos sobre la marcha; si cambian de idea, tú clickeas otra vez
- Visual: ven el mapa, el robot, la elipse, el goal en una sola ventana
- Mucho más rápido y menos estresante

### 5.3 Antes de prender — modo lista alternativo (si prefieres editar yaml)

Los profesores te dan en el momento:
- Posición inicial (típicamente (0, 0) esquina inferior-izquierda)
- 4-5 waypoints (x, y)

Edita `~/ros2_ws/src/MiniChallenge6/MiniChallenge6/puzzlebot_sim/config/real_robot_params.yaml`:

```yaml
ekf_localisation:
  ros__parameters:
    # ...
    initial_x:     0.0       # poner lo que dicen los profes
    initial_y:     0.0
    initial_theta: 0.0       # 0 = mirando +X

point_generator:
  ros__parameters:
    waypoints_x: [X1, X2, X3, X4, X5]      # poner los X de los profes
    waypoints_y: [Y1, Y2, Y3, Y4, Y5]      # poner los Y
    loop_trajectory: false                  # NO loop para evaluacion
    manual_advance: true                    # SI pausa por waypoint
    startup_delay: 3.0
    frame_id: odom
```

Guarda. **No necesitas rebuild** si usaste `--symlink-install` (sí lo usamos).

### 5.2 Posiciona el robot físicamente en (0, 0) mirando +X

La esquina inferior-izquierda del laberinto, alineado con +X (la dirección que va hacia adentro del laberinto). Mide con cinta para que esté en su pose inicial exacta.

### 5.3 Lanza el stack completo

**Terminal PC #1**:

```bash
source ~/mc6_real_env.sh
ros2 launch puzzlebot_sim real_robot_launch.py
```

Espera estos logs:

```
[point_generator-N]    PointGenerator iniciado con 5 waypoints, loop=False.
[multi_point_nav-N]    MultiPointNav iniciado: tol=0.15 m, vmax=0.1, ...
[obstacle_avoidance-N] ObstacleAvoidance iniciado: obs_dist=0.22 m, ...
```

### 5.4 Bag de la evaluación

**Terminal PC #2**:

```bash
source ~/mc6_real_env.sh
ros2 bag record -o eval_$(date +%H%M%S) \
    /odom /aruco_detections /marker_publisher/markers /marker_publisher/result \
    /cmd_vel /pre_cmd_vel /scan /current_goal /planned_path \
    /pose_covariance_marker /next_waypoint /tf /tf_static
```

### 5.5 Flujo durante la evaluación

```
Robot arranca → va al WP1
                  ↓
            ¿Llegó (dist < 15 cm del goal)?
                  ↓ sí
            Robot SE DETIENE
            Log: "WP0 alcanzado. Esperando senal manual del profesor."
                  ↓
            Profesor mide error con cinta
            Profesor dice "siguiente"
                  ↓
**Terminal PC #3 (TÚ)**:
            ros2 topic pub --once /next_waypoint std_msgs/msg/Empty "{}"
                  ↓
            Robot va al WP2
                  ↓
            ... (repite por cada waypoint) ...
                  ↓
            Después del último WP:
            Log: "Trayectoria completada."
            Robot queda parado.
```

### 5.6 Comando para avanzar al siguiente waypoint

Cada vez que el profesor te diga "siguiente":

```bash
# En la Terminal PC #3 (mantenla abierta):
source ~/mc6_real_env.sh
ros2 topic pub --once /next_waypoint std_msgs/msg/Empty "{}"
```

Para ahorrar tecla, pon esto en un alias antes de empezar:

```bash
alias next='ros2 topic pub --once /next_waypoint std_msgs/msg/Empty "{}"'
# Luego solo escribes "next" cada vez que tengas que avanzar
```

### 5.7 Si se atora antes de llegar a un waypoint

- El obstacle_avoidance debe sacarlo solo (Bug 0 reactivo)
- Si lleva >30 s atorado: cuenta como **colisión** (si no lo movieron tú)
- Máximo 3 colisiones, -5 pts cada una
- A las 4 colisiones: descalificación

### 5.8 Termina la grabación

```bash
# Ctrl+C en bag (Terminal PC #2)
# Ctrl+C en launch (Terminal PC #1)
ls -lh eval_*
```

---

## 6. Escena 3 — Tests de robustez (opcional)

El PDF lo pide explícitamente:

> *"The student must test their localisation library in different scenarios, such as multiple marker observation, no marker observation, partial marker observation, etc. to prove its robustness."*

Mismo launch que Escena 1 (`enable_navigation:=false`) + teleop. Maniobras:

| Escenario | Maniobra | Lo que se ve |
|---|---|---|
| **No marker visibility** | Apuntar la cámara al piso/techo durante 1 min | Cilindro crece sin parar (predict puro) |
| **Multiple markers** | Posicionar la cámara en un cruce donde se vean 2-3 markers a la vez | Cilindro colapsa muy rápido |
| **Partial occlusion** | Tapar el marker hasta la mitad con la mano | aruco_ros aún detecta (con más ruido) → corrección menos agresiva |
| **Marker re-acquisition** | Tapar el marker 30 s, destapar | Salto visible en la trayectoria del odom (corrige drift acumulado) |

Bag aparte: `partC_robustness_*`.

---

## 7. Reproducir bags para grabar el video

Para grabar la pantalla con OBS sin necesidad del robot físico:

```bash
# Terminal 1 (PC): solo RViz + EKF, sin Jetson, sin nav
source ~/mc6_real_env.sh
ros2 launch puzzlebot_sim real_robot_launch.py enable_navigation:=false

# Terminal 2 (PC): reproduce el bag (a la mitad de velocidad)
source ~/mc6_real_env.sh
ros2 bag play partA_ekf_teleop_HHMMSS --rate 0.5
```

Eso te permite pausar, hacer zoom, ajustar cámara en RViz, mientras una
herramienta como **OBS Studio** o **SimpleScreenRecorder** captura todo.

---

## 8. Estructura del video (3-4 min, en inglés)

| min | Contenido | Material |
|---|---|---|
| 0:00 – 0:25 | **Intro + diagrama** del stack (sección 0 de este README) | Slide |
| 0:25 – 1:10 | **What is the Kalman Filter** en tus palabras | Ver guion abajo |
| 1:10 – 2:10 | **Demo Part 1**: reproducción del bag `partA` con maniobras 1-5 | Screen capture de RViz |
| 2:10 – 2:50 | **Demo robustez**: reproducción del bag `partC` | Misma vista |
| 2:50 – 3:30 | **Demo Part 2**: reproducción del bag `partB`, vuelta completa al loop | Misma vista + cámara externa del robot moviéndose |
| 3:30 – 4:00 | **Retos enfrentados** + métricas | Slide final |

### Guion sugerido para "What is the Kalman Filter"

> "The Extended Kalman Filter is a recursive Bayesian estimator for non-linear
> systems. Each iteration has two steps:
>
> **Predict** — using the motion model (in our case, differential drive
> kinematics from wheel encoders), we propagate the state and the
> covariance: `x_k = f(x_{k-1}, u_k)` and `P_k = F P F^T + Q`, where F is
> the Jacobian of the motion model and Q is the process noise.
>
> **Correct** — when we observe an ArUco marker whose position is in our
> map, we compute the expected observation `h(x_k)` and the innovation
> `y = z - h(x_k)`. We then compute the Kalman gain
> `K = P H^T (H P H^T + R)^-1`, update the state `x_k += K y` and shrink
> the covariance `P = (I - K H) P`. H is the Jacobian of the observation
> model.
>
> In our implementation, the state is the 2D pose (x, y, theta) of the
> robot. The observation is the 2D position of each ArUco marker in the
> robot's base frame, transformed there by aruco_ros on the Jetson. The
> covariance ellipsoid in RViz visualizes the 2x2 (x, y) block of P in
> real time: when we don't see markers, Q grows it; when we observe a
> marker, the correct step shrinks it."

### "Reasoning behind every engineering decision" (slide bullets)

- **Detección ArUco en Jetson, EKF en PC** — minimiza ancho de banda WiFi.
- **observation_frame=base** — aruco_ros ya hace la TF.
- **allowed_ids filter** — drop de falsos positivos como el ID 249 sobre el 702.
- **`sigma_scale=2`** — la elipse representa 2-sigma (~95% confianza), no 1-sigma.
- **`aruco_max_distance=1.5 m`** — markers de 10 cm a 320×240 dan pose ruidosa más allá de eso.

---

## 9. Troubleshooting

| Síntoma | Causa probable | Fix |
|---|---|---|
| `ros2 topic info /cmd_vel` dice `Subscription count: 0` | `micro_ros_agent` no está corriendo | Lanzar `puzzlebot_ros micro_ros_agent.launch.py` en Jetson |
| Robot no se mueve aunque hay `/cmd_vel` | Velocidad bajo deadband del motor | En teleop, presionar `q` hasta `linear.x ≥ 0.10` |
| `/marker_publisher/markers` vacío con marker enfrente | Mismatch de diccionario | En `aruco_jetson.launch.py` añadir `{"dictionary": "ARUCO"}` |
| Elipsoide no encoge cuando ve marker | Bridge crashó o ID no está en `marker_ids` | Revisar log de `aruco_ros_bridge`; ver `allowed_ids` |
| `/aruco_detections` no se publica | `aruco_ros_bridge` no se inició | Revisar `BYTE_ARRAY` error en log (ya fixed) |
| RobotModel en RViz se ve con llanta separada | Falta `/joint_states` | El launch ya incluye `joint_state_publisher` (default zeros) |
| Robot va al revés en nav autónoma | `initial_theta` no coincide con orientación física | Verificar que el robot físicamente mire a la izquierda al prender |
| Cámara se cuelga después de N min | nvargus daemon stuck | En Jetson: `sudo systemctl restart nvargus-daemon` |
| Falso positivo de ID 249 encima de 702 | aruco_ros confunde patrón con desenfoque | Ya filtrado en `aruco_ros_bridge.allowed_ids` |
| `topic hz /video_source/raw` da 5 Hz en la PC | Es WiFi, no la cámara | Verificar en el JETSON con `ros2 topic hz /video_source/raw` → ~30 Hz |
| Image `/marker_publisher/result` lenta en RViz | Es la imagen viajando por WiFi | Esperado; usar para validación, no requiere ser fluido |

---

## 10. Mapeo de requisitos del PDF al código

| Requisito del PDF | Implementación | Archivo |
|---|---|---|
| Camera-based EKF | EKF con predict + correct | [`ekf_localisation.py`](puzzlebot_sim/ekf_localisation.py) |
| ArUco identification | Detección en Jetson via aruco_ros | OneNote / `aruco_jetson.launch.py` |
| Vision algorithms for ArUco | aruco_ros usa OpenCV `cv2.aruco` | upstream |
| Coordinate transformations | `observation_frame=base` + `camera_x/y/yaw` | [`ekf_localisation.py:correct_with_marker`](puzzlebot_sim/ekf_localisation.py) |
| Camera parameter identification | `jetson_cam.yaml` cargado por `camera_info_publisher` | Jetson |
| Adequately identify observation model | Docstring + función `correct_with_marker` | [`ekf_localisation.py`](puzzlebot_sim/ekf_localisation.py) |
| Multiple marker observation | EKF itera sobre `msg.detections` | [`ekf_localisation.py:aruco_cb`](puzzlebot_sim/ekf_localisation.py) |
| No marker observation | Predict-only step crece la covarianza | Mismo |
| Partial marker observation | aruco_ros lo maneja; nuestro EKF acepta cualquier detección válida | Bridge |
| ArUco dictionary | Original ArUco (configurable) | `aruco_jetson.launch.py` |
| Plot confidence ellipsoids in RVIZ | Cilindro 2-sigma con eigendecomp | [`covariance_visualizer.py`](puzzlebot_sim/covariance_visualizer.py) |
| Evaluation metrics | Bag → plot determinante de covarianza vs tiempo (script aparte) | TODO |
| Solo NumPy + OpenCV + libs de matrices | Cumple — solo math/numpy/cv2/rclpy | Todo |
| Trayectoria cerrada ≥4 waypoints | 5 waypoints en `point_generator` | [`real_robot_params.yaml`](config/real_robot_params.yaml) |
| Sin rectas paralelas | Cada tramo direcciones distintas | Mismo |
| Circumnavigate obstáculos | Bug 0 reactivo en `obstacle_avoidance` | [`obstacle_avoidance.py`](puzzlebot_sim/obstacle_avoidance.py) |
| Launch files definidos | `real_robot_launch.py` | [`launch/real_robot_launch.py`](launch/real_robot_launch.py) |
| Sampling time correcto | 50 Hz EKF, 20 Hz nav/obstacle | yaml params |

---

## 11. Preguntas técnicas que pueden hacer los profesores

El PDF dice: *"Durante la demostración, los profesores podrán realizar
preguntas técnicas individuales relacionadas con: Arquitectura del sistema
en ROS 2, Implementación del algoritmo de navegación, Uso de sensores,
Localización y Filtro de Kalman, Comunicación entre nodos, Visualización en
RViz, Estrategias de control y evasión de obstáculos."*

Respuestas mínimas que TODO el equipo debe poder dar:

### Arquitectura del sistema en ROS 2

> "Tenemos dos máquinas: el Jetson del Puzzlebot corre `aruco_jetson.launch`
> (cámara CSI + detector aruco_ros) y `micro_ros_agent` (bridge al firmware
> de la Hackerboard). La PC corre `real_robot_launch.py` que levanta el
> EKF, multi_point_nav, obstacle_avoidance y RViz. La comunicación es por
> DDS sobre WiFi: solo viajan `Pose` messages (no imágenes), por eso no se
> satura."

### Implementación del algoritmo de navegación (Bug 0)

> "Usamos Bug 0 reactivo separado en dos capas: `multi_point_nav` calcula
> el comando `/pre_cmd_vel` para ir directo al waypoint (controlador P
> sobre distancia y ángulo). `obstacle_avoidance` lee `/pre_cmd_vel` y
> `/scan`. Si el cono frontal (±10°) está libre, pasa el comando tal cual a
> `/cmd_vel`. Si detecta un obstáculo a <22 cm, entra a modo FOLLOW_WALL:
> sigue la pared izquierda a 18 cm con un P. Cuando el frente vuelve a
> estar libre y la pared izquierda se 'pierde', regresa a GO_TO_GOAL."

### Uso de sensores

> "Encoders en cada rueda publican `/VelocityEncL,R` a 50 Hz (Float32).
> LiDAR RPLidar publica `/scan` a 10 Hz (sensor_msgs/LaserScan). Cámara CSI
> a 30 Hz publicada en el Jetson como `/video_source/raw` (no la usamos en
> la PC, queda local). La detección ArUco corre en el Jetson, viaja
> `/marker_publisher/markers` (aruco_msgs/MarkerArray)."

### Localización y Filtro de Kalman

> "Estado: 2D pose `(x, y, θ)` con covarianza 3×3. **Predict** desde
> encoders: integra la cinemática diferencial (`v = r(wr+wl)/2`,
> `ω = r(wr-wl)/L`) y propaga `Σ = F Σ F^T + Q`, donde Q viene del ruido
> por rueda mapeado al estado por el Jacobiano `grad_w`. **Correct** por
> cada ArUco visto cuyo ID está en nuestro mapa: convierte la pose
> (`base_link` frame, ya transformada por aruco_ros en el Jetson) a una
> medición 2D, calcula la innovación `y = z - h(x)` y la ganancia
> `K = P H^T (H P H^T + R)^-1`. Actualiza estado `x += K y` y
> `P = (I - K H) P`."

### Comunicación entre nodos

> "Todo es DDS (Fast RTPS por default). Topics principales:
> - `/cmd_vel` (PC → Jetson → firmware)
> - `/odom` (EKF en PC, publica con `pose.covariance` no-cero)
> - `/aruco_detections` (bridge convierte de aruco_msgs a nuestro tipo)
> - `/pose_covariance_marker` (visualizador → RViz)
> - `/next_waypoint` (señal manual durante evaluación)"
> Subscribers QoS: RELIABLE + KEEP_LAST(10) para mensajes de control.

### Visualización en RViz

> "RobotModel del URDF, TF (chain `map → odom → base_footprint`),
> LaserScan, Odometry con covariance habilitada, MarkerArray para el
> cilindro 2σ (eigendescomposición del bloque 2×2 de Σ), Path del
> planned_path, Image del topic `/marker_publisher/result` (cámara con
> markers resaltados)."

### Estrategias de control y evasión de obstáculos

> "Controlador P por separado para linear y angular en `multi_point_nav`:
> si `|ang| > 20°` gira en sitio (v=0); si no, `v = k_lin * dist` y
> `w = k_ang * ang`, ambos saturados. Para evitar obstáculos, el cono
> frontal se mide del LiDAR (sector `±10°` del frame del LiDAR). El
> wall-follow usa P sobre el error `(left_dist - target)`, con histéresis
> al salir (front_clear * 1.3 + wall_lost) para evitar oscilación entre
> los dos estados."

---

## Apéndice — Comandos cheat sheet

```bash
# === SETUP ===
source ~/mc6_real_env.sh                            # PC: stack del Final Challenge (real robot)
source ~/mc6_env.sh                                 # PC: simulación (Gazebo)

# === EN EL JETSON ===
ssh puzzlebot@10.42.0.1
ros2 launch puzzlebot_ros micro_ros_agent.launch.py # firmware bridge
ros2 launch puzzlebot_ros aruco_jetson.launch.py    # camara + aruco_ros

# === EN LA PC ===
# Solo EKF + teleop (Escena 1, demo Part 1)
ros2 launch puzzlebot_sim real_robot_launch.py enable_navigation:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Stack completo con nav autonoma (Escena 2, demo Part 2)
ros2 launch puzzlebot_sim real_robot_launch.py

# === EVALUACION FINAL: avanzar al siguiente waypoint cuando el profe diga ===
ros2 topic pub --once /next_waypoint std_msgs/msg/Empty "{}"
# o con alias:
alias next='ros2 topic pub --once /next_waypoint std_msgs/msg/Empty "{}"'

# === GRABACION ===
ros2 bag record -o NOMBRE \
    /odom /aruco_detections /marker_publisher/markers /marker_publisher/result \
    /cmd_vel /pre_cmd_vel /scan /current_goal /planned_path \
    /pose_covariance_marker /tf /tf_static

# === REPRODUCCION (para grabar video) ===
ros2 bag play NOMBRE --rate 0.5
ros2 bag info NOMBRE

# === DIAGNOSTICO ===
ros2 topic list | grep -E 'cmd_vel|marker|scan|odom'
ros2 topic hz /odom                  # ~50 Hz
ros2 topic hz /marker_publisher/markers  # ~25-30 Hz cuando ve marker
ros2 topic echo /aruco_detections --once
ros2 topic echo /odom --field pose.covariance --once
ros2 node info /ekf_localisation
```

# Auditoría de valores hardcodeados (rama Grant)

Barrido completo del paquete `puzzlebot_sim` para evitar que un valor fijo en el
código rompa el uso del robot. Clasificado por riesgo. **"Configurable"** = se
cambia por YAML o launch arg sin tocar código.

## ✅ Arreglado en este commit — antes hardcodeado, ahora configurable

| Qué | Dónde estaba | Ahora |
|---|---|---|
| **Topics de encoders** (`/VelocityEncR/L`) | remap fijo en `real_robot_launch.py` | launch args `wr_topic` / `wl_topic` |
| **Topic `/cmd_vel`** | nombre fijo del nodo bug2 | launch arg `cmd_vel_topic` (remap) |
| **Topic `/scan`** | nombre fijo del nodo bug2 | launch arg `scan_topic` (remap) |
| **Nombre del xacro** (`puzzlebot_jetson_lidar_ed`) | f-string fijo en el launch | launch arg `robot_name` |

Ejemplo (también sirve para **corregir el giro invertido** cruzando ruedas):
```bash
ros2 launch puzzlebot_sim real_robot_launch.py \
    wr_topic:=/VelocityEncL wl_topic:=/VelocityEncR \
    cmd_vel_topic:=/cmd_vel scan_topic:=/scan
```

> Ya configurable desde la sesión anterior (bloque `bug2` del YAML):
> `safety_stop_distance`, `safety_cone_deg`, `critical_distance`,
> `lidar_yaw_offset`, `trust_odom`.

## 🟢 Ya eran configurables (verificar el valor por corrida, no es bug)

- **EKF** (`config/real_robot_params.yaml`): `initial_x/y/theta`, `kr/kl`,
  `camera_x/y/yaw`, mapa de markers (`marker_ids/xs/ys/yaws`), `aruco_*`.
- **point_generator**: `waypoints_x/y`, `loop_trajectory`, `interactive_mode`,
  `manual_advance`, `frame_id`.
- **aruco_ros_bridge**: `scale_correction`, `allowed_ids`, `input/output_topic`.
- **launch args**: `params_file`, `map_yaml`, `rviz_config`, `use_rviz`,
  `use_sim_time`, `enable_navigation`.

## ⚠️ La "trampa silenciosa": defaults del nodo que GANAN si falta el YAML

Cada nodo declara defaults en `declare_parameter(...)` que **difieren** del YAML.
Si se lanza **sin** `params_file`, o con una key **mal escrita** en el YAML, el
nodo usa su default hardcodeado y parece que "no hizo caso" a la config.

| Nodo | Default del .py | Valor real (YAML) |
|---|---|---|
| `bug2` | `obstacle_distance=0.25`, `forward_cone_deg=30`, `wall_target=0.20` | `0.30`, `40`, `0.27` |
| `point_generator` | `waypoints=[1,1,0,0]/[0,1,1,0]`, `loop_trajectory=True` | tu lista, `false` |
| `ekf_localisation` | `initial=(0,0,0)`, `marker_ids=[0]` | `(0.325,-0.28,0)`, tu mapa |

**Mitigación:** `real_robot_launch.py` SIEMPRE pasa `params_file`; no renombres
keys del YAML. Si dudas, verifica en vivo: `ros2 param get /bug2 obstacle_distance`.

## 🔧 Hardcodes de entorno que quedan (revisar según tu setup, no críticos para nav)

- **IP de la Jetson `10.42.0.1`** y `ROS_DOMAIN_ID` en
  `config/dds/cyclonedds_unicast.xml` y `scripts/mc6_real_env.sh`. Cámbialos si
  la red/AP cambia (documentado en ambos archivos).
- **TF estática `laser_frame → laser`** en el launch: el LiDAR real publica en
  el frame `lidar_link`. **No afecta a la navegación** (bug2 lee `/scan` crudo,
  sin TF); solo a la alineación del LaserScan en RViz. Si quieres que RViz
  alinee, cambia el arg a `lidar_link` o ajusta el URDF.
- **`frame_id='map'`** del cilindro de meta en `bug2.py`/`bug0.py`: cosmético
  (marcador en RViz).

## 🟡 Constantes de tuning en .py (intencionales, bajo riesgo) — opcional a futuro

No bloquean el uso del robot; son la "personalidad" del control. Se pueden
volver parámetros más adelante si hace falta afinar fino:
- `bug2.py`: multiplicadores (`*0.5`, `*0.8`, `*2.0`, `*1.5`…), sectores
  fronto-laterales `front_left/right` (`-50°..-5°`, `5°..50°`),
  `_leave_cooldown=40`, umbral `goal_path_clear` (60°).
- `obstacle_avoidance.py` / `multi_point_nav.py`: solo se usan en
  `final_challenge_launch.py` (sim), no en el robot real.

## Cómo verificar en vivo que NADA quedó en su default por error
```bash
ros2 param dump /bug2                 # vuelca todos los params efectivos del nodo
ros2 param get /bug2 trust_odom       # debe ser False (modo reactivo)
ros2 param get /ekf_localisation initial_x
ros2 node info /bug2                  # confirma a que topics quedo conectado (remaps)
```

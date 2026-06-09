# Auditoría y cambios — Repositorio Grant

Resultado de la auditoría profunda del repo PC-side del Puzzlebot, **verificado
contra el código real** (varios "críticos" del auditor automático resultaron
exagerados o peligrosos de aplicar a ciegas). Ordenado por: lo aplicado, lo que
necesita tu confirmación, y lo que es Jetson/hardware.

> Diagnóstico del problema de micro-ROS → ver
> [`MICROROS_TROUBLESHOOTING.md`](MICROROS_TROUBLESHOOTING.md).
> **Resumen:** micro-ROS no se cae; se *congela bajo carga*. Causa dominante =
> **brownout (batería compartida) + contención de CPU en la Jetson (aruco + agent)
> + serial**. Eso **no vive en este repo**. Lo del repo que sí ayuda es el
> **discovery DDS** ("no ve los nodos") y limpieza de bugs.

---

## ✅ Cambios aplicados (seguros y aditivos)

| # | Cambio | Archivo | Por qué |
|---|---|---|---|
| 1 | **Perfil DDS unicast (CycloneDDS)** | `config/dds/cyclonedds_unicast.xml` (nuevo) | Discovery sin multicast → arregla "no ve los nodos" sobre el WiFi de la Jetson. |
| 2 | **Script de entorno versionado** | `scripts/mc6_real_env.sh` (nuevo) | Antes vivía solo en el HOME del usuario (no reproducible). Fija `ROS_DOMAIN_ID`, `RMW=cyclonedds`, `CYCLONEDDS_URI`. |
| 3 | **Bug: bridge mutaba el msg de DDS in-place** | `aruco_ros_bridge.py` | `d.pose = m.pose.pose` aliasaba el buffer de entrada; al escalar la posición se corrompía el mensaje original. Ahora `copy.deepcopy`. |
| 4 | **Dep faltante `aruco_msgs`** | `package.xml` | El bridge la importa en runtime y NO está instalada en la PC. (`sudo apt install ros-humble-aruco-msgs`). |
| 5 | **Dep faltante `joint_state_publisher`** | `package.xml` | El launch usa el no-gui; el package.xml solo declaraba `_gui`. |
| 6 | **Rename `bug0`→`nav_controller`** | `real_robot_launch.py` | La variable se llamaba `bug0` pero lanza `bug2`; inducía a error al editar. Cosmético. |

> Nota: el robot real corre el nodo **monolítico `bug2`** (go-to-goal + evasión),
> NO la arquitectura de 2 capas (`multi_point_nav` + `obstacle_avoidance`) que
> describe el README. Funciona, pero **el diagrama del README no coincide con el
> grafo real** — tenlo en cuenta si un profesor pregunta.

---

## 🟡 Revisado contigo — DOCUMENTADO pero NO aplicado (tu decisión)

> Decisión del usuario: la detección de ArUcos "funciona perfecto" con la config
> actual y prefiere revisar él mismo el tuneo del EKF/nav. Por eso **NADA de esta
> sección fue modificado**. Queda como referencia por si más adelante hay que
> afinar precisión.

### A. `scale_correction = 1.40` — DEJADO COMO ESTÁ (el usuario reporta que funciona)

`config/real_robot_params.yaml` → `aruco_ros_bridge.scale_correction: 1.40`.

El **comentario del propio archivo** dice que la cámara *sobre-reporta* la
distancia y que el factor correcto es **≈0.696–0.73** (con ratios medidos por
marker, todos ~0.68–0.72). Pero `1.40 ≈ 1/0.714` → **amplifica** el error en
vez de corregirlo: un marker a 26 cm reales que aruco_ros reporta a ~38 cm,
con 1.40 se "corrige" a **~53 cm**. El EKF cree que el marker está casi al doble
de lejos → coloca mal al robot al corregir (daña los 30 pts del KF y los 50 de
nav).

**Prueba definitiva (2 min):** pon un marker a **50 cm exactos** de la cámara y:
```bash
ros2 topic echo /aruco_detections --once    # mira pose.position.z (o .x)
```
- Si reporta **~1.0 m** → 1.40 está al doble; baja a **~0.70**.
- Si reporta **~0.5 m** → 1.40 estaba bien (deja como está).

**No se cambió.** Ojo con una distinción para tu propia revisión: "la detección
funciona" (lee bien los IDs y dibuja el cuadro) **no** es lo mismo que "la
*distancia* reportada es exacta". Si algún día el robot queda corto/largo al
corregir con un marker, la prueba de la regla de arriba lo confirma en 2 min.

### B. Robustez del EKF (no tocado — tu tuneo, lo revisas tú)

- `aruco_max_jump = 2.00` m → el clamp anti-teleport **nunca dispara** (las
  correcciones rara vez superan 2 m). Recomendado: **0.3–0.5 m**.
- `aruco_var_base = 0.001` (σ≈3 cm) → EKF **sobre-confía** en ArUco; si el
  `scale_correction` está mal, propaga el error. Recomendado tras fijar escala:
  **0.004–0.01**.
- Gate de Mahalanobis: compara `sqrt(mahal²)` contra `5.0` → es un gate de ~5σ,
  **muy laxo** pero no es bug. Opcional bajar a ~3.0.

### C. Convención angular del LiDAR (riesgo en nav)

`bug2.py` asume sector derecho = `-120°..-60°` (LiDAR en `-π..π`). Si tu RPLidar
publica `0..2π`, los sectores izquierdo/derecho quedan mal y dispararían
frenados de emergencia falsos. **Verifica:**
```bash
ros2 topic echo /scan --field angle_min --once   # ¿~ -3.14 o ~0.0?
```
Si es `~0.0`, hay que normalizar los índices en `_sector_min` (te lo aplico).

### D. `/cmd_vel` QoS — el auditor dijo "crítico", pero es FALSO/peligroso

El auditor marcó "`/cmd_vel` RELIABLE → no llega ningún comando". **Incorrecto:**
un publisher RELIABLE *sí* es compatible con un sub BEST_EFFORT. Cambiarlo a
BEST_EFFORT a ciegas podría **romper** el control si el firmware suscribe como
RELIABLE. **No cambiar** sin saber el QoS del firmware. (20 Hz de `/cmd_vel`
tampoco es "flood": es normal y el firmware suele necesitarlo para su watchdog.)

---

## 🔧 Fuera del repo — Jetson / firmware / hardware (lo más importante)

Ver detalle en `MICROROS_TROUBLESHOOTING.md`. En orden de impacto:

1. **Brownout (batería compartida):** separar la alimentación de la Jetson de la
   de los motores, o batería sana + condensador de bulk. Medir voltaje **bajo carga**.
2. **CPU de la Jetson:** aruco corre SIEMPRE junto al agent → bajar framerate de
   cámara, `nvpmodel -m 0` + `jetson_clocks`, dar prioridad/`taskset` al agent.
3. **Serial:** subir baud a **921600** (firmware + agent), encoders en BEST_EFFORT.
4. **Discovery:** mismo `ROS_DOMAIN_ID` y `RMW` en PC y Jetson + perfil unicast
   (este repo ya trae el de la PC; replicar en la Jetson con la IP de la PC).

---

## Hallazgos descartados / sobrevalorados por el auditor automático

- `/cmd_vel` RELIABLE "no llega" → falso (QoS compatible). Ver D.
- EKF TF/odom a 50 Hz "compite con micro-ROS por WiFi" → falso: `/odom` y TF son
  tráfico PC↔PC (RViz local), no van a la Jetson.
- Empaquetado de covarianza 3×3→6×6, división por L, wrap de θ → revisados, **sin bug**.
- Cross-check de params de `bug2` → todos los del YAML coinciden con el nodo (OK).

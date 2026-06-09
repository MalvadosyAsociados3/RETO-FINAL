# micro-ROS se congela y "no ve los nodos" — Diagnóstico y arreglos

> Documento generado para el setup REAL del equipo (Reto Final, rama Grant).
>
> **Síntoma reportado:** micro-ROS *no se cae*, pero los tópicos **se congelan /
> llegan lentos** y el proceso del robot se alenta, causando conflictos. Ocurre
> sobre todo **al mover el robot** y **con batería baja** (también de forma
> impredecible).
>
> **Setup confirmado:**
> - Transporte micro-ROS: **serial USB** entre Hackerboard (firmware) y Jetson.
> - PC ↔ Jetson: **WiFi** (DDS).
> - **Jetson, Hackerboard y motores comparten la misma batería** (LiPo verde).
> - **aruco_ros (cámara CSI + detección) corre SIEMPRE en la Jetson** durante la nav.
> - Baud rate del serial micro-ROS: **desconocido**.

## TL;DR — qué es y qué no es

Esto **NO** es un crash del `micro_ros_agent`. Es un **estancamiento del stream
serial de micro-ROS bajo carga**. micro-ROS (XRCE-DDS) usa por defecto streams
*reliable*: cuando se pierde o se retrasa un paquete (por ruido eléctrico, caída
de voltaje, o porque la CPU no atendió el puerto a tiempo), **retransmite y
bloquea todo el flujo** hasta recuperar el orden. Desde afuera se ve idéntico a
"los tópicos se congelan y el robot se pone lento".

Hay **4 causas que se suman**. Atácalas en este orden de impacto:

| # | Causa | Probabilidad en tu setup | Dónde se arregla |
|---|---|---|---|
| 1 | **Brownout** (batería compartida, voltaje cae al mover motores) | **Alta** (al mover + batería baja) | Hardware |
| 2 | **Contención de CPU en la Jetson** (aruco + agent juntos) | **Alta** (aruco siempre ON) | Jetson |
| 3 | **Saturación del enlace serial** (baud bajo + tasas altas) | Media-Alta | Jetson + firmware + repo |
| 4 | **Discovery DDS por WiFi** ("no ve los nodos" desde la PC) | Media | PC/Jetson (red) |

---

## 1. Brownout — energía compartida (sospechoso #1, hardware)

**Por qué:** cuando los motores arrancan o frenan, jalan un pico de corriente.
Si la Jetson y la Hackerboard salen de la **misma** batería, ese pico hace caer
el voltaje del bus; el microcontrolador de la Hackerboard sufre micro-resets o
glitches en su UART → el stream serial de micro-ROS se corrompe y se congela.
Empeora con batería baja porque el voltaje ya está cerca del umbral.

**Cómo confirmarlo (5 min):**
```bash
# En la Jetson, mientras alguien mueve el robot con teleop, observa el log del agent:
ros2 launch puzzlebot_ros micro_ros_agent.launch.py    # con -v6 si se puede para ver verbose
# Si ves ráfagas de "Serial / transport error", "session restored", o pausas
# justo cuando los motores arrancan -> es brownout.

# Mide el voltaje de la batería EN MOVIMIENTO (no en reposo):
#   - Con un multímetro en las terminales, o
#   - Con la alarma de celdas LiPo ("pip" verde) puesta.
# LiPo 2S sano: ~7.4-8.4 V. Si bajo carga cae < 7.0 V (o < 3.5 V/celda) -> brownout.
```

**Arreglos (en orden):**
1. **Carga la batería / usa una con buena salud.** El síntoma "con batería baja"
   es literal: una LiPo cansada no sostiene el voltaje bajo carga.
2. **Separa la alimentación de la Jetson de la de los motores.** Idealmente la
   Jetson con su propio regulador/batería (powerbank USB-C de 5 V/4 A para Nano,
   o un BEC dedicado). Esta es la solución definitiva al brownout.
3. **Condensador de bulk** (1000–2200 µF, ≥16 V) en el bus de potencia de la
   Hackerboard para amortiguar los picos de los motores.
4. **Mide el voltaje en vivo.** Si no tienen alarma de LiPo, consigue una (las
   "pip" baratas). Nunca descargues una LiPo por debajo de 3.3 V/celda.

> ⚠️ **"Pilas verdes":** si resultan ser celdas tipo AA/NiMH y no un LiPo de
> potencia, **no entregan corriente suficiente** para Jetson + motores y el
> brownout es prácticamente garantizado. Verifica qué batería es.

---

## 2. Contención de CPU en la Jetson — aruco + agent juntos (sospechoso #1 software)

**Por qué:** la Jetson Nano tiene pocos núcleos. Correr **a la vez**:
- `video_source` (cámara CSI, nvargus),
- `aruco_ros marker_publisher` (detección OpenCV a ~30 Hz),
- `micro_ros_agent` (debe atender el puerto serial con baja latencia),

hace que el agent **no reciba tiempo de CPU a tiempo**. El buffer serial del
kernel se llena, llegan los bytes tarde, micro-ROS retransmite → **congelación**.
Esto explica "el proceso del robot se alenta" mejor que nada.

**Cómo confirmarlo:**
```bash
# En la Jetson, mientras todo corre:
htop          # mira si los núcleos están al 100% y quién consume (nvargus, aruco, agent)
tegrastats    # uso de CPU/GPU/RAM en tiempo real (NVIDIA)
# Si el agent compite por CPU al 100% -> es contención.
```

**Arreglos (en la Jetson, sin tocar este repo):**
1. **Baja la carga de la cámara/detección.** En `aruco_jetson.launch.py`:
   - Resolución ya está en 320×240 (bien); **baja el framerate** de la cámara a
     10–15 Hz (no necesitas 30 Hz para corregir el EKF).
   - Si aruco_ros lo permite, **procesa 1 de cada N frames** o limita su rate.
2. **Dale prioridad al agent.** Lánzalo con prioridad alta y fíjalo a un núcleo:
   ```bash
   # nice alto + pin a CPU 0 (ajusta el lanzamiento real del agent):
   sudo taskset -c 0 nice -n -10 ros2 run micro_ros_agent micro_ros_agent serial \
        --dev /dev/ttyUSB0 -b 921600
   ```
3. **Modo de máximo rendimiento de la Jetson** (sube los clocks de CPU):
   ```bash
   sudo nvpmodel -m 0      # modo 10W / max
   sudo jetson_clocks      # fija clocks al máximo
   ```
4. **Si el demo de Part 1 (EKF/teleop) no necesita navegar**, considera **no
   correr aruco al 100%** todo el tiempo, o correrlo en ráfagas.

---

## 3. Saturación del enlace serial — baud + tasas (Jetson + firmware + repo)

**Por qué:** por el serial suben encoders (2× `Float32` a 50 Hz) y baja
`/cmd_vel` (a 20 Hz). Si el baud es el **default 115200**, hay poco margen y
cualquier retransmisión lo tapona.

**Arreglos:**
1. **Sube el baud rate a 921600** (o más) — **debe coincidir en firmware y en el
   comando del agent**:
   ```bash
   # Agent en la Jetson:
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
   ```
   En el firmware de la Hackerboard, configura el mismo baud en la inicialización
   del transporte micro-ROS. **Primero averigua el baud actual** (revisa el
   `micro_ros_agent.launch.py` de `puzzlebot_ros` en la Jetson).
2. **Firmware: publica los encoders en BEST_EFFORT** (no reliable). Reduce
   retransmisiones por serial. (Lado firmware.)
3. **Baja la tasa de `/cmd_vel` que sale de la PC** → ver arreglos del repo
   (sección 5): el control corre a 20 Hz pero no necesita inundar el serial.
4. **Cable USB corto y con ferrita.** Reduce ruido del cableado de motores
   acoplado a la línea serial (relacionado con el brownout/EMI).

---

## 4. Discovery DDS por WiFi — "no ve los nodos" desde la PC (red)

**Por qué:** los APs WiFi suelen **descartar el tráfico multicast** que ROS 2
usa para descubrir nodos. Resultado: la PC no ve los tópicos de la Jetson aunque
todo corra. También causa lag de descubrimiento intermitente.

**Cómo confirmarlo:**
```bash
# En AMBAS máquinas:
echo "DOMAIN=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  DISTRO=$ROS_DISTRO"
# Deben COINCIDIR el ROS_DOMAIN_ID y el RMW_IMPLEMENTATION y ser el mismo distro (humble).
ros2 multicast receive    # en la PC
ros2 multicast send       # en la Jetson  -> si la PC NO recibe, el AP bloquea multicast
```

**Arreglos:**
1. **Igualá `ROS_DOMAIN_ID` y `RMW_IMPLEMENTATION`** en las dos máquinas (en tu
   `mc6_real_env.sh` y en el entorno de la Jetson). Por ejemplo:
   ```bash
   export ROS_DOMAIN_ID=0
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # o rmw_fastrtps_cpp en ambas
   ```
2. **Si el multicast está bloqueado**, usa **peers unicast estáticos**. Con
   CycloneDDS (recomendado por simplicidad), crea el XML de
   `config/dds/cyclonedds_unicast.xml` (incluido en este repo) y exporta:
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=file:///ruta/a/config/dds/cyclonedds_unicast.xml
   ```
   Ajusta las IPs (la de la Jetson 10.42.0.1 y la de la PC). Con Fast-DDS el
   equivalente se hace con `FASTRTPS_DEFAULT_PROFILES_FILE` (ver el XML alterno).
3. **Mismo distro (Humble) en ambas.** Un mismatch de distro rompe discovery.

---

## Checklist rápido de diagnóstico (ejecutar en orden)

```bash
# A) ¿Brownout? Mover el robot y mirar voltaje + log del agent.
# B) ¿CPU? htop / tegrastats en la Jetson con todo corriendo.
# C) ¿Serial? Confirmar baud actual; probar 921600.
# D) ¿Discovery? ros2 multicast send/receive entre PC y Jetson; comparar DOMAIN/RMW.
```

## Qué se arregla en ESTE repo (PC) y qué no

| Causa | Repo PC | Jetson / firmware | Hardware |
|---|---|---|---|
| Brownout | — | — | ✅ (batería, separar fuentes, capacitor) |
| CPU contención | parcial (bajar carga PC→Jetson) | ✅ (rate cámara, prioridad agent, nvpmodel) | — |
| Serial saturación | ✅ (bajar tasa /cmd_vel) | ✅ (baud, best-effort encoders) | ✅ (cable/ferrita) |
| Discovery WiFi | ✅ (perfil DDS + env) | ✅ (mismo DOMAIN/RMW) | — |

> Los arreglos de código PC-side (QoS, tasa de `/cmd_vel`, evitar doble publisher
> de `/cmd_vel`, `respawn` de nodos) se documentan en el resumen de cambios del
> repo y se aplican directamente sobre los nodos de `puzzlebot_sim`.

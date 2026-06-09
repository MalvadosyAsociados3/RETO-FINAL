#!/usr/bin/env bash
# =============================================================================
# mc6_real_env.sh — Entorno de la PC para el Final Challenge (Puzzlebot REAL).
#
#   Uso:   source mc6_real_env.sh
#
# Arregla el descubrimiento DDS sobre WiFi (el multicast suele estar bloqueado
# por el AP de la Jetson) usando CycloneDDS con peers UNICAST. Esto ataca el
# síntoma "no ve los nodos" desde la PC.
#
# Ver:  puzzlebot_sim/docs/MICROROS_TROUBLESHOOTING.md
# =============================================================================

# --- ROS 2 base ---
source /opt/ros/humble/setup.bash

# --- Workspace donde compilaste puzzlebot_sim (AJUSTA si usas otro) ---
# Exporta MC6_WS antes de hacer source para sobreescribir el default.
: "${MC6_WS:=$HOME/ros2_ws}"
if [ -f "$MC6_WS/install/setup.bash" ]; then
  source "$MC6_WS/install/setup.bash"
else
  echo "[mc6_real_env] AVISO: no existe $MC6_WS/install/setup.bash"
  echo "[mc6_real_env]   -> compila el paquete y/o ajusta MC6_WS=/ruta/a/tu_ws"
fi

# --- DDS: el ROS_DOMAIN_ID y el RMW DEBEN COINCIDIR con los de la Jetson ---
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- Perfil unicast (no depende de multicast sobre WiFi) ---
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
DDS_PROFILE="$SCRIPT_DIR/../puzzlebot_sim/config/dds/cyclonedds_unicast.xml"
if [ -f "$DDS_PROFILE" ]; then
  export CYCLONEDDS_URI="file://$DDS_PROFILE"
else
  echo "[mc6_real_env] AVISO: no encontré el perfil DDS en $DDS_PROFILE"
fi

PC_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "[mc6_real_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "[mc6_real_env] CYCLONEDDS_URI=${CYCLONEDDS_URI:-<sin perfil>}"
echo "[mc6_real_env] IP de esta PC: ${PC_IP:-?}  (la Jetson debe listarla como <Peer> y usar el MISMO domain/RMW)"
echo "[mc6_real_env] Test de discovery:  ros2 multicast receive   (PC)  /  ros2 multicast send  (Jetson)"

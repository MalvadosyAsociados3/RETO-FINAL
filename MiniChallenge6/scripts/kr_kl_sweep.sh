#!/usr/bin/env bash
# Sweep de valores kr/kl para Task 2 del Mini Challenge 5.
# Corre experiment_runner con distintos kr=kl y guarda resultados en CSV.
set -eo pipefail

WORKSPACE=~/Documents/Robotica/MiniChallenge5/Minichallenge5/MiniChallenge5
YAML="$WORKSPACE/puzzlebot_sim/config/puzzlebot_params.yaml"
YAML_BAK="$YAML.sweep_backup"
RESULTS_FILE="/tmp/kr_kl_sweep_results.csv"

# Valores a probar
VALUES=(0.01 0.02 0.05 0.1 0.2 0.5)

# Backup del YAML original
cp "$YAML" "$YAML_BAK"

# Trap para restaurar siempre el YAML, aun si falla a la mitad
cleanup() {
    echo ""
    echo "[SWEEP] Limpiando..."
    cp "$YAML_BAK" "$YAML" 2>/dev/null || true
    rm -f "$YAML_BAK"
    pkill -9 -f "ros2 launch" 2>/dev/null || true
    pkill -9 -f "puzzlebot_sim" 2>/dev/null || true
    pkill -9 -f "localisation" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
}
trap cleanup EXIT

echo "experiment,kr,kl,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,err_xy,err_yaw,sigma_x,sigma_y,sigma_yaw,consistent_xy,consistent_yaw" > "$RESULTS_FILE"

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash

for K in "${VALUES[@]}"; do
    echo ""
    echo "============================================================"
    echo "[SWEEP] Probando kr=kl=$K"
    echo "============================================================"

    # Sustituir kr y kl en el YAML
    sed -i "s/^\(\s*kr:\).*/\1 $K/" "$YAML"
    sed -i "s/^\(\s*kl:\).*/\1 $K/" "$YAML"

    # Matar cualquier nodo previo
    pkill -9 -f "ros2 launch" 2>/dev/null || true
    pkill -9 -f "puzzlebot_sim" 2>/dev/null || true
    pkill -9 -f "localisation" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    sleep 2

    # Lanzar mc5_launch.py sin RViz (headless para sweep)
    ros2 launch puzzlebot_sim mc5_launch.py use_rviz:=false \
        > /tmp/sweep_launch_${K}.log 2>&1 &
    LAUNCH_PID=$!

    # Esperar a que arranquen los nodos
    sleep 5

    # Correr el experimento y capturar output
    EXP_OUTPUT=$(ros2 run puzzlebot_sim experiment_runner \
        --ros-args -p experiment:=straight -p distance:=1.0 \
        -p linear_speed:=0.15 -p report_kr:=${K} -p report_kl:=${K} 2>&1) || true

    # Extraer la linea CSV_REPORT
    CSV_LINE=$(echo "$EXP_OUTPUT" | grep "^CSV_REPORT" | sed 's/^CSV_REPORT,//' | head -1)

    if [ -n "$CSV_LINE" ]; then
        echo "$CSV_LINE" >> "$RESULTS_FILE"
        echo "[SWEEP] Resultado kr=$K: OK"
    else
        echo "[SWEEP] WARN: No se encontro CSV_REPORT en output para kr=$K"
        echo "$EXP_OUTPUT" | tail -10
    fi

    # Matar el launch
    kill $LAUNCH_PID 2>/dev/null || true
    pkill -9 -f "ros2 launch" 2>/dev/null || true
    pkill -9 -f "puzzlebot_sim" 2>/dev/null || true
    pkill -9 -f "localisation" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    sleep 2
done

# Restaurar YAML original (el trap tambien lo hace, pero por claridad)
cp "$YAML_BAK" "$YAML"
rm -f "$YAML_BAK"

echo ""
echo "============================================================"
echo "SWEEP COMPLETO. Resultados en: $RESULTS_FILE"
echo "============================================================"

# Tabla bonita
echo ""
echo "Tabla comparativa:"
echo ""
printf "%-6s | %-10s | %-10s | %-10s | %-10s | %s\n" "kr=kl" "err_xy" "sigma_xy" "3sigma" "yaw_err" "consist"
printf -- "-------|------------|------------|------------|------------|---------\n"

while IFS=, read -r experiment kr kl gt_x gt_y gt_yaw est_x est_y est_yaw err_xy err_yaw sigma_x sigma_y sigma_yaw c_xy c_yaw; do
    [ "$experiment" = "experiment" ] && continue  # skip header
    s_xy=$(python3 -c "import math; print(f'{math.sqrt(float(\"$sigma_x\")**2 + float(\"$sigma_y\")**2):.6f}')")
    t_sigma=$(python3 -c "print(f'{3*float(\"$s_xy\"):.6f}')")
    consist="OK"
    [ "$c_xy" = "0" ] && consist="FAIL"
    [ "$c_yaw" = "0" ] && consist="FAIL"
    printf "%-6s | %-10s | %-10s | %-10s | %-10s | %s\n" "$kr" "$err_xy" "$s_xy" "$t_sigma" "$err_yaw" "$consist"
done < "$RESULTS_FILE"

echo ""
echo "Recomendacion: el kr/kl optimo es aquel donde sigma_xy sea similar"
echo "al err_xy real (idealmente sigma_xy ~ 2-3x err_xy), manteniendo"
echo "consistencia OK."

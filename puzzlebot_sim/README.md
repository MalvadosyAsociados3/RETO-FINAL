# puzzlebot_sim — Mini Challenge 5

Simulacion de un Puzzlebot diferencial en ROS2 Humble con propagacion de
incertidumbre (modelo EKF predictivo) para visualizar el elipsoide de
covarianza en RViz.

## Modelo matematico

Estado: s = [x, y, theta]

Cinematica (Euler explicito, dt = 1/50 s):

    x_k     = x_{k-1}     + v_k * dt * cos(theta_{k-1})
    y_k     = y_{k-1}     + v_k * dt * sin(theta_{k-1})
    theta_k = theta_{k-1} + w_k * dt

donde v_k = r*(wr+wl)/2 y w_k = r*(wr-wl)/L.

Propagacion de covarianza:

    Sigma_k = H_k * Sigma_{k-1} * H_k^T + Q_k
    Q_k     = grad_w * Sigma_Delta * grad_w^T
    Sigma_Delta = diag(kr*|wr|, kl*|wl|)

## Nodos

| Nodo | Archivo | Publishers | Subscribers |
|---|---|---|---|
| simulator | simulator.py | /wr, /wl, /joint_states, /sim_pose, /sim_pose_odom, /tf | /cmd_vel |
| localisation | localisation.py | /odom, /tf, /tf_static | /wr, /wl |
| experiment_runner | experiment_runner.py | /cmd_vel | — |

## Launch files

| Launch | Para que |
|---|---|
| mc5_launch.py | Setup minimo para Mini Challenge 5 (sim + localisation + rviz) |
| control_launch.py | Pipeline completo con waypoints (challenges previos) |
| localisation_launch.py | Solo sim + localisation, sin RViz |
| demo_launch.py | Demo de joint_state |

## Como correr

```bash
# Terminal 1: lanzar el simulador
ros2 launch puzzlebot_sim mc5_launch.py

# Terminal 2: experimento recto 1 metro
ros2 run puzzlebot_sim experiment_runner \
  --ros-args -p experiment:=straight -p distance:=1.0 -p linear_speed:=0.15

# Otros experimentos:
# rotate (una vuelta):
ros2 run puzzlebot_sim experiment_runner \
  --ros-args -p experiment:=rotate -p rotation:=6.283185 -p angular_speed:=0.5

# square (cuadrado de 0.5m):
ros2 run puzzlebot_sim experiment_runner \
  --ros-args -p experiment:=square -p distance:=0.5 -p linear_speed:=0.2 -p angular_speed:=1.0
```

## Topics clave para RViz

- /odom — pose estimada por localisation (display "Estimated", flecha roja
  + elipsoide cyan/magenta)
- /sim_pose_odom — ground truth del simulator (display "GroundTruth",
  flecha verde sin elipsoide)

Si el elipsoide se ve pequeno, subir Scale en RViz:
Estimated -> Covariance -> Position -> Scale = 5.0
Estimated -> Covariance -> Orientation -> Scale = 5.0

## Tuning de kr y kl (Task 2)

Editar puzzlebot_sim/config/puzzlebot_params.yaml:

```yaml
localisation:
  ros__parameters:
    kr: 0.02   # ruido relativo de rueda derecha
    kl: 0.02   # ruido relativo de rueda izquierda
```

- Subir kr/kl -> elipsoide mas grande (mas incertidumbre modelada)
- Bajar kr/kl -> elipsoide mas pequeno (modelo mas confiado)
- El tuning correcto es donde el elipsoide engloba la diferencia entre
  /odom (rojo) y /sim_pose_odom (verde).

### Resultado del sweep automatico

Implementamos `scripts/kr_kl_sweep.sh` que prueba 6 valores de kr/kl entre
0.01 y 0.5 y reporta la incertidumbre estimada (sigma_xy) en un experimento
de avance recto de 1 metro:

| kr=kl | sigma_xy | 3-sigma | consistencia |
|-------|----------|---------|--------------|
| 0.01  | 0.014 m  | 0.041 m | OK           |
| 0.02  | 0.013 m  | 0.038 m | OK (elegido) |
| 0.05  | 0.031 m  | 0.092 m | OK           |
| 0.10  | 0.044 m  | 0.131 m | OK           |
| 0.20  | 0.040 m  | 0.121 m | OK           |
| 0.50  | 0.097 m  | 0.292 m | OK (sobreestima) |

**Valor elegido:** `kr=kl=0.02` -- produce un sigma de 1.3 cm en 1 metro de
avance, valor proporcional al ruido tipico de encoders del Puzzlebot.
Mantiene consistencia (error real dentro de 3-sigma) sin sobreestimar la
incertidumbre.

Para correr el sweep:

```bash
./scripts/kr_kl_sweep.sh
```

## Compilar

```bash
cd ~/Documents/Robotica/MiniChallenge5/Minichallenge5/MiniChallenge5
colcon build --packages-select puzzlebot_sim --symlink-install
source install/setup.bash
```

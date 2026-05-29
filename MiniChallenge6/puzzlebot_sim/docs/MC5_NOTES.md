# Mini Challenge 5 — Notas tecnicas para el equipo

## Resumen

El Mini Challenge 5 pide dos tasks (PPTX del socio formador):

1. **Task 1**: completar la matriz de covarianza 3x3 en el mensaje
   /odom de localisation, usando propagacion EKF predictiva.
2. **Task 2**: tunear kr y kl en Sigma_Delta y demostrar el efecto
   en RViz comparando pose estimada vs ground truth.

## Task 1 — Como esta implementada

En localisation.py:

- Lineas 141-165: calculo de H (Jacobiano del estado), grad_w
  (Jacobiano del ruido), Sigma_delta (covarianza del ruido por rueda),
  Q (ruido de proceso) y propagacion Sigma_k = H Sigma_{k-1} H^T + Q.
- Lineas 207-220 (_pack_pose_covariance): empaquetado de Sigma 3x3
  [x, y, yaw] en la matriz 6x6 row-major del mensaje Odometry.
  Indices usados: 0, 1, 5, 6, 7, 11, 30, 31, 35.

## Task 2 — Metodologia propuesta

1. Correr `experiment_runner` modo `straight` y observar el elipsoide
   al final.
2. Si el elipsoide es mucho mas grande que la diferencia visual entre
   flecha verde y roja -> bajar kr/kl.
3. Si el elipsoide es mas pequeno que la diferencia -> subir kr/kl.
4. Repetir con `rotate` y `square` para verificar consistencia.

Valores probados (sweep automatico con scripts/kr_kl_sweep.sh):
- kr=kl=0.01 -> sigma_xy=0.014 m
- kr=kl=0.02 -> sigma_xy=0.013 m  <- VALOR ELEGIDO
- kr=kl=0.05 -> sigma_xy=0.031 m
- kr=kl=0.10 -> sigma_xy=0.044 m
- kr=kl=0.20 -> sigma_xy=0.040 m
- kr=kl=0.50 -> sigma_xy=0.097 m

Criterio de seleccion: proporcional al ruido tipico de encoders, manteniendo
consistencia (error real < 3-sigma) sin sobreestimar.

## Estructura del workspace

    MiniChallenge5/
    ├── build/        <- generado, ignorado por git
    ├── install/      <- generado, ignorado por git
    ├── log/          <- generado, ignorado por git
    ├── .gitignore
    └── puzzlebot_sim/    <- paquete ROS2 directamente (sin src/)
        ├── launch/
        │   ├── mc5_launch.py        <- NUEVO en MC5
        │   ├── control_launch.py
        │   ├── localisation_launch.py
        │   └── demo_launch.py
        ├── config/
        │   └── puzzlebot_params.yaml
        ├── rviz/
        │   └── puzzlebot_rviz.rviz
        ├── urdf/
        ├── meshes/
        └── puzzlebot_sim/
            ├── simulator.py
            ├── localisation.py
            ├── experiment_runner.py  <- NUEVO en MC5
            ├── control.py
            ├── point_generator.py
            └── joint_state_publisher.py

Compilar:

    colcon build --packages-select puzzlebot_sim --symlink-install
    source install/setup.bash

## Frames TF

    map --(static)-- odom --(dynamic)-- base_footprint --(URDF)-- base_link
     │                                                                │
     └--(dynamic)-- sim_base_footprint                              wheels, caster

- map -> odom (estatico identidad, publicado por localisation)
- odom -> base_footprint (dinamico, dead-reckoning)
- map -> sim_base_footprint (dinamico, ground truth del simulator)
- El resto del arbol viene del URDF via robot_state_publisher.

## Diagnostico

    ros2 node list
    ros2 topic hz /odom
    ros2 topic hz /tf
    ros2 topic echo /tf_static
    ros2 param get /localisation kr
    ros2 run tf2_ros tf2_echo map base_footprint
    rqt_graph

## Problemas conocidos

- **Elipsoide invisible en RViz**: subir Scale en
  Estimated -> Covariance -> Position/Orientation a 5.0 o mas.
- **Robot salton/buggeado**: nodos zombies de sesiones previas.
  Limpiar con `pkill -9 -f ros2`.
- **publish_map_odom_tf no aplica**: el YAML necesita el bloque
  completo de parametros (no solo wheel_radius/wheel_base).
- **/tf_static parece incompleto con `head -20`**: NO es bug. El
  mensaje viene en dos paquetes (TFs del URDF + el de localisation).
  Usar `ros2 topic echo /tf_static` sin pipe para ver todo.

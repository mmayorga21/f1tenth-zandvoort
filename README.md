# F1TENTH — Follow the Gap en Zandvoort

Proyecto individual de Vehículos Autónomos: desarrollar y evaluar un controlador reactivo Follow the Gap en ROS 2 Humble y el simulador F1TENTH.

## Objetivos y entregables

- Completar 10 vueltas consecutivas sin colisión.
- Medir el tiempo de cada vuelta, el total y la mejor vuelta.
- Contar vueltas automáticamente y mostrar los resultados durante la ejecución.
- Evaluar la vuelta más rápida y optimizar el desempeño mediante pruebas reproducibles.
- Entregar tutorial en GitHub, video con vehículo, contador y tiempos visibles, y sustentación oral.

## Estado al 15 de noviembre de 2026

Zandvoort activado y validado: mapa de ROS y simulador coinciden, LiDAR y odometría operativos, avance y giro comprobados a baja velocidad. El vehículo inicia en `(0, 0, 1.1954730407943466 rad)`. Véase el [informe de validación](docs/VALIDACION_ZANDVOORT.md).

El paquete propio `f1tenth_controller` implementa Follow the Gap y completó una vuelta ROS sin colisión en **321.83 s simulados** (374.13 m), usando una velocidad máxima de 1.2 m/s. Arranca desactivado y dispone de parada por datos vencidos, pausa, reinicio, colisión y pérdida del proceso. Véase el [tutorial del controlador y los resultados](docs/FOLLOW_THE_GAP.md).

El puente publica `/clock`, colisión y estado de intentos; pausa, reset y registro persistente están validados. Véase [la base de medición](docs/MEDICION_INTENTOS.md). El contador/cronómetro definitivos, diez vueltas consecutivas, optimización y video de entrega siguen pendientes.

## Estructura actual

```text
src/
├── controllers/        # Plantillas y ejemplos del repositorio base
├── f1tenth_controller/ # Controlador reactivo propio, parámetros, launch y pruebas
├── f1tenth_gym_ros/     # Puente ROS, configuración, launch y mapas
├── path_planning/      # Recursos del repositorio base
├── square_driver/      # Trabajo previo: recorrido rectangular, no Follow the Gap
└── waypoints/          # Recursos de trayectorias del repositorio base
docs/
├── PLAN_PROYECTO.md
├── PROCEDENCIA.md
├── VALIDACION_ZANDVOORT.md
├── MEDICION_INTENTOS.md
├── FOLLOW_THE_GAP.md
└── README_BASE_EN.md
```

La implementación propia está en `f1tenth_controller`. Las plantillas del paquete `controllers` y el antecedente `square_driver` se conservan separados.

## Entorno de referencia y ejecución del simulador

Ubuntu 22.04, ROS 2 Humble, Python 3.10, `colcon`, `f1tenth_gym_ros` y `f110_gym`. Versiones verificadas: Python 3.10.12, NumPy 1.22.0, Numba 0.68.0, Gym 0.19.0 y F110 Gym 0.2.1. El procedimiento de instalación completo se revisará para la entrega reproducible.

En el equipo actual, con el workspace ya compilado:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Este comando lanza Zandvoort con el vehículo detenido. Para activar la conducción autónoma, seguir el [tutorial de Follow the Gap](docs/FOLLOW_THE_GAP.md): el controlador requiere lanzamiento y activación explícitos. La ruta del mapa en la configuración actual es específica del equipo de desarrollo.

La integración usa `/scan` para LiDAR, `/drive` para comandos Ackermann y `/ego_racecar/odom` para telemetría. Los comandos del controlador y la explicación detallada del algoritmo están en el [tutorial](docs/FOLLOW_THE_GAP.md).

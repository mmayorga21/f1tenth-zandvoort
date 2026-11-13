# F1TENTH — Follow the Gap en Zandvoort

Proyecto individual de Vehículos Autónomos: desarrollar y evaluar un controlador reactivo Follow the Gap en ROS 2 Humble y el simulador F1TENTH.

## Objetivos y entregables

- Completar 10 vueltas consecutivas sin colisión.
- Medir el tiempo de cada vuelta, el total y la mejor vuelta.
- Contar vueltas automáticamente y mostrar los resultados durante la ejecución.
- Evaluar la vuelta más rápida y optimizar el desempeño mediante pruebas reproducibles.
- Entregar tutorial en GitHub, video con vehículo, contador y tiempos visibles, y sustentación oral.

## Estado al 12 de noviembre de 2026

Proyecto en preparación. Los mapas de Zandvoort están incorporados, pero `sim.yaml` todavía selecciona LevineB. `controllers/gap_node.py` es una plantilla incompleta. El controlador del proyecto, el contador, el cronómetro y la validación de 10 vueltas siguen pendientes. No hay resultados de carrera registrados.

El funcionamiento previo del entorno fue reportado en la conversación de origen; debe volver a comprobarse durante la validación de Zandvoort. El puente inspeccionado no publica `/clock`; la fuente de tiempo y la exposición de colisiones deben resolverse antes de validar tiempos.

## Estructura actual

```text
src/
├── controllers/       # Plantillas y ejemplos del repositorio base
├── f1tenth_gym_ros/   # Puente ROS, configuración, launch y mapas
├── path_planning/     # Recursos del repositorio base
├── square_driver/     # Trabajo previo: recorrido rectangular, no Follow the Gap
└── waypoints/         # Recursos de trayectorias del repositorio base
docs/
├── PLAN_PROYECTO.md
├── PROCEDENCIA.md
└── README_BASE_EN.md
```

La implementación nueva se organizará en un paquete propio. Se conservarán los antecedentes sin confundirlos con la solución del proyecto.

## Entorno de referencia y ejecución del simulador

Ubuntu 22.04, ROS 2 Humble, Python 3.10, `colcon`, `f1tenth_gym_ros` y `f110_gym`. Las versiones efectivas y el procedimiento de instalación se documentarán después de verificarlos.

En el equipo actual, con el workspace ya compilado:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Este comando lanza el simulador con la configuración vigente; todavía no constituye una demostración de conducción autónoma en Zandvoort. La ruta del mapa en la configuración actual es específica del equipo de desarrollo.

La integración prevista usa `/scan` para LiDAR, `/drive` para comandos Ackermann y `/ego_racecar/odom` para telemetría. Los comandos del controlador y la explicación detallada del algoritmo se agregarán cuando exista una implementación verificada.

## Plan, resultados y procedencia

- [Plan por etapas y criterios de aceptación](docs/PLAN_PROYECTO.md).
- [Procedencia del código y de los archivos](docs/PROCEDENCIA.md).
- [Tutorial original en español](README_es.md).
- [Tutorial original en inglés](docs/README_BASE_EN.md).

Los resultados, parámetros finales y enlace al video se añadirán después de las pruebas. El video es obligatorio según el enunciado académico.

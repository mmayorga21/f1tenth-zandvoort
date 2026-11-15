# F1TENTH — Follow the Gap en Zandvoort

Proyecto individual de Vehículos Autónomos: controlador reactivo Follow the Gap con ROS 2 Humble y el simulador F1TENTH.

## Objetivo y estado actual

El proyecto debe completar diez vueltas consecutivas sin colisión, medir cada vuelta y competir por la vuelta más rápida. La entrega incluye este tutorial, contador automático, cronómetro visible, video del vehículo con contador y tiempos, y sustentación oral.

Al 15 de noviembre de 2026, el controlador completó una vuelta autónoma con ROS sin colisión en **321.83 s simulados**, recorriendo **374.13 m**, con velocidad máxima de **1.2 m/s**. Están implementados el reloj de simulación, la detección de colisión y el control de sesiones. Quedan pendientes el contador y cronómetro definitivos, diez vueltas consecutivas, optimización y video de entrega.

## Estructura del código

| Ruta | Función |
| --- | --- |
| [gap.py](src/f1tenth_controller/f1tenth_controller/gap.py) | Algoritmo Follow the Gap, independiente de ROS |
| [gap_node.py](src/f1tenth_controller/f1tenth_controller/gap_node.py) | Recepción del LiDAR, publicación de mandos y servicios ROS |
| [control_gate.py](src/f1tenth_controller/f1tenth_controller/control_gate.py) | Activación, disponibilidad de datos y cambios de sesión |
| [follow_the_gap.yaml](src/f1tenth_controller/config/follow_the_gap.yaml) | Parámetros del controlador |
| [follow_the_gap.launch.py](src/f1tenth_controller/launch/follow_the_gap.launch.py) | Lanzamiento del controlador, desactivado inicialmente |
| [gym_bridge.py](src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py) | Puente entre F110 Gym y ROS; reloj, colisiones e intentos |
| [simulation_session.py](src/f1tenth_gym_ros/f1tenth_gym_ros/simulation_session.py) | Estado y tiempo de cada sesión e intento |
| [sim.yaml](src/f1tenth_gym_ros/config/sim.yaml) | Mapa, pose inicial y configuración del simulador |
| `scripts/` | Comprobaciones de integración y recorridos de prueba |

Los paquetes `controllers` y `path_planning`, las trayectorias de `waypoints` y los laboratorios proceden del repositorio base. `square_driver` corresponde al trabajo previo de recorrido rectangular. El controlador de este proyecto está en `f1tenth_controller`.

## Funcionamiento del algoritmo

El nodo recibe `/scan` y calcula decisiones de dirección y velocidad a partir del LiDAR. `/sim/status` permite o bloquea la conducción según el estado del simulador. La selección de dirección no utiliza mapa ni odometría.

1. **Interpretación y limpieza.** Cada rayo usa `angle_min + i * angle_increment`. Se conserva el sector frontal de ±90° y se limita la distancia útil a 6 m. Los valores inválidos no crean espacio libre; el infinito positivo representa ausencia de retorno hasta el alcance del sensor.
2. **Filtrado conservador.** Un filtro de mínimo de tres rayos conserva obstáculos estrechos. Una distancia menor a 0.45 m o un dato inválido en el cono frontal provoca parada.
3. **Margen para el vehículo.** En discontinuidades de distancia se extiende el obstáculo cercano hacia el lado más lejano. También se elimina una burbuja alrededor del obstáculo más próximo. Ambas operaciones usan un radio de seguridad de 0.30 m y una amplitud angular dependiente de la distancia.
4. **Selección del hueco.** Se buscan intervalos contiguos con distancia útil suficiente. Dentro de ellos se promedia la profundidad sobre una ventana de hasta 31 rayos. La puntuación combina profundidad, orientación hacia delante y continuidad respecto al objetivo anterior; sus extremos se penalizan para mantener margen.
5. **Dirección.** El objetivo del LiDAR se convierte a dirección de las ruedas mediante geometría local. Se limita el giro a ±0.4189 rad y se suaviza su variación.
6. **Velocidad.** La velocidad se reduce según la dirección deseada y el despeje frontal. La aceleración se limita a 1 m/s². El perfil actual es conservador y se mantiene como referencia antes de optimizar.

La conversión geométrica de dirección utiliza la distancia entre ejes del backend, `L = 0.3302 m`:

```text
d = mínimo(1.4 m, distancia útil hacia el objetivo)
dirección deseada = atan2(2 * L * sin(ángulo_objetivo), d)
```

Los parámetros se cargan desde el YAML al iniciar el nodo. Para aplicar cambios, editar el archivo y relanzar el controlador; modificar un parámetro ROS en caliente no reconstruye el algoritmo.

## Entorno y configuración

Entorno verificado: Ubuntu 22.04, ROS 2 Humble, Python 3.10.12, NumPy 1.22.0, Numba 0.68.0, Gym 0.19.0 y F110 Gym 0.2.1. Los siguientes comandos parten del workspace y las dependencias ya instalados; el procedimiento de instalación desde cero se revisará para la entrega final.

El mapa está en `src/f1tenth_gym_ros/maps/Zandvoort_map.*`. La configuración actual utiliza una ruta absoluta del equipo de desarrollo: al clonar en otra ubicación, adaptar `map_path` en `sim.yaml`, sin extensión, y mantener juntos el PNG y su YAML.

La pose de desarrollo es `x=0`, `y=0`, `yaw=1.1954730407943466 rad`, alineada con la recta larga. La posición de salida y el sentido definitivos deben ajustarse a las reglas del profesor. La teleoperación está deshabilitada para evitar mandos simultáneos.

## Compilación y ejecución

Terminal 1: compilar y lanzar el simulador.

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_gym_ros f1tenth_controller
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Terminal 2: lanzar el controlador. Arranca desactivado.

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_controller follow_the_gap.launch.py
```

Terminal 3: activar, consultar el estado y detener.

```bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
ros2 service call /controller/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /controller/status --once
ros2 service call /controller/enable std_srvs/srv/SetBool '{data: false}'
```

Para pausar, reanudar o restablecer el simulador:

```bash
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'
ros2 service call /sim/reset std_srvs/srv/Trigger '{}'
```

Tras una pausa o un reinicio hay que activar el controlador de nuevo explícitamente. Ejecutar únicamente un controlador que publique en `/drive`.

## Tiempo, colisiones y protecciones

El puente publica `/clock` a partir de pasos de física de 0.01 s. LiDAR, odometría y transformaciones usan ese mismo tiempo. Los consumidores usan `use_sim_time=True`; los temporizadores de física del puente funcionan con un reloj monotónico independiente para evitar depender del reloj que producen.

`/sim/status` identifica el proceso, la sesión y el intento, con estados `ready`, `running`, `ended` o `collision`. El primer mando de velocidad no nula inicia el intento. Frenar con velocidad cero permite que siga contando el tiempo; una pausa explícita congela física y tiempo. Un reset crea una sesión nueva e invalida un intento activo.

La colisión se obtiene de la señal del backend y se publica en `/ego_racecar/collision`. Permanece retenida hasta reset. `attempt_valid=true` significa que el intento sigue libre de colisión o interrupción; la comprobación definitiva de diez vueltas se incorporará con el contador.

El controlador se desactiva y envía velocidad cero ante pérdida o antigüedad de datos, ausencia de hueco seguro, pausa, colisión, fin de intento, cambio de sesión o aparición de otro publicador. Su watchdog sigue funcionando aunque `/clock` esté detenido. Si desaparecen los mandos de avance durante más de 0.3 s, el puente invalida y congela el intento con motivo `drive_timeout`.

`/controller/status` muestra activación, última decisión y configuración. Los eventos de sesión se publican en `/sim/events` y se registran localmente en `~/.ros/f1tenth_zandvoort/events.jsonl`. Estos archivos son registros de desarrollo y se mantienen fuera del repositorio.

## Resultados y validación

| Prueba | Resultado |
| --- | --- |
| Vuelta directa en F110 Gym | 321.84 s simulados, sin colisión |
| Vuelta con ROS | 321.83 s simulados, sin colisión |
| Recorrido ROS | 374.13 m; seis regiones en orden y cruce dirigido de salida |
| Pruebas unitarias | 12 del controlador y seis de sesiones, pasadas |
| Activación y vuelta ROS | 12 comprobaciones pasadas |
| Pérdida de datos/proceso | Cinco comprobaciones pasadas |
| Regresión de sesiones | 23 comprobaciones pasadas |

La verificación del recorrido usa odometría en un proceso separado del controlador. `scripts/lap_validation.py` comprueba regiones en orden, distancia recorrida y cruce dirigido de una línea de desarrollo. El contador y cronómetro definitivos, con presentación visible, están pendientes.

Para repetir las pruebas unitarias:

```bash
python3 -m unittest discover -s src/f1tenth_controller/test -p test_gap.py -v
python3 -m unittest discover -s src/f1tenth_gym_ros/test -p test_simulation_session.py -v
```

Para repetir una vuelta con simulador y controlador iniciados y desactivados:

```bash
python3 scripts/validate_gap_ros.py --output /tmp/gap_ros.json
```

El verificador activa el vehículo, comprueba la vuelta y finalmente lo desactiva y restablece a la pose inicial. La velocidad y el tiempo actuales son una referencia conservadora; aún falta validar diez vueltas consecutivas y optimizar el desempeño.

Este proyecto parte de [widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository). Se conserva su historial y los avisos de licencia originales. El puente mantiene su [licencia MIT](src/f1tenth_gym_ros/LICENSE) y el paquete propio incluye [su licencia](src/f1tenth_controller/LICENSE).

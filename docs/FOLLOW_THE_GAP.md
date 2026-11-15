# Etapa 4: controlador reactivo Follow the Gap

Implementación del 15 de noviembre de 2026. El objetivo de esta etapa es una primera vuelta completa a velocidad conservadora y una integración que permita detener y reiniciar de forma controlada. La optimización de tiempos y las diez vueltas corresponden a etapas posteriores.

## Arquitectura y separación de responsabilidades

El paquete propio `src/f1tenth_controller` contiene:

| Archivo | Responsabilidad |
| --- | --- |
| `f1tenth_controller/gap.py` | Lógica matemática sin ROS, mapa ni pose; devuelve dirección, velocidad y explicación de la decisión |
| `f1tenth_controller/control_gate.py` | Activación explícita, identidad de sesión y comprobación de disponibilidad de datos |
| `f1tenth_controller/gap_node.py` | Adaptador ROS, suscripciones, publicación Ackermann, servicios y watchdog |
| `config/follow_the_gap.yaml` | Configuración reproducible del controlador |
| `launch/follow_the_gap.launch.py` | Lanzamiento desactivado por defecto |
| `test/test_gap.py` | Pruebas de geometría, datos inválidos, límites y activación |

El nodo usa `/scan` para tomar decisiones de conducción. Recibe `/sim/status` para permitir o bloquear la conducción ante pausas, colisiones y cambios de sesión. No usa la pose del estado del simulador para elegir dirección o velocidad, y no se suscribe a `/map` ni a odometría. Esta separación mantiene reactiva la navegación y permite probarla de forma independiente.

La verificación experimental de la vuelta se ejecuta en otro proceso. Allí sí se utiliza odometría para comprobar el recorrido; sus regiones de control no se envían al controlador.

## Cómo se transforma un escaneo en un mando

### 1. Ángulos y selección frontal

Para el rayo de índice `i`, el ángulo se obtiene con `angle_min + i * angle_increment`, usando los campos reales del mensaje. Se consideran rayos dentro de ±90°: un objetivo situado detrás no es una maniobra válida para esta política de avance.

### 2. Limpieza conservadora

Se limitan las distancias útiles a 6 m. Esto evita que una apertura muy lejana domine las decisiones y ofrece una escala estable para comparar huecos. `NaN`, infinito negativo y valores no positivos no se convierten en espacio libre. El infinito positivo se interpreta como ausencia de retorno hasta el alcance del sensor.

Se aplica un filtro de mínimo de tres rayos: conserva obstáculos estrechos en vez de promediarlos con distancias grandes. En el cono frontal de aproximadamente ±8°, una distancia menor a 0.45 m o un dato inválido obliga a parar.

### 3. Anchura del vehículo y discontinuidades

En cada salto de distancia importante se extiende el obstáculo cercano hacia el lado más lejano. La amplitud angular depende de la distancia y de un radio de seguridad de 0.30 m. Esta extensión reduce la posibilidad de elegir un rayo que pase justo junto a una esquina aunque el cuerpo del coche no quepa.

También se elimina una burbuja alrededor del obstáculo más próximo. El radio elegido supera la semianchura del modelo (0.155 m) y añade margen. Es una protección geométrica aproximada; su eficacia se comprueba con la señal real de colisión del backend.

### 4. Huecos y objetivo

Se identifican intervalos contiguos con al menos 1 m de distancia útil y una amplitud angular mínima. Dentro de cada intervalo se promedia la profundidad sobre una ventana de hasta 31 rayos. El relleno con ceros en los extremos penaliza elegir demasiado cerca del borde.

La puntuación combina profundidad, orientación hacia delante y continuidad respecto al objetivo anterior. No se selecciona simplemente el rayo aislado más largo: eso puede producir cambios bruscos o apuntar a un borde. La continuidad evita cambiar de lado innecesariamente sin impedir un giro cuando aparece un hueco mejor.

### 5. Dirección de las ruedas

El ángulo hacia el objetivo no equivale al ángulo de las ruedas. Se utiliza una conversión geométrica local:

```text
d = mínimo(1.4 m, distancia útil hacia el objetivo)
dirección deseada = atan2(2 * distancia_entre_ejes * sin(ángulo_objetivo), d)
```

La distancia entre ejes es 0.3302 m, suma de los parámetros `lf` y `lr` del backend. Esta geometría usa un objetivo reactivo del LiDAR; no utiliza una trayectoria global.

La dirección se limita a ±0.4189 rad. Después se suaviza con una constante de tiempo de 0.08 s y un límite de cambio de 2 rad/s. El suavizado reduce oscilaciones sin fijar artificialmente la dirección durante mucho tiempo.

### 6. Velocidad

La velocidad máxima inicial es 1.2 m/s. La política la reduce según la dirección deseada, anticipando un giro antes de que las ruedas terminen de orientarse. En el giro máximo, la velocidad base se aproxima a 0.65 m/s; la proximidad frontal puede reducirla aún más. La aceleración está limitada a 1 m/s².

Estos valores son un perfil conservador para validar el algoritmo, no una configuración optimizada para competir. Se conservaron tras la validación directa y se probaron después con ROS.

## Activación y protecciones

El controlador arranca desactivado. Solo puede activarse si hay estado reciente del simulador, escaneos recientes, una sesión habilitada para avanzar y un único publicador de `/drive`.

Se desactiva y envía velocidad cero ante datos inválidos, ausencia de hueco seguro, escaneos antiguos, pérdida de LiDAR, estado del simulador vencido, pausa, colisión, fin de intento, reinicio o aparición de otro publicador. Reanudar o reiniciar el simulador no reactiva el controlador automáticamente.

El watchdog del nodo utiliza un reloj monotónico independiente de `/clock`: debe seguir comprobando la seguridad aunque el tiempo simulado esté detenido. Los escaneos se comprueban además contra el reloj simulado y sus marcas deben avanzar dentro de la sesión.

El puente tiene una protección complementaria: si conserva un mando de avance y deja de recibir comandos durante más de `drive_timeout_s` (0.3 s por defecto), borra el mando e invalida/congela el intento con motivo `drive_timeout`. Así se cubre también la caída completa del proceso controlador. La congelación es una acción del simulador; no pretende representar el frenado de un vehículo físico.

El controlador publica `/controller/status` con su estado de activación, última decisión, parámetros y hashes de código/configuración. El puente incorpora esos metadatos al registro de intentos y emite `controller_configured` cuando cambia la configuración anunciada.

## Compilar y ejecutar

Terminal 1, simulador:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_gym_ros f1tenth_controller
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Terminal 2, controlador inicialmente desactivado:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_controller follow_the_gap.launch.py
```

Terminal 3, activación y parada:

```bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
ros2 service call /controller/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /controller/status --once
ros2 service call /controller/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /sim/reset std_srvs/srv/Trigger '{}'
```

Para modificar el perfil, editar `follow_the_gap.yaml` y relanzar el nodo. Los parámetros se leen al construir el algoritmo; cambiar un parámetro ROS en caliente no reconstruye la lógica. El lanzamiento suministra `use_sim_time=True`, necesario para comprobar las marcas del LiDAR.

## Verificación independiente de la vuelta

`scripts/lap_validation.py` comprueba seis regiones amplias en orden, distancia recorrida superior a 200 m y cruce de un segmento de salida en el sentido de avance. Los aproximadamente 374 m recorridos y la trayectoria registrada permiten comprobar el trazado completo. No es todavía el contador definitivo de la entrega ni una definición oficial de meta del profesor.

La elección inicial de una región de verificación se corrigió al comprobar visualmente que estaba fuera de la vía. Esa corrección no modificó el algoritmo ni sus parámetros de conducción.

Para ejecutar el mismo núcleo directamente con F110 Gym:

```bash
python3 scripts/run_gap_offline.py --output /tmp/gap_offline.json
```

Para verificar activación, frenado, pausa, reinicio y una vuelta con ROS, teniendo simulador y controlador iniciados y desactivados:

```bash
python3 scripts/validate_gap_ros.py --output /tmp/gap_ros.json
```

El verificador activa el vehículo, comprueba el recorrido y finalmente lo desactiva y reinicia. La prueba de fallos de procesos está separada en `scripts/validate_gap_failures.py`; requiere los PID explícitos del puente y del controlador, y los reanuda al terminar.

## Pruebas y resultados

| Prueba | Resultado medido |
| --- | --- |
| Vuelta con núcleo directo en F110 Gym | 321.84 s simulados, sin colisión |
| Vuelta con controlador ROS | 321.83 s simulados, sin colisión |
| Distancia de la vuelta ROS | 374.13 m |
| Verificación del recorrido | Seis regiones en orden y cruce dirigido de salida |
| Velocidad máxima observada | Aproximadamente 1.20 m/s |
| Mando de dirección máximo observado en la telemetría | 0.3062 rad, inferior al límite de 0.4189 rad |
| Comprobaciones ROS de activación y vuelta | 12 pasadas |
| Inyección de pérdida de datos/proceso | Cinco comprobaciones pasadas |
| Regresión de las sesiones de etapa 3 | 23 comprobaciones pasadas |
| Pruebas unitarias | 12 del controlador y seis de sesiones, pasadas |
| Integración de pruebas con colcon | 12 pruebas recogidas y pasadas |

La pérdida de escaneos produjo desactivación y comandos de velocidad cero. El observador registró la condición a los 0.57 s aproximadamente; esa cifra incluye el periodo de publicación del estado y la espera del verificador. Al suspender el proceso controlador, el puente invalidó el intento por `drive_timeout` en unos 0.32 s observados. Al reanudar el proceso no se reactivó la conducción.

Se registraron 3220 muestras de trayectoria ROS, todas dentro del componente de pista del PNG. La ausencia de colisión se verificó mediante el estado retenido del simulador y la continuidad del mismo intento; no se dedujo únicamente de una inspección visual. El tiempo tiene resolución de física de 0.01 s y corresponde a una línea de validación de desarrollo, pendiente de las reglas oficiales de salida/meta.

El coche quedó restablecido a la pose de salida y el controlador desactivado. Estos resultados no representan todavía una sesión de diez vueltas ni una configuración optimizada de velocidad. El video obligatorio y la visualización definitiva de vueltas y tiempos siguen pendientes.

Pruebas unitarias:

```bash
python3 -m unittest discover -s src/f1tenth_controller/test -p test_gap.py -v
python3 -m unittest discover -s src/f1tenth_gym_ros/test -p test_simulation_session.py -v
```

La etapa 5 desarrollará el contador y cronómetro definitivos, su presentación visible y sus pruebas de antirrebote. No se han iniciado como parte de esta etapa.

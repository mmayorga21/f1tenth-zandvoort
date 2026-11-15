# Etapa 3: tiempo de simulación, colisiones e intentos

Implementado y validado el 12 de noviembre de 2026. Este módulo proporciona la base de medición; todavía no cuenta vueltas ni declara una carrera completada.

## Reloj y marcas de tiempo

El puente publica `/clock` como `rosgraph_msgs/msg/Clock`. El tiempo se obtiene del número de pasos de física completados multiplicado por el `timestep` real del backend (0.01 s en este entorno), con representación entera en nanosegundos. Tras cada reinicio comienza en cero; el paso interno que F110 Gym realiza durante `reset()` sirve para inicializar observaciones y no se cuenta como tiempo del nuevo intento.

Los temporizadores que hacen avanzar la física usan un reloj monotónico independiente de `/clock`. El puente permanece con `use_sim_time=False` para evitar depender del reloj que produce. RViz, publicadores del robot, servidor de mapas y lifecycle manager usan `use_sim_time=True`. Los futuros nodos de controlador y telemetría también deberán usarlo.

LiDAR, odometría y transformaciones reciben marcas del mismo tiempo de física. Se publica como máximo una muestra de sensores por paso, eliminando las repeticiones a 250 Hz de la etapa anterior. La frecuencia nominal de física/sensores es ahora 100 Hz en tiempo real si el equipo mantiene el ritmo; bajo carga puede ser menor. El tiempo simulado sigue correspondiendo a los pasos ejecutados.

El reloj avanza en estado `ready`, aunque el coche esté quieto. El tiempo del intento empieza con el primer comando de velocidad no nula y excluye la espera inicial. Un comando de velocidad cero frena, pero no pausa el intento: el tiempo sigue contando. Una pausa explícita congela física, sensores y tiempo del intento, manteniendo disponible el estado por ROS.

## Sesiones e intentos

| Campo/estado | Significado |
| --- | --- |
| `run_id` | Identifica una ejecución del proceso del puente; cambia al reiniciarlo |
| `session_id` y `reset_index` | Identifican el estado desde el último reinicio o cambio de pose |
| `attempt_id` | Se asigna al recibir el primer mando de velocidad no nula |
| `ready` | Preparado, sin intento iniciado |
| `running` | Intento en curso; `paused` indica si está suspendido |
| `ended` | Finalización manual o interrupción registrada |
| `collision` | Colisión detectada por el backend; intento inválido y física congelada |
| `attempt_valid` | `null` sin intento; `true` provisionalmente libre de colisión; `false` por colisión o interrupción |
| `elapsed_sim_s` | Duración del intento en tiempo de física, excluyendo pausas |
| `end_reason` | Motivo de finalización o invalidación |

`attempt_valid=true` no certifica diez vueltas ni cumplimiento de todas las reglas de competición. El conteo y la validación deportiva se incorporarán en las etapas siguientes. La pausa es una herramienta de diagnóstico; su uso permitido durante la competencia debe aclararse con el profesor.

Una colisión se obtiene de `obs['collisions'][0]`, no de una estimación de distancia LiDAR ni de que el coche deje de moverse. Se conserva hasta el siguiente reset. Los mandos y una solicitud de reanudar no pueden quitar el estado de colisión.

El campo `done` nativo de F110 Gym incluye una heurística de vueltas del backend. Se conserva para inspección interna, pero no se usa para detener nuestros intentos: hacerlo impediría implementar correctamente la prueba futura de diez vueltas. El contador propio se desarrollará en la etapa 5.

## Interfaces ROS

| Interfaz | Tipo | Contenido |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Reloj del simulador, QoS best effort |
| `/ego_racecar/collision` | `std_msgs/msg/Bool` | Colisión retenida hasta reset, QoS reliable/transient local |
| `/sim/status` | `std_msgs/msg/String` con JSON | Estado, identificadores, tiempo, pose, velocidad y configuración; 10 Hz y transiciones |
| `/sim/events` | `std_msgs/msg/String` con JSON | Eventos de inicio, pausa, reanudación, cierre, colisión y nueva sesión |
| `/sim/reset` | `std_srvs/srv/Trigger` | Restaura la pose configurada y limpia mandos, intento y colisión |
| `/sim/pause` | `std_srvs/srv/SetBool` | Pausa o reanuda; borra mandos almacenados |
| `/sim/end_attempt` | `std_srvs/srv/Trigger` | Cierra manualmente el intento y congela la física; requiere reset para otro intento |

El cambio de pose por `/initialpose` también crea una sesión nueva e invalida un intento activo. Tanto este cambio como `/sim/reset` eliminan los mandos antiguos antes de reiniciar. Al reanudar una pausa se necesitan mandos nuevos; la dinámica puede conservar la velocidad que tenía al congelarse y frenar con el mando cero.

Un cierre normal del puente registra el intento activo como inválido con motivo `shutdown`. Ante una terminación abrupta sin posibilidad de escribir el cierre, el intento queda incompleto en el registro: no debe interpretarse como un éxito. El siguiente proceso tendrá un `run_id` distinto.

La física queda congelada en estados terminales: una velocidad publicada allí describe el estado del modelo en ese instante congelado. Para detener con dinámica de frenado antes de un cierre manual, enviar primero velocidad cero. Los consumidores deben atender al estado, no solo al valor de velocidad.

## Registro persistente

Los eventos se añaden, una línea JSON por evento, a:

```text
~/.ros/f1tenth_zandvoort/events.jsonl
```

La ruta se configura con el parámetro ROS `event_log_path`. El registro incluye fecha UTC de escritura, identificadores, tiempos, motivo, configuración del vehículo, mapa y hashes de mapa y código. La fecha real sirve para identificar archivos; no se usa para cronometrar la carrera. Los parámetros del futuro controlador se incorporarán junto con su implementación.

La lógica de sesiones vive en `simulation_session.py`, separada de ROS para probar sus invariantes. El puente se encarga de integrar esa lógica con el simulador, publicar datos y escribir eventos.

## Uso

Con el simulador iniciado y el entorno ROS cargado:

```bash
ros2 topic echo /clock --qos-reliability best_effort --once
ros2 topic echo /sim/status --once
ros2 topic echo /ego_racecar/collision --once
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'
ros2 service call /sim/reset std_srvs/srv/Trigger '{}'
ros2 service call /sim/end_attempt std_srvs/srv/Trigger '{}'
```

Un reset hace retroceder `/clock` a cero. Es normal que RViz avise de un salto temporal y limpie su buffer TF. Los consumidores futuros deben reiniciar sus mediciones al cambiar `session_id`; no basta con observar un retroceso de tiempo.

## Validación ejecutada

- Seis pruebas unitarias: espera inicial, pausa, invalidación por reset, colisión retenida, cierre manual y distinción de reinicios/cierre.
- Veintitrés comprobaciones ROS integradas: fuente única de reloj, consumidor ROS coherente, sellos en la cuadrícula de 10 ms, pausa, reanudación, cierre, cambios de pose, limpieza de mandos y colisión real del backend.
- Durante una espera de 0.6 s de tiempo real en pausa, el reloj quedó exactamente en 1.11 s y la duración del intento en 0.60 s.
- Choque deliberado dentro del simulador: intento invalidado a los 1.12 s simulados. La colisión quedó retenida hasta reset.
- Cierre normal de un intento activo y reinicio real del proceso: evento `shutdown` inválido y nuevos identificadores, sin heredar el intento.
- El proceso final ejecuta el mismo hash de código que los archivos fuente comprobados y queda en `ready`, con el vehículo detenido.

Para repetir las pruebas unitarias:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s src/f1tenth_gym_ros/test -p test_simulation_session.py -v
```

Para repetir la prueba de integración, únicamente con el simulador Zandvoort de un agente y sin otro controlador:

```bash
python3 scripts/validate_session.py --allow-simulated-collision --output /tmp/session_validation.json
```

La prueba provoca un choque intencional en simulación y finalmente restaura la pose configurada. No es una prueba de conducción autónoma. El cierre/rearranque real del proceso se verificó por separado; no lo realiza automáticamente ese script.

Siguiente hito: implementar el controlador reactivo Follow the Gap y conseguir una vuelta a velocidad conservadora.

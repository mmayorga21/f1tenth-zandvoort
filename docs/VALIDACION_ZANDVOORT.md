# Etapa 2: activación y validación de Zandvoort

Validación ejecutada el 12 de noviembre de 2026 en el equipo de desarrollo. Esta etapa valida integración y movimientos breves; no demuestra todavía conducción autónoma ni vueltas completas.

Estado posterior: [la etapa 3](MEDICION_INTENTOS.md) incorporó `/clock`, colisiones por ROS y sensores sin duplicados a la frecuencia de física. Los resultados de este informe describen la ejecución original de la etapa 2.

## Configuración aplicada

- Mapa del puente: `src/f1tenth_gym_ros/maps/Zandvoort_map`, extensión `.png`.
- Mapa del servidor ROS: el YAML del mismo mapa, confirmado mediante parámetro efectivo.
- Pose inicial: `x=0`, `y=0`, `yaw=1.1954730407943466 rad` (aproximadamente 68.5°).
- Teleoperación por `/cmd_vel` deshabilitada (`kb_teleop: False`) para evitar mandos simultáneos.
- Los archivos originales PNG y YAML no fueron modificados.

El punto `(0,0)` está dentro de la recta larga. El rumbo anterior `0 rad` veía una pared a aproximadamente 1.147 m; el nuevo rumbo está alineado con la recta y ve aproximadamente 30 m hacia delante. Esta es una pose de desarrollo, pendiente de las reglas de salida de la competencia.

## Resultados observados

| Comprobación | Resultado |
| --- | --- |
| Imports de NumPy, Numba, Gym y F110 Gym | Correctos |
| Compilación selectiva del puente | Correcta |
| Nodos | Bridge, map server, lifecycle manager, robot state publisher y RViz iniciados |
| Ciclo de vida del servidor de mapas | `active` |
| Configuración efectiva de ambos mapas | Zandvoort |
| Mapa recibido | 2000 × 2000, resolución 0.05109 m/píxel, origen del YAML |
| Contenido de `/map` | Coincide exactamente con la conversión del PNG original a ocupación ROS |
| LiDAR | 1080 rayos; todas las muestras iniciales finitas y positivas |
| Frecuencia de entrega observada de scan y odometría | Aproximadamente 250 Hz de tiempo real; no es frecuencia de física |
| Avance: mando 0.5 m/s durante 1.5 s de tiempo real | Desplazamiento de 0.6672 m, incluyendo transición y frenado |
| Giro: mando positivo de 0.1 rad a 0.5 m/s | Incremento de orientación de 0.2245 rad: signo correcto |
| Parada y reinicio | Pose inicial restaurada; velocidad final 0 m/s |
| Publicadores de `/drive` al terminar | 0; un suscriptor del simulador |
| Publicadores de `/clock` | 0; pendiente de etapa 3 |

El simulador directo informó ausencia de colisión al salir desde la pose elegida y tras 100 pasos con mando recto de 0.5 m/s. En la prueba ROS, las 3231 poses muestreadas permanecieron dentro del mismo componente de pista. La distancia a paredes del PNG, descontando el radio circunscrito del vehículo, dejó un margen conservador mínimo de aproximadamente 0.675 m.

La prueba ROS aún no dispone de un tópico de colisión del simulador. La comprobación geométrica de las muestras es evidencia adicional, no sustituye la telemetría de colisión que se implementará en la etapa 3. RViz arrancó y registró la creación del mapa; se comprobó la coherencia entre mapa, LiDAR y odometría.

## Reproducción en el equipo actual

Terminal 1:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_gym_ros
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Terminal 2, con el simulador iniciado y sin otro controlador:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/validate_simulator.py --output /tmp/zandvoort_sensors.json
```

Para repetir también el movimiento breve, giro, parada y retorno a la pose inicial, agregar `--move`. Ejecutar esa opción únicamente contra el simulador F1TENTH de un agente con la configuración de esta etapa. El script usa tiempo real para limitar la duración del diagnóstico y medir entrega de mensajes; no implementa el cronómetro de carrera.

La ruta del mapa en `sim.yaml` sigue siendo absoluta y específica de este equipo. Debe adaptarse al clonar en otra ubicación. La configuración `use_sim_time` del servidor y su lifecycle manager proviene del launch base; el sistema de reloj completo se resolverá en la etapa 3, antes de usarlo para medir vueltas.

## Límites del backend

Parámetros por defecto encontrados en el backend: coche de 0.58 × 0.31 m, dirección limitada a ±0.4189 rad y velocidad máxima de 20 m/s. Son límites del modelo, no reglas del profesor ni velocidades recomendadas. La simulación integra pasos nominales de 0.01 s; la entrega de sensores a aproximadamente 250 Hz incluye publicaciones repetidas entre pasos.

Siguiente hito: exponer y validar tiempo del simulador, colisiones y reinicios antes de implementar el cronometraje de carrera.

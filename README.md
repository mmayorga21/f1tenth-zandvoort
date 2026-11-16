# F1TENTH — Follow the Gap en Zandvoort

## 1. Descripción del enfoque

El proyecto utiliza Follow the Gap, un algoritmo de navegación reactiva que decide la dirección y la velocidad del vehículo a partir de las distancias medidas por el LiDAR.

El controlador procesa las lecturas, aplica un margen de seguridad alrededor de los obstáculos y busca espacios libres contiguos. Dentro del espacio seleccionado elige un objetivo hacia el cual orientar el vehículo. La velocidad se adapta al giro requerido y a la distancia disponible frente al coche.

Este enfoque permite responder al entorno observado durante el recorrido de Zandvoort sin utilizar una trayectoria predefinida para conducir.

## 2. Estructura del código

Los archivos principales del proyecto se organizan de la siguiente manera:

```text
F1Tenth-Repository/
├── README.md
├── scripts/                         # Scripts de comprobación
└── src/
    ├── f1tenth_controller/
    │   ├── config/
    │   │   ├── follow_the_gap.yaml   # Parámetros del controlador
    │   │   ├── conservative.yaml     # Perfil de velocidad conservador
    │   │   └── lap_tracker.yaml      # Configuración del contador
    │   ├── f1tenth_controller/
    │   │   ├── gap.py                # Algoritmo Follow the Gap
    │   │   ├── gap_node.py           # Comunicación del controlador con ROS
    │   │   ├── control_gate.py       # Activación y desactivación del control
    │   │   ├── laps.py               # Conteo de vueltas y cronometraje
    │   │   ├── lap_node.py           # Nodo ROS del contador
    │   │   ├── lap_panel.py          # Presentación de tiempos en terminal
    │   │   ├── lap_display_node.py   # Visualizador adicional del panel
    │   │   └── session_guard.py      # Comprobaciones de los intentos
    │   ├── launch/                   # Archivos de lanzamiento
    │   └── test/                     # Pruebas del controlador y contador
    └── f1tenth_gym_ros/
        ├── config/
        │   └── sim.yaml             # Configuración del simulador
        ├── maps/
        │   ├── Zandvoort_map.png     # Imagen de la pista
        │   └── Zandvoort_map.yaml    # Resolución y origen del mapa
        ├── f1tenth_gym_ros/
        │   ├── gym_bridge.py         # Integración del simulador con ROS
        │   └── simulation_session.py # Estados y finalización del intento
        └── launch/
            ├── gym_bridge_launch.py  # Inicio del simulador y RViz
            └── gym_bridge.rviz       # Configuración de visualización
```

El repositorio también conserva otros paquetes del simulador original. La ejecución de este proyecto utiliza principalmente `f1tenth_controller` y `f1tenth_gym_ros`.

## 3. Instrucciones de ejecución

Estas instrucciones parten de un equipo con **Ubuntu 22.04, ROS 2 Humble y F1TENTH Gym instalados**, además de las herramientas `rosdep` y `colcon`.

### 3.1. Clonar y preparar el proyecto

Abrir una terminal y clonar el repositorio:

```bash
cd ~
git clone https://github.com/mmayorga21/f1tenth-zandvoort.git F1Tenth-Repository
cd ~/F1Tenth-Repository
```

Antes de compilar, abrir la configuración del simulador:

```bash
nano src/f1tenth_gym_ros/config/sim.yaml
```

Modificar `map_path` para que coincida con la ubicación del proyecto en el equipo. Por ejemplo, reemplazar `TU_USUARIO` por el nombre de usuario de Ubuntu:

```yaml
map_path: '/home/TU_USUARIO/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Zandvoort_map'
```

La ruta debe escribirse **sin la extensión** `.png` o `.yaml`.

Instalar las dependencias de ROS y compilar:

```bash
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

colcon build --symlink-install
source install/setup.bash
```

### 3.2. Iniciar el simulador

En la **terminal 1**, ejecutar:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Esperar a que RViz muestre el mapa de Zandvoort y el vehículo. Mantener esta terminal abierta.

https://github.com/user-attachments/assets/1d38aa8d-d977-45f5-8e1f-aa6f435003ab

### 3.3. Iniciar el controlador y el contador

Abrir una **terminal 2** para el controlador:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch f1tenth_controller follow_the_gap.launch.py
```

El controlador se inicia desactivado. El vehículo permanecerá detenido hasta que se dé la orden de inicio.

https://github.com/user-attachments/assets/fda90ad6-1573-44f2-9abe-b83f9fd9939c

Abrir una **terminal 3** para el contador y el cronómetro:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run f1tenth_controller lap_tracker --ros-args \
  --params-file install/f1tenth_controller/share/f1tenth_controller/config/lap_tracker.yaml
```

Esta terminal mostrará las vueltas completadas, el tiempo de cada vuelta, la mejor vuelta y el tiempo acumulado. Mantener abiertos el controlador y el contador durante el intento.

https://github.com/user-attachments/assets/42a19ed6-84ac-4e24-83c3-7dfc1afe9e51

### 3.4. Ejecutar el intento de diez vueltas

Abrir una **terminal 4**:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Reiniciar el simulador para colocar el vehículo en la posición inicial y preparar un nuevo intento:

```bash
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
```

Cuando el reinicio haya terminado, activar el controlador:

```bash
ros2 service call /controller/enable std_srvs/srv/SetBool "{data: true}"
```

El vehículo comenzará a recorrer la pista de forma autónoma. Al completar diez vueltas válidas, frenará y el intento finalizará automáticamente. El panel conservará los tiempos y mostrará el mensaje de finalización.

Para realizar otro intento, volver a ejecutar los comandos de reinicio y activación en ese orden.

El siguiente video muestra los comandos de inicio y los primeros segundos de conducción:

https://github.com/user-attachments/assets/1b382e8a-57e8-459e-8228-c0e61c74e8b7

### 3.5. Detener la ejecución

Para detener el vehículo antes de completar las diez vueltas:

```bash
ros2 service call /controller/enable std_srvs/srv/SetBool "{data: false}"
```

Para cerrar los programas, presionar `Ctrl+C` en las terminales del controlador, del contador y del simulador.

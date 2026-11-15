# Procedencia y atribución

## Código base

Este proyecto parte de https://github.com/widegonz/F1Tenth-Repository.git, revisión inicial `7e79e59`. Se conserva el historial Git y la documentación original. Los autores del código base no deben confundirse con el autor de los cambios del proyecto.

El puente `f1tenth_gym_ros` contiene su [licencia MIT original](../src/f1tenth_gym_ros/LICENSE), copyright de Hongrui Zheng. Se conservan los avisos y licencias existentes; esta nota no asigna una licencia nueva a todo el repositorio.

## Avances locales previos

- `src/square_driver/`: trabajo previo de recorrido rectangular; no cumple por sí mismo el nuevo objetivo de Follow the Gap.
- `src/f1tenth_gym_ros/config/sim.yaml`: configuración local del equipo, todavía con LevineB.
- `src/f1tenth_gym_ros/maps/Zandvoort_map.*`: archivos suministrados para la pista asignada, conservados sin modificaciones.

SHA-256 de los mapas:

```text
7d2dbc9682152f5567892752202e27c823856e4440e011fdc7e87797b31791c0  Zandvoort_map.png
cec2760ad8e3b9deb36ca66bec56c17b6c6d02f358f05b3eca6e1eeb1e946d7b  Zandvoort_map.yaml
```

## Documentación de continuidad

El enunciado académico y el contexto de la conversación anterior se usan como referencias de trabajo locales. Sus contenidos no se publican automáticamente. Las propuestas de ese contexto no se consideran implementadas sin evidencia.

## Implementación de la etapa 4

El paquete `src/f1tenth_controller` contiene la implementación propia del proyecto, con lógica reactiva, adaptador ROS, configuración y pruebas. Su licencia se incluye en el propio paquete. Las regiones geométricas de `scripts/lap_validation.py` sirven exclusivamente para validar experimentos y no se usan como trayectoria de conducción.

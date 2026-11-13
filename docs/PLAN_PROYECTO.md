# Plan del proyecto

Prioridad: fiabilidad de 10 vueltas, medición correcta, desempeño y evidencia. Cada hito debe dejar cambios revisables, un commit y evidencia de su verificación.

| Etapa | Trabajo | Criterio de aceptación |
| --- | --- | --- |
| 1. GitHub | Preservar avances, crear repositorio personal, publicar estado inicial e invitar al profesor | Repositorio accesible y estado de invitación registrado |
| 2. Zandvoort | Verificar entorno, mapa de simulación y ROS, pose, rumbo, LiDAR y mando | Movimiento controlado y configuración efectiva comprobados |
| 3. Medición | Resolver tiempo del simulador, colisiones, reinicios y registro de intentos | Inicio, duración y validez de cada intento identificables |
| 4. Follow the Gap | Implementar algoritmo LiDAR, márgenes, objetivo, dirección, velocidad y parada ante pérdida de datos | Una vuelta sin choque con parámetros guardados |
| 5. Telemetría | Contador, meta con sentido y antirrebote, cronómetro y visualización | Conteo y tiempos correctos; reinicios y choques gestionados |
| 6. Robustez | Progresar por 1, 2, 5 y 10 vueltas y repetir sesiones | Diez vueltas sin colisión con evidencia reproducible |
| 7. Desempeño | Comparar ajustes con una configuración estable de referencia | Mejora medida de tiempos conservando fiabilidad |
| 8. Entrega | Tutorial reproducible, video y preparación oral | Evidencia completa e instrucciones comprobadas |

## Registro de experimentos previsto

Cada intento registrará revisión del código, configuración, mapa, pose inicial, fuente de tiempo, tiempos por vuelta, total, mejor vuelta, colisiones y motivo de finalización. No se añadirán resultados estimados como si fueran medidos.

## Verificaciones de especial importancia

- No asumir que la existencia de `/clock` significa que tiene un publicador operativo.
- Usar el estado de colisión del simulador; no inferir ausencia de choques solo por inspección visual.
- Evitar doble conteo, vueltas en sentido inverso y conteos causados por reinicios o saltos de pose.
- Mantener la conducción reactiva basada en LiDAR; el contador puede usar odometría.
- Conservar una configuración estable antes de optimizar.
- Excluir archivos generados y credenciales del repositorio.

## Pendientes de coordinación

- Repositorio acordado: `mmayorga21/f1tenth-zandvoort`, privado. Creado en GitHub. El historial del proyecto se publica en la rama `main`; el repositorio base se conserva como remoto `upstream`.
- Invitación al profesor aplazada por el estudiante hasta disponer de su usuario o correo.
- Fecha de entrega y reglas de salida, sentido, cronometraje y colisión.
- Restricciones sobre cambios al puente y configuraciones entre modalidades.
- Aclaración de la rúbrica: el enunciado menciona tanto 25+5 como 20+10 puntos.

## Estado inicial comprobado por lectura

- Los mapas originales coinciden por SHA-256 con los copiados al workspace.
- Al iniciar la etapa 1, la configuración seleccionaba LevineB. Ahora usa Zandvoort, validado en la etapa 2.
- El launch lee `sim.yaml` desde el share del paquete; la copia instalada actual resuelve al archivo fuente.
- `controllers/gap_node.py` es una plantilla sin implementación completa.
- El puente inspeccionado no publica reloj ROS ni expone actualmente la información de colisión mediante un tópico dedicado.
- El remoto original es `https://github.com/widegonz/F1Tenth-Repository.git`.

Estas comprobaciones no sustituyen una prueba de ejecución del simulador.

## Hitos completados

- Etapa 1: repositorio privado publicado y sincronizado. Invitación al profesor aplazada por el estudiante.
- Etapa 2: Zandvoort, pose inicial, mapa ROS, sensores, avance, giro, parada y retorno comprobados. Véase [el informe](VALIDACION_ZANDVOORT.md). Reloj y colisiones por ROS incorporados en la etapa 3.
- Etapa 3: reloj de física, colisión retenida, pausa, reset, cierre de intentos y registro JSONL comprobados. Seis pruebas unitarias y 23 comprobaciones ROS pasadas; cierre y reinicio real del proceso verificados. Véase [la documentación](MEDICION_INTENTOS.md).

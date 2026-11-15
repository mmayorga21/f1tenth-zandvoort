# F1TENTH — Follow the Gap en Zandvoort

## Descripción del enfoque

El proyecto utiliza Follow the Gap, un algoritmo de navegación reactiva que decide la dirección y la velocidad del vehículo a partir de las distancias medidas por el LiDAR.

El controlador procesa las lecturas, aplica un margen de seguridad alrededor de los obstáculos y busca espacios libres contiguos. Dentro del espacio seleccionado elige un objetivo hacia el cual orientar el vehículo. La velocidad se adapta al giro requerido y a la distancia disponible frente al coche.

Este enfoque permite responder al entorno observado durante el recorrido de Zandvoort sin utilizar una trayectoria predefinida para conducir.

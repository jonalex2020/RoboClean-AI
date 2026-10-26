# RoboClean AI — Simulación educativa de aspiradora robótica

## Objetivo
Simular percepción, localización, mapeo probabilístico, planificación A*, seguimiento y detección de anomalías en un agente inteligente.

## Requisitos
Python 3.10 o superior, con Tkinter. No requiere instalar paquetes con pip.

## Ejecución
Desde una terminal, en la carpeta del proyecto:

```bash
python roboclean_ai.py
```

En algunas distribuciones Linux se debe instalar el paquete de sistema `python3-tk`.

## Interfaz y pruebas
- **Iniciar**: activa navegación y limpieza automáticas.
- **Pausar**: detiene temporalmente el agente.
- **Reiniciar**: restablece obstáculos, polvo, probabilidades y batería.
- **Anomalía**: inserta un obstáculo inesperado.
- **Clic en el mapa**: agrega o elimina obstáculos, sin modificar paredes exteriores.
- El panel derecho muestra coordenadas, batería, pasos, objetivo, progreso, mapa y anomalías.

## Diseño de IA
- **Percepción:** cuatro sensores de alcance (cinco celdas), con posibilidad de errores de observación.
- **Localización:** la posición del agente se conoce exactamente desde el motor de simulación; no se implementa filtrado probabilístico de posición.
- **Mapeo:** cuadrícula de probabilidades de ocupación; inicia en 0.5 y actualiza con evidencia mediante log-odds.
- **Planificación:** ruta por A* y selección de la suciedad alcanzable más cercana.
- **Seguimiento:** traza histórica de posiciones y métricas.
- **Anomalías:** nuevas colisiones previstas y baja batería.

## Prompt de desarrollo para la bitácora académica
"Desarrolla un simulador educativo en Python de un robot aspirador inteligente con visualización 2D. Implementa sensores virtuales, mapa de ocupación probabilístico actualizado con observaciones, planificación A*, recorrido visual, limpieza de objetivos, panel de variables y detección de obstáculos inesperados y batería baja. Incluye controles para iniciar, pausar, reiniciar y añadir obstáculos, más instrucciones de ejecución y limitaciones técnicas."

## Limitaciones que deben declararse en la exposición
Esta es una simulación didáctica: no contiene LIDAR físico, SLAM completo, red neuronal ni un modelo cinemático continuo. El entorno se discretiza en celdas; la posición exacta se conoce por el motor, y el agente decide rutas según la estimación del mapa. Las observaciones contienen ruido, pero las colisiones son evitadas mediante verificación física del siguiente movimiento.

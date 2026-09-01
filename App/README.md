# App/ — aplicación de control (PC)

`CINEMATICA_3D_VISUAL_py.py` es la aplicación que corre en el ordenador y desde la que se maneja todo el sistema: calcula la cinemática del brazo, visualiza el robot en 3D, permite moverlo manualmente o por rutinas programadas, y es quien manda los setpoints al Central por puerto serie. Está escrita en Python con PyQt5 para la interfaz y PyVista/VTK para el visor 3D, y es una única clase de unas 6900 líneas — pensada para funcionar tanto en un PC de escritorio como en una pantalla táctil de 800x480 sobre una Raspberry Pi 5, que es justo la variante escalada que contiene este fichero (hay otra versión, `CINEMATICA_3D_VISUAL.py`, con la interfaz a tamaño de escritorio normal; la lógica de ambas debe mantenerse igual a mano).

## La clase principal: `BrazoRobot`

Es una `QMainWindow` que organiza toda la interfaz en pestañas, cada una centrada en una tarea:

- **Comunicación serie y configuración de nodos** — selección de puerto y conexión con el Central, y el diálogo `NodeConfigDialog` para ajustar en caliente el PID, las velocidades y los límites de cada articulación (con `ResponseGraph` pintando en vivo el setpoint contra la posición real).
- **Control manual y visualización 3D** — mover cada articulación con sliders o campos numéricos, ver el robot en el visor 3D (construido a partir de los meshes `P1.obj`...`P7.obj`) y trabajar directamente en coordenadas XYZ en vez de ángulos.
- **Rutinas por bloques** — el editor visual (`BlockContainer` / `BlockWidget`) donde cada paso de una rutina es un bloque que se puede añadir, arrastrar o eliminar: mover a una posición, pausas, bucles, condicionales sobre el estado de la cinta, arranque/parada de la cinta. `FlowchartDialog` genera automáticamente un diagrama de flujo de la rutina cargada.
- **Diagnóstico, calibración y registro** — el asistente de calibración (`_open_calibration_wizard`), la comparación entre lo que predice la cinemática directa y la posición real medida sobre el modelo CAD, el diálogo `CintaConfigDialog` para la cinta transportadora, y el registro de eventos y errores del sistema.

## Cinemática

La cinemática directa e inversa vive en un único sitio, reutilizado por el visor 3D, el control manual y las rutinas: la convención empleada es Denavit-Hartenberg Modificada (MDH), con los parámetros reales del robot y una matriz de alineación final para que las coordenadas que ve y edita el usuario coincidan con las del modelo 3D diseñado en Fusion 360. La inversa se resuelve con un método numérico iterativo (mínimos cuadrados amortiguados, DLS) en vez de una solución analítica cerrada. Todo este desarrollo, con las matrices y ejemplos numéricos completos, está documentado en el apartado 3.5 de la memoria.

## Cómo se ejecuta una rutina sin bloquear la interfaz

El botón Run no recorre la rutina de golpe: `_build_prog_gen()` construye un generador Python que va produciendo un bloque cada vez que se le pide, y un `QTimer` de 25 ms (`_prog_tick()`) lo recorre mediante una pequeña máquina de estados. Cada bloque de "Mover a" separa dos cosas que a primera vista parecen la misma: el setpoint real se manda por serie al hardware en cuanto empieza el bloque, mientras que la animación del brazo en el visor 3D interpola hacia esa misma posición a un ritmo fijo, de forma completamente independiente. Quien decide cuándo pasar al siguiente bloque es `_prog_settle_tick()`, comparando los ángulos reales que reporta el Central contra el objetivo, no la animación.

## Persistencia

La configuración (parámetros de nodos, perfiles de herramienta, rutinas) se guarda en ficheros JSON, con copia de seguridad y restauración disponibles desde la propia interfaz (`_backup_config()` / `_restore_config()`).

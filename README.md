# Brazo Robótico de 6 GDL — Software

Firmware y aplicación de control de un brazo robótico de 6 grados de libertad, construido con motores paso a paso NEMA17, encoders magnéticos AS5600 y una red de microcontroladores ESP32 que se comunican entre sí por ESP-NOW.

## Qué hay en cada carpeta

### `App/`

La aplicación de control que corre en el PC (`CINEMATICA_3D_VISUAL_py.py`). Desde aquí se calcula la cinemática directa e inversa, se programan rutinas por bloques, se visualiza el robot en 3D y se envían los setpoints al resto del sistema por puerto serie. Es el punto de entrada habitual para manejar el brazo.

### `Robot/`

El firmware de las seis articulaciones y del nodo que las coordina.

- **`Central/`** — Puente entre el PC y el resto de la electrónica: recibe los comandos por Serie y los reparte por ESP-NOW a cada articulación, a la cinta y al mando. También centraliza la parada de emergencia y la baliza de señalización.
- **`Articulacion_1/` a `Articulacion_6/`** — El firmware que corre en cada eje. Cada una implementa su propio lazo de control PID en tiempo real sobre un motor NEMA17, con el encoder AS5600 como realimentación de posición y un driver TB6600 para la potencia.

### `Cinta/`

Firmware de la cinta transportadora (ESP32-C3): control de velocidad en lazo abierto del motor paso a paso y lectura del sensor ultrasónico que detecta piezas a su paso.

### `Mando/`

Firmware del pendant de control (ESP32-S3): pantalla táctil Nextion, pulsadores físicos y enlace ESP-NOW con el Central, pensado para mover el robot y lanzar rutinas sin depender del PC.

## Cómo se comunican los nodos

```
PC  ──Serie (USB)──▶  Central  ──ESP-NOW──▶  Articulaciones 1-6, Cinta, Mando
```

Cada articulación cierra su propio lazo de posición de forma local a 1 ms; el Central y el PC solo intercambian setpoints y estado a un ritmo mucho más bajo, sin intervenir en el control de bajo nivel de cada eje.

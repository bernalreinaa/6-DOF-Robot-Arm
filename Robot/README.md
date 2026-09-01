# Robot/ — Central y las 6 articulaciones

Aquí vive el firmware que mueve físicamente el brazo: el nodo **Central** (ESP32-S3), que hace de puente con el PC, y las seis **Articulaciones** (ESP32-C3), una por cada eje del robot. Los dos diagramas de esta carpeta (`Diagrama_Flujo_Nodo_Central.png` y `Diagrama_Flujo_Articulación.png`) resumen visualmente lo que se explica aquí abajo.

## Central/

El Central no controla ningún motor directamente: su trabajo es traducir. Por un lado escucha al PC por puerto Serie (115200 baudios) con un protocolo de texto simple — `SP[1]=10.0;SP[2]=20.0;...;V=75;` para mandar setpoints a varias articulaciones a la vez, `init[3]...` para reprogramar el PID de una articulación, `cinta=1;vel=75` para arrancar la cinta, `bomba=1` para la ventosa, etc. Por otro lado escucha por ESP-NOW a las seis articulaciones, a la cinta y al mando, y sabe distinguir qué tipo de mensaje ha llegado de cada uno simplemente mirando su tamaño en bytes (`sizeof`), sin necesidad de una cabecera extra.

`main.cpp` es solo el punto de montaje: crea un objeto por cada responsabilidad y los conecta entre sí. Toda la lógica de verdad vive en `lib/`:

- **JointBus** — el canal de ida y vuelta con las 6 articulaciones: manda setpoints, resets y parámetros de ajuste (PID, velocidades, zonas), y recibe de vuelta el ángulo que reporta cada una en continuo.
- **MandoBus** — el mismo tipo de canal pero con el mando físico, al que le reporta los ángulos cada 100 ms para que los pinte en su pantalla.
- **ConveyorBus** — habla con la cinta transportadora: arranque/parada, velocidad y el umbral de distancia del sensor de piezas.
- **EmergencyStop** — la seta de emergencia física, leída por interrupción para no perder ni un pulsado, combinada con la seta remota que puede activar el mando por ESP-NOW. Basta que una de las dos esté pulsada para que el sistema quede parado; hace falta que las dos estén liberadas para volver a moverse.
- **StatusBeacon** — la baliza luminosa (rojo/ámbar/verde) y el zumbador, que reflejan si hay una emergencia activa, si algo se está moviendo o si el sistema está libre.
- **CentralOta** — actualización de firmware por WiFi bajo demanda. Es la única placa del sistema que no puede quedarse bloqueada esperando una subida de firmware, porque tiene que seguir vigilando la seta de emergencia en todo momento; por eso su OTA se atiende dentro del propio `loop()` en vez de congelarlo.
- **CentralConfig** — un único fichero con los pines físicos y las direcciones MAC de todos los nodos del sistema.
- **EspNowProtocol** / **ConveyorProtocol** — las estructuras de datos que viajan por ESP-NOW. Tienen que ser exactamente iguales, campo a campo, en los ocho firmwares que las usan (Central, Mando y las 6 articulaciones), porque el sistema entero identifica el tipo de cada mensaje por su tamaño en bytes en lugar de por una cabecera.

`loop()` no usa tareas de FreeRTOS propias: es un único bucle cooperativo que, en cada vuelta, atiende la seta de emergencia, procesa un posible comando del PC, reenvía los setpoints que haya podido pedir el mando, y manda sus dos reportes periódicos (al PC cada 100 ms con los ángulos de las 6 articulaciones y el estado de la cinta, y al mando cada 100 ms solo con los ángulos).

## Articulacion_1/ … Articulacion_6/

Las seis carpetas contienen exactamente el mismo firmware — mismo `main.cpp`, mismas librerías — con una única diferencia entre ellas: el fichero `lib/JointConfig/JointConfig.h`, que fija los pines, la relación de reductora, las ganancias del PID y los límites de esa articulación en concreto. Documentar una vale para las seis.

Cada articulación cierra su propio lazo de control de posición de forma local, sin depender de la latencia del enlace inalámbrico, repartido en dos tareas de FreeRTOS que corren en paralelo sobre el único núcleo del ESP32-C3:

- **`encoderTask` (cada 10 ms)** — lee el encoder magnético **AS5600** a través de `MagneticEncoder`, que lleva la cuenta de vueltas completas para dar un ángulo absoluto (puede superar los 360°) en vez del valor crudo módulo-360 del sensor. Con ese ángulo actualiza el display OLED, manda el ángulo actual al Central por ESP-NOW, y comprueba si ha llegado una petición de reset.
- **`motionTask` (a 1 ms, cuando hay un movimiento en curso)** — toda la lógica vive en `JointMotionController::stepOnce()`: calcula el error respecto al setpoint, aplica un PID clásico con anti-windup, recorta la velocidad resultante con un perfil de tres tramos (`AngleMath::velocityProfile()`, arranque suave / crucero / aproximación), y con esa velocidad genera el pulso PUL/DIR que `StepperDriver` manda al driver **TB6600** del motor **NEMA17**. Antes de arrancar, `beginMove()` calcula el camino más corto que evita la zona prohibida configurada y compensa el backlash mecánico si la articulación acaba de cambiar de sentido.

El resto de librerías dan soporte a estas dos tareas: `JointStorage` guarda los parámetros de ajuste en flash (NVS) para no perderlos al reiniciar, `JointOta` gestiona la actualización de firmware por WiFi con una ventana de 30 s al arrancar, y `JointDisplay` es el envoltorio de la pantalla OLED SH1106.

## Cómo se comunican entre sí

```
PC ──Serie──▶ Central ──ESP-NOW──▶ Articulacion_1 ... Articulacion_6
                  ▲                        │
                  └────────ESP-NOW─────────┘  (ángulo cada 10 ms)
```

El Central nunca calcula ni corrige una posición: solo reenvía. Cada articulación decide, por sí misma y en tiempo real, cómo llegar a su setpoint.

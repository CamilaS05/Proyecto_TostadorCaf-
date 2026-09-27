# Tostador de Café ESP32 - IoT Level 1

Proyecto ESP-IDF para tostadora de café de bajo costo con ESP32, FreeRTOS, arquitectura de 3 capas, seguridad funcional básica, registro en SD y documentación Doxygen.

## Hardware base
- ESP32 DevKit 30 pines
- MAX6675 + termocupla tipo K
- SSR-40DA + placa calefactora 110 VAC
- Motorreductor N20 12 V + módulo MOSFET LR7843
- Ventilador 12 V + MOSFET
- MicroSD SPI
- Fuente 12 V + LM2596 5 V

## Compilar
```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

## Capas
- Interacción: consola serial provisional. Se puede ampliar a LCD/botones.
- Procesamiento/Servicios: configuración, proceso, monitoreo, control, seguridad, registro, supervisión.
- Física: drivers MAX6675, SSR, motor, ventilador y SD.

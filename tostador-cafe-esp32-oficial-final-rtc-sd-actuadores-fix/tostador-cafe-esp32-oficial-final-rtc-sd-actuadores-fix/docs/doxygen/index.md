# Tostadora automática de café con ESP32

## Descripción general

Este proyecto implementa el firmware de una tostadora automática de café a escala de laboratorio usando ESP32, ESP-IDF y FreeRTOS.

El sistema integra lectura de sensores, control automático de actuadores, visualización local y registro en microSD.

## Componentes funcionales

- ESP32 DevKit.
- Dos MAX6675 con termocuplas tipo K.
- HX711 con celda de carga en canal B.
- OLED SSD1306 por I2C.
- RTC DS3231 por I2C.
- MicroSD por SPI.
- Encoder KY-040.
- Botones START y STOP.
- Resistencia calefactora PTC de 12 V.
- Motor DC de 12 V.
- Ventilador DC de 12 V.
- MOSFETs LR7843 o similares para control de potencia.

## Modos de operación

- **IDLE:** sistema en reposo, actuadores apagados.
- **ROASTING:** tostión activa, motor encendido y calefactor regulado por temperatura.
- **COOLING:** enfriamiento, ventilador encendido.
- **EMERGENCY:** emergencia por falla o sobretemperatura.
- **FINISHED:** proceso finalizado.

## Tareas FreeRTOS

- `monitoring_task`: lectura de sensores.
- `control_task`: control automático y seguridad.
- `buttons_task`: lectura de START y STOP.
- `encoder_task`: ajuste de temperatura objetivo.
- `oled_task`: actualización de pantalla.
- `logging_task`: registro en microSD.

## Recursos compartidos

El firmware usa mutex para proteger datos globales y el bus I2C. También usa una cola FreeRTOS para desacoplar la lectura de sensores del registro en microSD.

# Tostadora de Café Automática ESP32 + FreeRTOS

Versión oficial final del proyecto, integrando lo trabajado durante las pruebas:

- ESP32 con FreeRTOS.
- MAX6675 #1 para temperatura del café.
- MAX6675 #2 para temperatura del tambor/cámara.
- HX711 corregido en canal B, ganancia 32, para celda de carga de 1 kg.
- OLED SSD1306 128x64 I2C mostrando estado, temperaturas, peso y actuadores.
- Botones START y STOP en GPIO34 y GPIO35 con pull-up externo de 10 kΩ.
- Encoder KY-040 en GPIO16, GPIO17 y GPIO4.
- MicroSD SPI para registro CSV.
- RTC DS3231 por I2C para fecha/hora real en terminal y CSV.
- Actuadores automáticos por MOSFET:
  - GPIO25: resistencia calefactora.
  - GPIO26: motor.
  - GPIO27: ventilador.
  - GPIO13: relé/salida auxiliar.
- SHT31/SHT3x contemplado en el código, pero deshabilitado porque aún no está conectado.

## Conexiones principales

### HX711 canal B

```txt
HX711 VCC  -> ESP32 3.3V
HX711 GND  -> ESP32 GND
HX711 DT   -> ESP32 GPIO32
HX711 SCK  -> ESP32 GPIO33

Celda rojo   -> E+
Celda negro  -> E-
Celda verde  -> B+
Celda blanco -> B-

A+ y A- quedan libres.
```

### OLED SSD1306

```txt
OLED VCC -> 3.3V
OLED GND -> GND
OLED SDA -> GPIO21
OLED SCL -> GPIO22
Direccion I2C: 0x3C o 0x3D, detectada automaticamente
```

### RTC DS3231

```txt
DS3231 VCC -> 3.3V o 5V segun modulo
DS3231 GND -> GND
DS3231 SDA -> GPIO21
DS3231 SCL -> GPIO22
Direccion I2C: 0x68

Comparte el bus I2C con la OLED y el SHT31 futuro.
El codigo no configura la hora del RTC; usa la hora que ya tenga grabada el modulo.
Si el RTC no responde, el sistema continua y en CSV aparece RTC_OFF.
```

### MAX6675

```txt
SCK compartido -> GPIO18
SO/MISO        -> GPIO19
CS cafe        -> GPIO15
CS tambor      -> GPIO14
VCC            -> 3.3V
GND            -> GND
```

### MicroSD

```txt
CS   -> GPIO5
SCK  -> GPIO18
MISO -> GPIO19
MOSI -> GPIO23
VCC  -> segun modulo, 3.3V o 5V
GND  -> GND
```

### Botones

GPIO34 y GPIO35 necesitan pull-up externo.

```txt
START:
3.3V -> 10kΩ -> GPIO34
GPIO34 -> boton START -> GND

STOP:
3.3V -> 10kΩ -> GPIO35
GPIO35 -> boton STOP -> GND
```

### KY-040

```txt
CLK -> GPIO16
DT  -> GPIO17
SW  -> GPIO4
+   -> 3.3V
GND -> GND
```

### Actuadores por MOSFET

Para cada MOSFET:

```txt
GPIO -> 220Ω -> Gate
Gate -> 10kΩ -> GND
Source -> GND fuente 12V
GND ESP32 -> GND fuente 12V
```

Resistencia calefactora:

```txt
+12V -> fusible -> calefactor -> Drain MOSFET
Source MOSFET -> GND 12V
GPIO25 -> 220Ω -> Gate
Gate -> 10kΩ -> GND
Diodo: NO necesita
```

Motor:

```txt
+12V -> fusible -> motor -> Drain LR7843
Source LR7843 -> GND 12V
GPIO26 -> 220Ω -> Gate
Gate -> 10kΩ -> GND
Diodo 1N4007 en paralelo con el motor:
  raya/catodo -> +12V
  sin raya/anodo -> negativo del motor / Drain
```

Ventilador:

```txt
+12V -> fusible -> ventilador -> Drain LR7843
Source LR7843 -> GND 12V
GPIO27 -> 220Ω -> Gate
Gate -> 10kΩ -> GND
Diodo 1N4007 en paralelo con el ventilador:
  raya/catodo -> +12V
  sin raya/anodo -> negativo del ventilador / Drain
```

## Lógica de funcionamiento

- START inicia el tostado automático.
- En tostado:
  - El motor permanece encendido.
  - La resistencia se enciende o apaga según la temperatura objetivo.
  - El ventilador se enciende si la temperatura del tambor supera el límite definido.
- STOP durante tostado pasa a enfriamiento.
- En enfriamiento:
  - Resistencia apagada.
  - Motor apagado.
  - Ventilador encendido.
- En emergencia por termocupla inválida o sobretemperatura:
  - Resistencia apagada.
  - Motor apagado.
  - Ventilador encendido.

## Compilación

Desde la carpeta del proyecto:

```bash
rm -rf build sdkconfig sdkconfig.old
idf.py set-target esp32
idf.py reconfigure
idf.py build -j 1
```

Para flashear:

```bash
idf.py -p COM3 flash monitor
```

o en Linux:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

## Nota importante

La microSD está implementada. Si no se monta correctamente, el sistema continúa funcionando por consola y OLED.
El RTC DS3231 está implementado. Si no se detecta en 0x68, el sistema continúa y registra `RTC_OFF`.
El SHT31 está dejado como futuro módulo y no bloquea el arranque.

## CSV generado en microSD

El archivo se crea en:

```txt
/sdcard/tostador.csv
```

Columnas:

```txt
datetime,ms,state,cafe_c,tambor_c,peso_g,hx_ok,heater,motor,fan,target_c,rtc_ok
```

Ejemplo:

```txt
2026-10-04 20:15:02,15320,ROAST,145.25,170.00,120.50,1,1,1,0,205.0,1
```

## Ajuste de actuadores y RTC/SD
Esta versión mantiene:
- GPIO25: resistencia calefactora automática por MOSFET.
- GPIO26: motor automático por LR7843.
- GPIO27: ventilador automático por LR7843.
- DS3231 por I2C en GPIO21/GPIO22.
- microSD por SPI con CS GPIO5.

Corrección aplicada: el control ya no entra en emergencia antes de tener la primera muestra de sensores. Además imprime periódicamente el estado real de actuadores:
`CONTROL estado=ROAST sample=1 cafe=... tambor=... -> RES=ON MOTOR=ON FAN=OFF`.

Para probar actuadores en el sistema completo: presionar START en GPIO34. Si se quiere arranque automático temporal, cambiar `AUTO_START_FOR_TESTS` a 1 en `main/main.c` y volver a compilar. Para entrega dejarlo en 0.

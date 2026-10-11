/**
 * @file main_tostadora_doxygen.c
 * @brief Firmware principal de una tostadora automática de café con ESP32 y FreeRTOS.
 *
 * Este archivo contiene la lógica funcional del prototipo de tostadora automática
 * de café. El sistema lee sensores, controla actuadores, actualiza una pantalla
 * OLED y registra datos en una tarjeta microSD en formato CSV.
 *
 * Componentes integrados:
 * - Dos termocuplas tipo K mediante módulos MAX6675.
 * - Celda de carga con HX711 usando canal B.
 * - Pantalla OLED SSD1306 por I2C.
 * - RTC DS3231 por I2C.
 * - Encoder KY-040.
 * - Botones físicos START y STOP.
 * - Resistencia calefactora, motor y ventilador controlados por MOSFET.
 * - Registro de datos en microSD.
 *
 * @author Equipo tostadora de café
 * @date 2026
 */




#include <stdio.h>

#include <string.h>

#include <stdbool.h>

#include <stdint.h>

#include <math.h>

#include <sys/stat.h>



#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "freertos/queue.h"

#include "freertos/semphr.h"



#include "esp_log.h"

#include "esp_err.h"

#include "esp_timer.h"

#include "esp_rom_sys.h"

#include "esp_vfs_fat.h"



#include "driver/gpio.h"

#include "driver/i2c_master.h"

#include "driver/spi_master.h"

#include "driver/sdspi_host.h"

#include "sdmmc_cmd.h"



// =====================================================

/** @name Configuración general del sistema
 *  @brief Parámetros de operación, periodos de tarea y calibración.
 *  @{ */
// CONFIGURACION GENERAL

// =====================================================



static const char *TAG = "TOSTADOR";



/** @brief Habilita o deshabilita el sensor SHT31. En el montaje actual queda deshabilitado. */
#define ENABLE_SHT31 0  // El sensor esta contemplado, pero aun no esta conectado.

// Si quieren que arranque solo para pruebas, cambiar a 1. Para entrega dejar en 0.

/** @brief Permite iniciar automáticamente el proceso para pruebas controladas. */
#define AUTO_START_FOR_TESTS 0



/** @brief Temperatura objetivo inicial de tostión en grados Celsius. */
#define TARGET_TEMP_C              205.0f

/** @brief Temperatura máxima segura antes de entrar en estado de emergencia. */
#define MAX_SAFE_TEMP_C            240.0f

/** @brief Histéresis del calefactor para evitar conmutación rápida. */
#define HEATER_HYSTERESIS_C        3.0f

/** @brief Temperatura de café para considerar terminado el enfriamiento. */
#define COOLING_SAFE_TEMP_C        60.0f

/** @brief Temperatura del tambor a partir de la cual se activa el ventilador en tostión. */
#define FAN_ON_DRUM_TEMP_C         190.0f



/** @brief Periodo de la tarea de monitoreo en milisegundos. */
#define MONITORING_PERIOD_MS       1000

/** @brief Periodo de la tarea de control en milisegundos. */
#define CONTROL_PERIOD_MS          500

/** @brief Periodo de actualización de la OLED en milisegundos. */
#define OLED_PERIOD_MS             500

/** @brief Tiempo máximo de espera de la tarea de registro en milisegundos. */
#define LOG_PERIOD_MS              1000

/** @brief Periodo de lectura de botones para debounce por software. */
#define BUTTON_PERIOD_MS           40

/** @brief Periodo de lectura del encoder KY-040. */
#define ENCODER_PERIOD_MS          20



// Calibracion inicial HX711 canal B. Ajustar con peso conocido.

/** @brief Factor inicial de calibración del HX711. Debe ajustarse con peso conocido. */
#define HX711_CALIBRATION_FACTOR   150.0f

#define HX711_AVERAGE_SAMPLES      5

#define HX711_SAMPLE_DELAY_MS      80

#define HX711_ZERO_DEADBAND_G      3.0f

#define HX711_MIN_VALID_WEIGHT_G   -50.0f

#define HX711_MAX_VALID_WEIGHT_G   1200.0f



// =====================================================

/** @} */

/** @name Asignación de pines
 *  @brief Pines usados por sensores, actuadores, buses y periféricos.
 *  @{ */
// PINES

// =====================================================



// I2C OLED y SHT31 futuro

/** @brief Pin SDA del bus I2C compartido. */
#define PIN_I2C_SDA      GPIO_NUM_21

/** @brief Pin SCL del bus I2C compartido. */
#define PIN_I2C_SCL      GPIO_NUM_22

#define OLED_ADDR        0x3C

#define SHT31_ADDR       0x44

#define DS3231_ADDR      0x68



// SPI compartido: MAX6675 y microSD

/** @brief Pin SCK del bus SPI compartido. */
#define PIN_SPI_SCK      GPIO_NUM_18

/** @brief Pin MISO/SO del bus SPI compartido. */
#define PIN_SPI_MISO     GPIO_NUM_19

/** @brief Pin MOSI del bus SPI, usado por la microSD. */
#define PIN_SPI_MOSI     GPIO_NUM_23

#define PIN_MAX_CAFE_CS  GPIO_NUM_15

#define PIN_MAX_TAMB_CS  GPIO_NUM_14

#define PIN_SD_CS        GPIO_NUM_5

#define SPI_HOST_USED    SPI2_HOST



// HX711 canal B

/** @brief Pin DT/DOUT del módulo HX711. */
#define PIN_HX711_DT     GPIO_NUM_32

/** @brief Pin SCK/PD_SCK del módulo HX711. */
#define PIN_HX711_SCK    GPIO_NUM_33



// KY-040

/** @brief Pin CLK del encoder KY-040. */
#define PIN_ENC_CLK      GPIO_NUM_16

/** @brief Pin DT del encoder KY-040. */
#define PIN_ENC_DT       GPIO_NUM_17

/** @brief Pin SW/pulsador del encoder KY-040. */
#define PIN_ENC_SW       GPIO_NUM_4



// Botones externos con pull-up externo 10k

/** @brief Pin del botón START con pull-up externo. */
#define PIN_BTN_START    GPIO_NUM_34

/** @brief Pin del botón STOP con pull-up externo. */
#define PIN_BTN_STOP     GPIO_NUM_35



// Actuadores por MOSFET

/** @brief Pin de control del MOSFET de la resistencia calefactora. */
#define PIN_HEATER       GPIO_NUM_25   // Calefactor / resistencia

/** @brief Pin de control del MOSFET del motor. */
#define PIN_MOTOR        GPIO_NUM_26   // Motor

/** @brief Pin de control del MOSFET del ventilador. */
#define PIN_FAN          GPIO_NUM_27   // Ventilador

/** @brief Salida auxiliar disponible para relé o expansión. */
#define PIN_RELAY_AUX    GPIO_NUM_13   // Salida auxiliar disponible



// =====================================================

/** @} */

/** @name Tipos de datos y estado global
 *  @brief Estructuras y enumeraciones usadas por la máquina de estados.
 *  @{ */
// TIPOS DE DATOS

// =====================================================



/**
 * @brief Estados principales del proceso de tostión.
 *
 * Esta enumeración implementa la máquina de estados del sistema.
 * Cada estado define el comportamiento de los actuadores y la respuesta
 * ante inicio, enfriamiento, finalización o emergencia.
 */
typedef enum {
    /** Sistema en reposo: sensores activos y actuadores apagados. */
    STATE_IDLE = 0,

    /** Proceso de tostión: motor activo y calefactor regulado por temperatura. */
    STATE_ROASTING,

    /** Enfriamiento: calefactor y motor apagados, ventilador encendido. */
    STATE_COOLING,

    /** Emergencia por falla de sensor o sobretemperatura. */
    STATE_EMERGENCY,

    /** Proceso finalizado: actuadores apagados. */
    STATE_FINISHED
} process_state_t;



/**
 * @brief Muestra de sensores del sistema.
 *
 * Agrupa las lecturas tomadas por la tarea de monitoreo. Esta estructura
 * se guarda como última muestra global y también se envía por cola a la
 * tarea de registro en microSD.
 */
typedef struct {
    /** Tiempo interno del ESP32 en milisegundos. */
    int64_t timestamp_ms;

    /** Fecha/hora desde DS3231 en formato YYYY-MM-DD HH:MM:SS, o RTC_OFF. */
    char datetime[20];

    /** Temperatura del café medida con MAX6675. */
    float coffee_temp_c;

    /** Temperatura del tambor medida con MAX6675. */
    float drum_temp_c;

    /** Indica si la lectura de café fue válida. */
    bool coffee_ok;

    /** Indica si la lectura del tambor fue válida. */
    bool drum_ok;

    /** Peso medido por HX711 en gramos. */
    float weight_g;

    /** Indica si la lectura del HX711 fue válida. */
    bool hx711_ok;

    /** Temperatura ambiente del SHT31, si está habilitado. */
    float ambient_c;

    /** Humedad relativa del SHT31, si está habilitado. */
    float humidity_percent;

    /** Indica si el SHT31 está disponible. */
    bool sht31_ok;
} sensor_sample_t;



/**
 * @brief Estado global del sistema de tostión.
 *
 * Contiene el estado de operación, las salidas lógicas de actuadores y
 * la temperatura objetivo configurada por defecto o mediante encoder.
 */
typedef struct {
    /** Estado actual de la máquina de estados. */
    process_state_t state;

    /** Estado lógico del calefactor. */
    bool heater_on;

    /** Estado lógico del motor. */
    bool motor_on;

    /** Estado lógico del ventilador. */
    bool fan_on;

    /** Estado lógico de la salida auxiliar. */
    bool relay_on;

    /** Temperatura objetivo del proceso. */
    float target_temp_c;

    /** Marca de tiempo del inicio de tostión. */
    int64_t roast_start_ms;

    /** Marca de tiempo del inicio de enfriamiento. */
    int64_t cooling_start_ms;
} system_state_t;



static sensor_sample_t g_last_sample = {0};

static bool g_has_sample = false;

static system_state_t g_state = {

    .state = STATE_IDLE,

    .heater_on = false,

    .motor_on = false,

    .fan_on = false,

    .relay_on = false,

    .target_temp_c = TARGET_TEMP_C,

    .roast_start_ms = 0,

    .cooling_start_ms = 0,

};



/**
 * @brief Mutex para proteger datos compartidos como g_state y g_last_sample.
 */
static SemaphoreHandle_t g_data_mutex = NULL;

/**
 * @brief Mutex para proteger el bus I2C compartido entre OLED, RTC y SHT31 futuro.
 */
static SemaphoreHandle_t g_i2c_mutex = NULL;

/**
 * @brief Cola FreeRTOS que comunica monitoring_task con logging_task.
 */
static QueueHandle_t g_log_queue = NULL;



// Handles perifericos

static i2c_master_bus_handle_t g_i2c_bus = NULL;

static i2c_master_dev_handle_t g_oled_dev = NULL;

static i2c_master_dev_handle_t g_rtc_dev = NULL;

static bool g_oled_ok = false;

static bool g_rtc_ok = false;

static uint8_t g_oled_addr = OLED_ADDR;

static spi_device_handle_t g_max_cafe = NULL;

static spi_device_handle_t g_max_tambor = NULL;

static sdmmc_card_t *g_sd_card = NULL;

static bool g_sd_ok = false;

static FILE *g_log_file = NULL;



// HX711

static int32_t g_hx711_offset = 0;

static bool g_hx711_tared = false;



// =====================================================

/** @} */

/** @name Actuadores y GPIO
 *  @brief Funciones para inicializar y controlar calefactor, motor y ventilador.
 *  @{ */
// UTILIDADES GPIO / ACTUADORES

// =====================================================



/**
 * @brief Aplica el estado lógico a los actuadores del sistema.
 *
 * Controla las salidas GPIO del calefactor, motor, ventilador y salida auxiliar.
 *
 * @param heater Estado deseado del calefactor.
 * @param motor Estado deseado del motor.
 * @param fan Estado deseado del ventilador.
 * @param relay Estado deseado de la salida auxiliar.
 */
static void actuators_apply(bool heater, bool motor, bool fan, bool relay)

{

    gpio_set_level(PIN_HEATER, heater ? 1 : 0);

    gpio_set_level(PIN_MOTOR,  motor  ? 1 : 0);

    gpio_set_level(PIN_FAN,    fan    ? 1 : 0);

    gpio_set_level(PIN_RELAY_AUX, relay ? 1 : 0);

}



/**
 * @brief Apaga todos los actuadores.
 *
 * Se usa durante la inicialización y en condiciones de parada segura.
 */
static void actuators_all_off(void)

{

    actuators_apply(false, false, false, false);

}



/**
 * @brief Inicializa las salidas GPIO asociadas a los actuadores.
 *
 * Configura como salida los pines del calefactor, motor, ventilador y salida
 * auxiliar. Al finalizar, apaga todos los actuadores por seguridad.
 */
static void gpio_outputs_init(void)

{

    gpio_config_t out = {0};

    out.pin_bit_mask = (1ULL << PIN_HEATER) | (1ULL << PIN_MOTOR) |

                       (1ULL << PIN_FAN) | (1ULL << PIN_RELAY_AUX);

    out.mode = GPIO_MODE_OUTPUT;

    out.pull_up_en = GPIO_PULLUP_DISABLE;

    out.pull_down_en = GPIO_PULLDOWN_DISABLE;

    out.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&out));

    actuators_all_off();

}



/**
 * @brief Inicializa las entradas GPIO de botones y encoder.
 *
 * Los botones START y STOP usan pull-up externo. El encoder KY-040 utiliza
 * pull-up interno en las entradas configuradas.
 */
static void gpio_inputs_init(void)

{

    gpio_config_t in_no_pull = {0};

    in_no_pull.pin_bit_mask = (1ULL << PIN_BTN_START) | (1ULL << PIN_BTN_STOP);

    in_no_pull.mode = GPIO_MODE_INPUT;

    in_no_pull.pull_up_en = GPIO_PULLUP_DISABLE;    // GPIO34/35 requieren pull-up externo

    in_no_pull.pull_down_en = GPIO_PULLDOWN_DISABLE;

    in_no_pull.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&in_no_pull));



    gpio_config_t enc = {0};

    enc.pin_bit_mask = (1ULL << PIN_ENC_CLK) | (1ULL << PIN_ENC_DT) | (1ULL << PIN_ENC_SW);

    enc.mode = GPIO_MODE_INPUT;

    enc.pull_up_en = GPIO_PULLUP_ENABLE;

    enc.pull_down_en = GPIO_PULLDOWN_DISABLE;

    enc.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&enc));

}





// =====================================================

/** @} */

/** @name Bus I2C compartido
 *  @brief Operaciones protegidas por mutex para OLED, RTC y SHT31 futuro.
 *  @{ */
// I2C SEGURO COMPARTIDO: OLED + DS3231 + SHT31 FUTURO

// =====================================================



/**
 * @brief Envía datos por I2C usando mutex de protección.
 *
 * @param dev Dispositivo I2C destino.
 * @param data Buffer de datos a transmitir.
 * @param len Número de bytes a transmitir.
 * @param timeout_ms Tiempo máximo de espera en milisegundos.
 * @return ESP_OK si la transmisión fue correcta.
 */
static esp_err_t i2c_tx_locked(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len, uint32_t timeout_ms)

{

    if (!dev || !data || len == 0) return ESP_ERR_INVALID_ARG;

    esp_err_t ret;

    if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(timeout_ms));

    ret = i2c_master_transmit(dev, data, len, pdMS_TO_TICKS(timeout_ms));

    if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);

    return ret;

}



/**
 * @brief Realiza una transacción I2C de escritura y lectura protegida por mutex.
 *
 * @param dev Dispositivo I2C.
 * @param tx_data Datos a enviar.
 * @param tx_len Cantidad de bytes a enviar.
 * @param rx_data Buffer para datos recibidos.
 * @param rx_len Cantidad de bytes a recibir.
 * @param timeout_ms Tiempo máximo de espera.
 * @return ESP_OK si la transacción fue correcta.
 */
static esp_err_t i2c_txrx_locked(i2c_master_dev_handle_t dev,

                                 const uint8_t *tx_data, size_t tx_len,

                                 uint8_t *rx_data, size_t rx_len,

                                 uint32_t timeout_ms)

{

    if (!dev || !tx_data || tx_len == 0 || !rx_data || rx_len == 0) return ESP_ERR_INVALID_ARG;

    esp_err_t ret;

    if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(timeout_ms));

    ret = i2c_master_transmit_receive(dev, tx_data, tx_len, rx_data, rx_len, pdMS_TO_TICKS(timeout_ms));

    if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);

    return ret;

}



// =====================================================

/** @} */

/** @name Pantalla OLED SSD1306
 *  @brief Inicialización y escritura básica de texto en la pantalla OLED.
 *  @{ */
// OLED SSD1306 BASICO 128x64

// =====================================================



/**
 * @brief Envía un comando de control a la pantalla OLED SSD1306.
 *
 * @param cmd Comando SSD1306 a enviar.
 * @return ESP_OK si el comando fue transmitido.
 */
static esp_err_t oled_write_cmd(uint8_t cmd)

{

    if (!g_oled_dev) return ESP_ERR_INVALID_STATE;

    uint8_t data[2] = {0x00, cmd};

    return i2c_tx_locked(g_oled_dev, data, sizeof(data), 100);

}



/**
 * @brief Envía datos gráficos/texto a la pantalla OLED.
 *
 * Divide el envío en bloques pequeños para mejorar estabilidad con cables largos.
 *
 * @param data Buffer de datos.
 * @param len Número de bytes.
 * @return ESP_OK si los datos fueron transmitidos.
 */
static esp_err_t oled_write_data(const uint8_t *data, size_t len)

{

    if (!g_oled_dev) return ESP_ERR_INVALID_STATE;

    uint8_t buffer[17];

    size_t offset = 0;

    while (offset < len) {

        size_t chunk = len - offset;

        if (chunk > 16) chunk = 16;

        buffer[0] = 0x40;

        memcpy(&buffer[1], &data[offset], chunk);

        esp_err_t err = i2c_tx_locked(g_oled_dev, buffer, chunk + 1, 100);

        if (err != ESP_OK) return err;

        offset += chunk;

    }

    return ESP_OK;

}



/**
 * @brief Obtiene el patrón básico 5x7 de un carácter.
 *
 * @param c Carácter a dibujar.
 * @param out Buffer de 5 bytes donde se escribe el patrón.
 */
static void font5x7(char c, uint8_t out[5])

{

    memset(out, 0x00, 5);

    switch (c) {

        case '0': {uint8_t f[5]={0x3E,0x51,0x49,0x45,0x3E}; memcpy(out,f,5); break;}

        case '1': {uint8_t f[5]={0x00,0x42,0x7F,0x40,0x00}; memcpy(out,f,5); break;}

        case '2': {uint8_t f[5]={0x42,0x61,0x51,0x49,0x46}; memcpy(out,f,5); break;}

        case '3': {uint8_t f[5]={0x21,0x41,0x45,0x4B,0x31}; memcpy(out,f,5); break;}

        case '4': {uint8_t f[5]={0x18,0x14,0x12,0x7F,0x10}; memcpy(out,f,5); break;}

        case '5': {uint8_t f[5]={0x27,0x45,0x45,0x45,0x39}; memcpy(out,f,5); break;}

        case '6': {uint8_t f[5]={0x3C,0x4A,0x49,0x49,0x30}; memcpy(out,f,5); break;}

        case '7': {uint8_t f[5]={0x01,0x71,0x09,0x05,0x03}; memcpy(out,f,5); break;}

        case '8': {uint8_t f[5]={0x36,0x49,0x49,0x49,0x36}; memcpy(out,f,5); break;}

        case '9': {uint8_t f[5]={0x06,0x49,0x49,0x29,0x1E}; memcpy(out,f,5); break;}

        case 'A': case 'a': {uint8_t f[5]={0x7E,0x11,0x11,0x11,0x7E}; memcpy(out,f,5); break;}

        case 'B': case 'b': {uint8_t f[5]={0x7F,0x49,0x49,0x49,0x36}; memcpy(out,f,5); break;}

        case 'C': case 'c': {uint8_t f[5]={0x3E,0x41,0x41,0x41,0x22}; memcpy(out,f,5); break;}

        case 'D': case 'd': {uint8_t f[5]={0x7F,0x41,0x41,0x22,0x1C}; memcpy(out,f,5); break;}

        case 'E': case 'e': {uint8_t f[5]={0x7F,0x49,0x49,0x49,0x41}; memcpy(out,f,5); break;}

        case 'F': case 'f': {uint8_t f[5]={0x7F,0x09,0x09,0x09,0x01}; memcpy(out,f,5); break;}

        case 'G': case 'g': {uint8_t f[5]={0x3E,0x41,0x49,0x49,0x7A}; memcpy(out,f,5); break;}

        case 'H': case 'h': {uint8_t f[5]={0x7F,0x08,0x08,0x08,0x7F}; memcpy(out,f,5); break;}

        case 'I': case 'i': {uint8_t f[5]={0x00,0x41,0x7F,0x41,0x00}; memcpy(out,f,5); break;}

        case 'J': case 'j': {uint8_t f[5]={0x20,0x40,0x41,0x3F,0x01}; memcpy(out,f,5); break;}

        case 'K': case 'k': {uint8_t f[5]={0x7F,0x08,0x14,0x22,0x41}; memcpy(out,f,5); break;}

        case 'L': case 'l': {uint8_t f[5]={0x7F,0x40,0x40,0x40,0x40}; memcpy(out,f,5); break;}

        case 'M': case 'm': {uint8_t f[5]={0x7F,0x02,0x0C,0x02,0x7F}; memcpy(out,f,5); break;}

        case 'N': case 'n': {uint8_t f[5]={0x7F,0x04,0x08,0x10,0x7F}; memcpy(out,f,5); break;}

        case 'O': case 'o': {uint8_t f[5]={0x3E,0x41,0x41,0x41,0x3E}; memcpy(out,f,5); break;}

        case 'P': case 'p': {uint8_t f[5]={0x7F,0x09,0x09,0x09,0x06}; memcpy(out,f,5); break;}

        case 'Q': case 'q': {uint8_t f[5]={0x3E,0x41,0x51,0x21,0x5E}; memcpy(out,f,5); break;}

        case 'R': case 'r': {uint8_t f[5]={0x7F,0x09,0x19,0x29,0x46}; memcpy(out,f,5); break;}

        case 'S': case 's': {uint8_t f[5]={0x46,0x49,0x49,0x49,0x31}; memcpy(out,f,5); break;}

        case 'T': case 't': {uint8_t f[5]={0x01,0x01,0x7F,0x01,0x01}; memcpy(out,f,5); break;}

        case 'U': case 'u': {uint8_t f[5]={0x3F,0x40,0x40,0x40,0x3F}; memcpy(out,f,5); break;}

        case 'V': case 'v': {uint8_t f[5]={0x1F,0x20,0x40,0x20,0x1F}; memcpy(out,f,5); break;}

        case 'W': case 'w': {uint8_t f[5]={0x7F,0x20,0x18,0x20,0x7F}; memcpy(out,f,5); break;}

        case 'X': case 'x': {uint8_t f[5]={0x63,0x14,0x08,0x14,0x63}; memcpy(out,f,5); break;}

        case 'Y': case 'y': {uint8_t f[5]={0x07,0x08,0x70,0x08,0x07}; memcpy(out,f,5); break;}

        case 'Z': case 'z': {uint8_t f[5]={0x61,0x51,0x49,0x45,0x43}; memcpy(out,f,5); break;}

        case ':': {uint8_t f[5]={0x00,0x36,0x36,0x00,0x00}; memcpy(out,f,5); break;}

        case '.': {uint8_t f[5]={0x00,0x60,0x60,0x00,0x00}; memcpy(out,f,5); break;}

        case '-': {uint8_t f[5]={0x08,0x08,0x08,0x08,0x08}; memcpy(out,f,5); break;}

        case '/': {uint8_t f[5]={0x20,0x10,0x08,0x04,0x02}; memcpy(out,f,5); break;}

        case '%': {uint8_t f[5]={0x23,0x13,0x08,0x64,0x62}; memcpy(out,f,5); break;}

        case '=': {uint8_t f[5]={0x14,0x14,0x14,0x14,0x14}; memcpy(out,f,5); break;}

        case ' ': default: break;

    }

}



/**
 * @brief Ubica el cursor de escritura de la OLED.
 *
 * @param page Página vertical de la OLED.
 * @param col Columna horizontal.
 */
static void oled_set_cursor(uint8_t page, uint8_t col)

{

    oled_write_cmd(0xB0 | (page & 0x07));

    oled_write_cmd(0x00 | (col & 0x0F));

    oled_write_cmd(0x10 | ((col >> 4) & 0x0F));

}



/**
 * @brief Limpia completamente la pantalla OLED.
 */
static void oled_clear(void)

{

    uint8_t zeros[128];

    memset(zeros, 0, sizeof(zeros));

    for (uint8_t page = 0; page < 8; page++) {

        oled_set_cursor(page, 0);

        oled_write_data(zeros, sizeof(zeros));

    }

}



/**
 * @brief Imprime una línea de texto en la OLED.
 *
 * @param page Página vertical donde se escribe.
 * @param text Cadena de texto a mostrar.
 */
static void oled_print_line(uint8_t page, const char *text)

{

    uint8_t buf[128];

    memset(buf, 0x00, sizeof(buf));

    uint8_t col = 0;

    for (int i = 0; text[i] != 0 && col < 126; i++) {

        uint8_t f[5];

        font5x7(text[i], f);

        for (int j = 0; j < 5 && col < 128; j++) buf[col++] = f[j];

        if (col < 128) buf[col++] = 0x00;

    }

    oled_set_cursor(page, 0);

    oled_write_data(buf, sizeof(buf));

}



/**
 * @brief Inicializa el bus I2C y detecta la OLED SSD1306.
 *
 * Prueba direcciones 0x3C y 0x3D, inicializa la pantalla y muestra un
 * mensaje de arranque.
 *
 * @return ESP_OK si la OLED fue detectada e inicializada.
 */
static esp_err_t i2c_oled_init(void)

{

    ESP_LOGI(TAG, "Inicializando I2C OLED SDA=GPIO21 SCL=GPIO22");



    i2c_master_bus_config_t bus_cfg = {

        .i2c_port = I2C_NUM_0,

        .sda_io_num = PIN_I2C_SDA,

        .scl_io_num = PIN_I2C_SCL,

        .clk_source = I2C_CLK_SRC_DEFAULT,

        .glitch_ignore_cnt = 7,

        .flags.enable_internal_pullup = true,

    };



    esp_err_t err = i2c_new_master_bus(&bus_cfg, &g_i2c_bus);

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "No se pudo crear bus I2C: %s", esp_err_to_name(err));

        return err;

    }



    // Muchos OLED SSD1306 vienen en 0x3C, otros en 0x3D.

    // Este codigo prueba ambos para evitar pantalla negra por direccion equivocada.

    const uint8_t candidates[] = {0x3C, 0x3D};

    esp_err_t last_err = ESP_FAIL;



    for (size_t i = 0; i < sizeof(candidates); i++) {

        uint8_t addr = candidates[i];

        i2c_device_config_t dev_cfg = {

            .dev_addr_length = I2C_ADDR_BIT_LEN_7,

            .device_address = addr,

            .scl_speed_hz = 100000,   // 100 kHz es mas seguro para cables largos/protoboard

        };



        i2c_master_dev_handle_t test_dev = NULL;

        err = i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, &test_dev);

        if (err != ESP_OK) {

            last_err = err;

            continue;

        }



        uint8_t probe = 0x00; // control byte de comando

        err = i2c_tx_locked(test_dev, &probe, 1, 100);

        if (err == ESP_OK) {

            g_oled_dev = test_dev;

            g_oled_addr = addr;

            ESP_LOGI(TAG, "OLED detectada en direccion I2C 0x%02X", g_oled_addr);

            break;

        }



        ESP_LOGW(TAG, "No responde OLED en 0x%02X: %s", addr, esp_err_to_name(err));

        i2c_master_bus_rm_device(test_dev);

        last_err = err;

    }



    if (!g_oled_dev) {

        ESP_LOGE(TAG, "OLED no encontrada en 0x3C ni 0x3D. Revisar VCC/GND/SDA/SCL.");

        return last_err;

    }



    vTaskDelay(pdMS_TO_TICKS(150));



    // Secuencia robusta SSD1306 128x64 I2C.

    const uint8_t init_cmds[] = {

        0xAE,       // display OFF

        0xD5, 0x80, // clock

        0xA8, 0x3F, // multiplex 1/64

        0xD3, 0x00, // display offset

        0x40,       // start line 0

        0x8D, 0x14, // charge pump ON

        0x20, 0x00, // memory mode horizontal

        0xA1,       // segment remap

        0xC8,       // COM scan dec

        0xDA, 0x12, // COM pins

        0x81, 0xCF, // contrast alto

        0xD9, 0xF1, // precharge

        0xDB, 0x40, // vcom detect

        0xA4,       // resume RAM display

        0xA6,       // normal display

        0x2E,       // deactivate scroll

        0xAF        // display ON

    };



    for (size_t i = 0; i < sizeof(init_cmds); i++) {

        err = oled_write_cmd(init_cmds[i]);

        if (err != ESP_OK) {

            ESP_LOGE(TAG, "Fallo comando OLED 0x%02X: %s", init_cmds[i], esp_err_to_name(err));

            return err;

        }

    }



    oled_clear();

    oled_print_line(0, "TOSTADOR ESP32");

    oled_print_line(1, "OLED OK");

    oled_print_line(2, "I2C 21/22");

    char addr_line[20];

    snprintf(addr_line, sizeof(addr_line), "ADDR:0x%02X", g_oled_addr);

    oled_print_line(3, addr_line);



    g_oled_ok = true;

    ESP_LOGI(TAG, "OLED inicializada correctamente");

    vTaskDelay(pdMS_TO_TICKS(1000));

    return ESP_OK;

}





// =====================================================

/** @} */

/** @name RTC DS3231
 *  @brief Funciones para detectar y leer fecha/hora desde el reloj de tiempo real.
 *  @{ */
// RTC DS3231 EN I2C

// =====================================================



/**
 * @brief Convierte un valor BCD a decimal.
 *
 * @param value Valor en formato BCD.
 * @return Valor decimal equivalente.
 */
static uint8_t bcd_to_dec(uint8_t value)

{

    return (uint8_t)(((value >> 4) * 10) + (value & 0x0F));

}



typedef struct {

    int year;

    int month;

    int day;

    int hour;

    int minute;

    int second;

    bool valid;

} rtc_datetime_t;



/**
 * @brief Inicializa y detecta el RTC DS3231 en el bus I2C.
 *
 * @return ESP_OK si el RTC responde correctamente.
 */
static esp_err_t rtc_ds3231_init(void)

{

    if (!g_i2c_bus) {

        ESP_LOGW(TAG, "RTC DS3231 no iniciado: bus I2C no disponible");

        g_rtc_ok = false;

        return ESP_ERR_INVALID_STATE;

    }



    i2c_device_config_t dev_cfg = {

        .dev_addr_length = I2C_ADDR_BIT_LEN_7,

        .device_address = DS3231_ADDR,

        .scl_speed_hz = 100000,

    };



    esp_err_t err = i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, &g_rtc_dev);

    if (err != ESP_OK) {

        ESP_LOGW(TAG, "No se pudo agregar DS3231 al bus I2C: %s", esp_err_to_name(err));

        g_rtc_ok = false;

        return err;

    }



    uint8_t reg = 0x00;

    uint8_t sec = 0;

    err = i2c_txrx_locked(g_rtc_dev, &reg, 1, &sec, 1, 100);

    if (err != ESP_OK) {

        ESP_LOGW(TAG, "DS3231 no responde en direccion 0x68: %s", esp_err_to_name(err));

        i2c_master_bus_rm_device(g_rtc_dev);

        g_rtc_dev = NULL;

        g_rtc_ok = false;

        return err;

    }



    g_rtc_ok = true;

    ESP_LOGI(TAG, "RTC DS3231 detectado en I2C 0x68");

    return ESP_OK;

}



/**
 * @brief Lee fecha y hora desde el RTC DS3231.
 *
 * @param dt Estructura donde se almacena la fecha y hora.
 * @return ESP_OK si la lectura fue válida.
 */
static esp_err_t rtc_ds3231_read(rtc_datetime_t *dt)

{

    if (!dt) return ESP_ERR_INVALID_ARG;

    memset(dt, 0, sizeof(*dt));

    dt->valid = false;



    if (!g_rtc_ok || !g_rtc_dev) {

        return ESP_ERR_INVALID_STATE;

    }



    uint8_t reg = 0x00;

    uint8_t data[7] = {0};

    esp_err_t err = i2c_txrx_locked(g_rtc_dev, &reg, 1, data, sizeof(data), 100);

    if (err != ESP_OK) {

        return err;

    }



    dt->second = bcd_to_dec(data[0] & 0x7F);

    dt->minute = bcd_to_dec(data[1] & 0x7F);



    // DS3231 puede estar en modo 12h o 24h. Se normaliza a 24h.

    if (data[2] & 0x40) {

        int hour12 = bcd_to_dec(data[2] & 0x1F);

        bool pm = data[2] & 0x20;

        if (hour12 == 12) hour12 = 0;

        dt->hour = hour12 + (pm ? 12 : 0);

    } else {

        dt->hour = bcd_to_dec(data[2] & 0x3F);

    }



    dt->day = bcd_to_dec(data[4] & 0x3F);

    dt->month = bcd_to_dec(data[5] & 0x1F);

    dt->year = 2000 + bcd_to_dec(data[6]);



    if (dt->year >= 2020 && dt->month >= 1 && dt->month <= 12 &&

        dt->day >= 1 && dt->day <= 31 && dt->hour <= 23 &&

        dt->minute <= 59 && dt->second <= 59) {

        dt->valid = true;

        return ESP_OK;

    }



    return ESP_ERR_INVALID_RESPONSE;

}



/**
 * @brief Obtiene una cadena de fecha/hora desde el RTC.
 *
 * Si el RTC no está disponible, escribe RTC_OFF.
 *
 * @param buffer Buffer de salida.
 * @param len Tamaño del buffer.
 */
static void rtc_datetime_string(char *buffer, size_t len)

{

    if (!buffer || len == 0) return;



    rtc_datetime_t dt;

    esp_err_t err = rtc_ds3231_read(&dt);

    if (err == ESP_OK && dt.valid) {

        snprintf(buffer, len, "%04d-%02d-%02d %02d:%02d:%02d",

                 dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);

    } else {

        snprintf(buffer, len, "RTC_OFF");

    }

}



// =====================================================

/** @} */

/** @name Bus SPI, termocuplas y microSD
 *  @brief Inicialización del bus SPI, lectura MAX6675 y montaje de microSD.
 *  @{ */
// SPI / MAX6675 / SD

// =====================================================



/**
 * @brief Inicializa el bus SPI compartido por MAX6675 y microSD.
 *
 * Agrega dos dispositivos MAX6675 al bus, cada uno con su propio CS.
 *
 * @return ESP_OK si el bus y los MAX6675 fueron configurados.
 */
static esp_err_t spi_shared_init(void)

{

    spi_bus_config_t buscfg = {

        .mosi_io_num = PIN_SPI_MOSI,

        .miso_io_num = PIN_SPI_MISO,

        .sclk_io_num = PIN_SPI_SCK,

        .quadwp_io_num = -1,

        .quadhd_io_num = -1,

        .max_transfer_sz = 4096,

    };

    esp_err_t err = spi_bus_initialize(SPI_HOST_USED, &buscfg, SDSPI_DEFAULT_DMA);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(TAG, "Fallo inicializando SPI: %s", esp_err_to_name(err));

        return err;

    }



    spi_device_interface_config_t max_cfg = {

        .clock_speed_hz = 1000000,

        .mode = 0,

        .spics_io_num = PIN_MAX_CAFE_CS,

        .queue_size = 1,

    };

    err = spi_bus_add_device(SPI_HOST_USED, &max_cfg, &g_max_cafe);

    if (err != ESP_OK) return err;



    max_cfg.spics_io_num = PIN_MAX_TAMB_CS;

    err = spi_bus_add_device(SPI_HOST_USED, &max_cfg, &g_max_tambor);

    if (err != ESP_OK) return err;



    return ESP_OK;

}



/**
 * @brief Lee temperatura desde un módulo MAX6675.
 *
 * @param dev Manejador SPI del MAX6675.
 * @param temp_c Puntero donde se almacena la temperatura en Celsius.
 * @param ok Bandera que indica si la lectura fue válida.
 * @return ESP_OK si la lectura fue correcta.
 */
static esp_err_t max6675_read(spi_device_handle_t dev, float *temp_c, bool *ok)

{

    if (!dev || !temp_c || !ok) return ESP_ERR_INVALID_ARG;

    uint8_t rx[2] = {0};

    spi_transaction_t t = {0};

    t.length = 16;

    t.rxlength = 16;

    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(dev, &t);

    if (err != ESP_OK) {

        *ok = false;

        return err;

    }

    uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];

    if (raw & 0x0004) {

        *ok = false;  // termocupla abierta

        return ESP_ERR_INVALID_RESPONSE;

    }

    float c = ((raw >> 3) & 0x0FFF) * 0.25f;

    if (c < 0.0f || c > 500.0f) {

        *ok = false;

        return ESP_ERR_INVALID_RESPONSE;

    }

    *temp_c = c;

    *ok = true;

    return ESP_OK;

}



/**
 * @brief Monta la microSD y abre el archivo CSV de registro.
 *
 * Si el archivo está vacío, escribe el encabezado del CSV.
 *
 * @return ESP_OK si la microSD fue montada.
 */
static esp_err_t sdcard_init(void)

{

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {

        .format_if_mount_failed = false,

        .max_files = 3,

        .allocation_unit_size = 16 * 1024,

    };



    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    host.slot = SPI_HOST_USED;



    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

    slot_config.gpio_cs = PIN_SD_CS;

    slot_config.host_id = SPI_HOST_USED;



    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &g_sd_card);

    if (ret != ESP_OK) {

        ESP_LOGW(TAG, "microSD no montada: %s. El sistema continua sin CSV.", esp_err_to_name(ret));

        g_sd_ok = false;

        return ret;

    }



    g_sd_ok = true;

    ESP_LOGI(TAG, "microSD montada correctamente");

    g_log_file = fopen("/sdcard/tostador.csv", "a");

    if (g_log_file) {

        struct stat st;

        if (stat("/sdcard/tostador.csv", &st) == 0 && st.st_size == 0) {

            fprintf(g_log_file, "datetime,ms,state,cafe_c,tambor_c,peso_g,hx_ok,heater,motor,fan,target_c,rtc_ok\n");

            fflush(g_log_file);

        }

    }

    return ESP_OK;

}



// =====================================================

/** @} */

/** @name HX711 canal B
 *  @brief Funciones para tara y lectura de peso mediante celda de carga.
 *  @{ */
// HX711 CANAL B GANANCIA 32

// =====================================================



/**
 * @brief Inicializa los pines del HX711.
 *
 * Configura SCK como salida y DT como entrada para lectura de celda de carga.
 *
 * @return ESP_OK si la configuración fue correcta.
 */
static esp_err_t hx711_init(void)

{

    gpio_config_t sck = {0};

    sck.pin_bit_mask = (1ULL << PIN_HX711_SCK);

    sck.mode = GPIO_MODE_OUTPUT;

    sck.pull_up_en = GPIO_PULLUP_DISABLE;

    sck.pull_down_en = GPIO_PULLDOWN_DISABLE;

    sck.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&sck));

    gpio_set_level(PIN_HX711_SCK, 0);



    gpio_config_t dt = {0};

    dt.pin_bit_mask = (1ULL << PIN_HX711_DT);

    dt.mode = GPIO_MODE_INPUT;

    dt.pull_up_en = GPIO_PULLUP_DISABLE;

    dt.pull_down_en = GPIO_PULLDOWN_DISABLE;

    dt.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&dt));



    ESP_LOGI(TAG, "HX711 inicializado en canal B: DT=GPIO32 SCK=GPIO33");

    return ESP_OK;

}



/**
 * @brief Lee una muestra cruda del HX711 usando canal B.
 *
 * Después de los 24 bits genera dos pulsos extra para seleccionar canal B
 * con ganancia 32 en la siguiente conversión.
 *
 * @param raw Puntero donde se almacena la lectura cruda.
 * @return ESP_OK si la lectura fue válida.
 */
static esp_err_t hx711_read_raw_b(int32_t *raw)

{

    if (!raw) return ESP_ERR_INVALID_ARG;



    gpio_set_level(PIN_HX711_SCK, 0);

    esp_rom_delay_us(10);



    int timeout_ms = 1200;

    while (gpio_get_level(PIN_HX711_DT) == 1 && timeout_ms-- > 0) {

        vTaskDelay(pdMS_TO_TICKS(1));

    }

    if (timeout_ms <= 0) return ESP_ERR_TIMEOUT;



    int32_t value = 0;

    for (int i = 0; i < 24; i++) {

        gpio_set_level(PIN_HX711_SCK, 1);

        esp_rom_delay_us(5);

        gpio_set_level(PIN_HX711_SCK, 0);

        esp_rom_delay_us(5);

        value <<= 1;

        if (gpio_get_level(PIN_HX711_DT)) value++;

    }



    // 2 pulsos extra: canal B, ganancia 32 para la siguiente conversion.

    for (int i = 0; i < 2; i++) {

        gpio_set_level(PIN_HX711_SCK, 1);

        esp_rom_delay_us(5);

        gpio_set_level(PIN_HX711_SCK, 0);

        esp_rom_delay_us(5);

    }



    if (value & 0x800000) value |= 0xFF000000;



    if (value >= 8388607 || value <= -8388608) return ESP_ERR_INVALID_RESPONSE;

    *raw = value;

    return ESP_OK;

}



/**
 * @brief Calcula el promedio de varias lecturas HX711.
 *
 * @param avg Puntero donde se almacena el promedio.
 * @param samples Cantidad de muestras a promediar.
 * @return ESP_OK si al menos una muestra fue válida.
 */
static esp_err_t hx711_read_average(int32_t *avg, int samples)

{

    if (!avg) return ESP_ERR_INVALID_ARG;

    int64_t sum = 0;

    int valid = 0;

    for (int i = 0; i < samples; i++) {

        int32_t raw = 0;

        if (hx711_read_raw_b(&raw) == ESP_OK) {

            sum += raw;

            valid++;

        }

        vTaskDelay(pdMS_TO_TICKS(HX711_SAMPLE_DELAY_MS));

    }

    if (valid == 0) return ESP_ERR_TIMEOUT;

    *avg = (int32_t)(sum / valid);

    return ESP_OK;

}



/**
 * @brief Realiza la tara inicial del HX711.
 *
 * Promedia varias muestras sin peso y guarda el offset global.
 *
 * @return ESP_OK si la tara fue realizada.
 */
static esp_err_t hx711_tare(void)

{

    int32_t avg = 0;

    esp_err_t ret = hx711_read_average(&avg, 10);

    if (ret == ESP_OK) {

        g_hx711_offset = avg;

        g_hx711_tared = true;

        ESP_LOGI(TAG, "Tara HX711 OK. offset=%ld", (long)g_hx711_offset);

    } else {

        ESP_LOGW(TAG, "No se pudo hacer tara HX711: %s", esp_err_to_name(ret));

    }

    return ret;

}



/**
 * @brief Convierte la lectura del HX711 a gramos.
 *
 * Usa el offset de tara y el factor de calibración definido.
 *
 * @param grams Puntero donde se guarda el peso en gramos.
 * @param ok Bandera de validez de la lectura.
 * @return ESP_OK si la lectura fue válida.
 */
static esp_err_t hx711_read_grams(float *grams, bool *ok)

{

    if (!grams || !ok) return ESP_ERR_INVALID_ARG;

    *grams = 0.0f;

    *ok = false;



    int32_t avg = 0;

    esp_err_t ret = hx711_read_average(&avg, HX711_AVERAGE_SAMPLES);

    if (ret != ESP_OK) return ret;



    if (!g_hx711_tared) {

        g_hx711_offset = avg;

        g_hx711_tared = true;

    }



    float weight = ((float)(avg - g_hx711_offset)) / HX711_CALIBRATION_FACTOR;

    if (weight > -HX711_ZERO_DEADBAND_G && weight < HX711_ZERO_DEADBAND_G) weight = 0.0f;



    if (weight < HX711_MIN_VALID_WEIGHT_G || weight > HX711_MAX_VALID_WEIGHT_G) {

        return ESP_ERR_INVALID_RESPONSE;

    }



    *grams = weight;

    *ok = true;

    return ESP_OK;

}



// =====================================================

/** @} */

/** @name Tareas FreeRTOS
 *  @brief Tareas concurrentes para monitoreo, control, interfaz y registro.
 *  @{ */
// TAREAS FREERTOS

// =====================================================



/**
 * @brief Convierte un estado del sistema a texto corto.
 *
 * @param s Estado del proceso.
 * @return Cadena de texto asociada al estado.
 */
static const char *state_name(process_state_t s)

{

    switch (s) {

        case STATE_IDLE: return "IDLE";

        case STATE_ROASTING: return "ROAST";

        case STATE_COOLING: return "COOL";

        case STATE_EMERGENCY: return "ALARM";

        case STATE_FINISHED: return "FIN";

        default: return "UNK";

    }

}



/**
 * @brief Tarea de monitoreo de sensores.
 *
 * Se ejecuta cada MONITORING_PERIOD_MS. Lee RTC, termocuplas MAX6675 y HX711,
 * actualiza la última muestra global y envía una copia a la cola de registro.
 *
 * @param arg Parámetro no utilizado.
 */
static void monitoring_task(void *arg)

{

    (void)arg;

    TickType_t last = xTaskGetTickCount();

    while (true) {

        vTaskDelayUntil(&last, pdMS_TO_TICKS(MONITORING_PERIOD_MS));



        sensor_sample_t s = {0};

        s.timestamp_ms = esp_timer_get_time() / 1000;

        rtc_datetime_string(s.datetime, sizeof(s.datetime));

        s.sht31_ok = false;

        s.ambient_c = 0.0f;

        s.humidity_percent = 0.0f;



        max6675_read(g_max_cafe, &s.coffee_temp_c, &s.coffee_ok);

        vTaskDelay(pdMS_TO_TICKS(20));

        max6675_read(g_max_tambor, &s.drum_temp_c, &s.drum_ok);

        hx711_read_grams(&s.weight_g, &s.hx711_ok);



        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

            g_last_sample = s;

            g_has_sample = true;

            xSemaphoreGive(g_data_mutex);

        }



        xQueueSend(g_log_queue, &s, 0);



        ESP_LOGI(TAG, "RTC=%s Cafe=%.2f[%s] Tambor=%.2f[%s] Peso=%.1f[%s] SHT31=%s",

                 s.datetime,

                 s.coffee_temp_c, s.coffee_ok ? "OK" : "FAIL",

                 s.drum_temp_c, s.drum_ok ? "OK" : "FAIL",

                 s.weight_g, s.hx711_ok ? "OK" : "PEND",

                 ENABLE_SHT31 ? "ON" : "OFF");

    }

}



/**
 * @brief Tarea de control automático y seguridad.
 *
 * Evalúa estado, temperatura del café, temperatura del tambor y fallas. Decide
 * el encendido del calefactor, motor y ventilador. Implementa histéresis y
 * modo de emergencia.
 *
 * @param arg Parámetro no utilizado.
 */
static void control_task(void *arg)

{

    (void)arg;

    TickType_t last = xTaskGetTickCount();

    while (true) {

        vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));



        sensor_sample_t s;

        bool has_sample = false;

        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

            s = g_last_sample;

            has_sample = g_has_sample;

            xSemaphoreGive(g_data_mutex);

        } else {

            continue;

        }



        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

            bool heater = false, motor = false, fan = false, relay = false;

            bool temp_fail = has_sample && (!s.coffee_ok || !s.drum_ok);

            bool overtemp = has_sample && ((s.coffee_ok && s.coffee_temp_c >= MAX_SAFE_TEMP_C) ||

                                           (s.drum_ok && s.drum_temp_c >= MAX_SAFE_TEMP_C));



            // Solo se activa emergencia por sensores durante estados activos. En IDLE se permite

            // iniciar el sistema sin que una lectura vieja o ausente bloquee los actuadores.

            if ((g_state.state == STATE_ROASTING || g_state.state == STATE_COOLING) &&

                (temp_fail || overtemp)) {

                g_state.state = STATE_EMERGENCY;

                ESP_LOGW(TAG, "SEGURIDAD: temp_fail=%d overtemp=%d -> EMERGENCY", temp_fail, overtemp);

            }



            switch (g_state.state) {

                case STATE_IDLE:

                    heater = false; motor = false; fan = false; relay = false;

                    break;



                case STATE_ROASTING:

                    motor = true;

                    relay = true;

                    if (s.coffee_ok) {

                        if (s.coffee_temp_c < (g_state.target_temp_c - HEATER_HYSTERESIS_C)) heater = true;

                        else if (s.coffee_temp_c >= g_state.target_temp_c) heater = false;

                        else heater = g_state.heater_on;

                    }

                    fan = (s.drum_ok && s.drum_temp_c >= FAN_ON_DRUM_TEMP_C);

                    break;



                case STATE_COOLING:

                    heater = false;

                    motor = false;

                    fan = true;

                    relay = false;

                    if (s.coffee_ok && s.coffee_temp_c <= COOLING_SAFE_TEMP_C) {

                        g_state.state = STATE_FINISHED;

                    }

                    break;



                case STATE_EMERGENCY:

                    heater = false;

                    motor = false;

                    fan = true;

                    relay = false;

                    break;



                case STATE_FINISHED:

                default:

                    heater = false; motor = false; fan = false; relay = false;

                    break;

            }



            g_state.heater_on = heater;

            g_state.motor_on = motor;

            g_state.fan_on = fan;

            g_state.relay_on = relay;

            actuators_apply(heater, motor, fan, relay);

            static int log_div = 0;

            if (++log_div >= 4) {

                log_div = 0;

                ESP_LOGI(TAG, "CONTROL estado=%s sample=%d cafe=%.1f tambor=%.1f -> RES=%s MOTOR=%s FAN=%s",

                         state_name(g_state.state), has_sample ? 1 : 0,

                         s.coffee_temp_c, s.drum_temp_c,

                         heater ? "ON" : "OFF", motor ? "ON" : "OFF", fan ? "ON" : "OFF");

            }

            xSemaphoreGive(g_data_mutex);

        }

    }

}



/**
 * @brief Tarea de lectura de botones START y STOP.
 *
 * Los botones son activos en bajo por pull-up externo. START inicia tostión
 * o reinicia desde emergencia. STOP pasa a enfriamiento o apaga el sistema.
 *
 * @param arg Parámetro no utilizado.
 */
static void buttons_task(void *arg)

{

    (void)arg;

    int last_start = 1, last_stop = 1;

    while (true) {

        int start = gpio_get_level(PIN_BTN_START);

        int stop = gpio_get_level(PIN_BTN_STOP);



        // Activo en bajo por pull-up externo

        if (last_start == 1 && start == 0) {

            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

                if (g_state.state == STATE_IDLE || g_state.state == STATE_FINISHED) {

                    g_state.state = STATE_ROASTING;

                    g_state.roast_start_ms = esp_timer_get_time() / 1000;

                    ESP_LOGW(TAG, "START: inicia tostado automatico");

                } else if (g_state.state == STATE_EMERGENCY) {

                    g_state.state = STATE_IDLE;

                    ESP_LOGW(TAG, "START: reinicio desde emergencia a IDLE");

                }

                xSemaphoreGive(g_data_mutex);

            }

        }



        if (last_stop == 1 && stop == 0) {

            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

                if (g_state.state == STATE_ROASTING) {

                    g_state.state = STATE_COOLING;

                    g_state.cooling_start_ms = esp_timer_get_time() / 1000;

                    ESP_LOGW(TAG, "STOP: pasa a enfriamiento");

                } else {

                    g_state.state = STATE_IDLE;

                    actuators_all_off();

                    ESP_LOGW(TAG, "STOP: todo apagado / IDLE");

                }

                xSemaphoreGive(g_data_mutex);

            }

        }



        last_start = start;

        last_stop = stop;

        vTaskDelay(pdMS_TO_TICKS(BUTTON_PERIOD_MS));

    }

}



/**
 * @brief Tarea de lectura del encoder KY-040.
 *
 * Permite ajustar la temperatura objetivo y usar el pulsador del encoder
 * como control de inicio/enfriamiento.
 *
 * @param arg Parámetro no utilizado.
 */
static void encoder_task(void *arg)

{

    (void)arg;

    int last_clk = gpio_get_level(PIN_ENC_CLK);

    int last_sw = gpio_get_level(PIN_ENC_SW);

    while (true) {

        int clk = gpio_get_level(PIN_ENC_CLK);

        int dt = gpio_get_level(PIN_ENC_DT);

        int sw = gpio_get_level(PIN_ENC_SW);



        if (clk != last_clk && clk == 0) {

            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {

                if (dt != clk) g_state.target_temp_c += 1.0f;

                else g_state.target_temp_c -= 1.0f;

                if (g_state.target_temp_c < 120.0f) g_state.target_temp_c = 120.0f;

                if (g_state.target_temp_c > 230.0f) g_state.target_temp_c = 230.0f;

                ESP_LOGI(TAG, "Encoder target=%.1f", g_state.target_temp_c);

                xSemaphoreGive(g_data_mutex);

            }

        }



        if (last_sw == 1 && sw == 0) {

            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {

                if (g_state.state == STATE_IDLE || g_state.state == STATE_FINISHED) {

                    g_state.state = STATE_ROASTING;

                    g_state.roast_start_ms = esp_timer_get_time() / 1000;

                    ESP_LOGW(TAG, "Encoder SW: START");

                } else if (g_state.state == STATE_ROASTING) {

                    g_state.state = STATE_COOLING;

                    g_state.cooling_start_ms = esp_timer_get_time() / 1000;

                    ESP_LOGW(TAG, "Encoder SW: COOLING");

                }

                xSemaphoreGive(g_data_mutex);

            }

        }



        last_clk = clk;

        last_sw = sw;

        vTaskDelay(pdMS_TO_TICKS(ENCODER_PERIOD_MS));

    }

}



/**
 * @brief Tarea de actualización de pantalla OLED.
 *
 * Muestra estado, temperaturas, peso, objetivo, actuadores, microSD, RTC y SHT31.
 *
 * @param arg Parámetro no utilizado.
 */
static void oled_task(void *arg)

{

    (void)arg;

    char line[32];

    if (!g_oled_ok) {

        ESP_LOGW(TAG, "Tarea OLED detenida: pantalla no inicializada");

        vTaskDelete(NULL);

        return;

    }

    while (true) {

        sensor_sample_t s;

        system_state_t st;

        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

            s = g_last_sample;

            st = g_state;

            xSemaphoreGive(g_data_mutex);

        } else {

            vTaskDelay(pdMS_TO_TICKS(OLED_PERIOD_MS));

            continue;

        }



        snprintf(line, sizeof(line), "Estado:%s", state_name(st.state));

        oled_print_line(0, line);

        snprintf(line, sizeof(line), "Cafe:%5.1fC", s.coffee_temp_c);

        oled_print_line(1, line);

        snprintf(line, sizeof(line), "Tamb:%5.1fC", s.drum_temp_c);

        oled_print_line(2, line);

        snprintf(line, sizeof(line), "Peso:%5.1fg", s.weight_g);

        oled_print_line(3, line);

        snprintf(line, sizeof(line), "Obj:%5.1fC", st.target_temp_c);

        oled_print_line(4, line);

        snprintf(line, sizeof(line), "RES:%s MOT:%s", st.heater_on ? "ON" : "OFF", st.motor_on ? "ON" : "OFF");

        oled_print_line(5, line);

        snprintf(line, sizeof(line), "FAN:%s SD:%s", st.fan_on ? "ON" : "OFF", g_sd_ok ? "OK" : "NO");

        oled_print_line(6, line);

        snprintf(line, sizeof(line), "RTC:%s SHT:%s", g_rtc_ok ? "OK" : "NO", ENABLE_SHT31 ? "ON" : "OFF");

        oled_print_line(7, line);



        vTaskDelay(pdMS_TO_TICKS(OLED_PERIOD_MS));

    }

}



/**
 * @brief Tarea de registro de datos en microSD.
 *
 * Recibe muestras desde g_log_queue y las escribe en formato CSV cuando la
 * microSD está disponible.
 *
 * @param arg Parámetro no utilizado.
 */
static void logging_task(void *arg)

{

    (void)arg;

    sensor_sample_t s;

    while (true) {

        if (xQueueReceive(g_log_queue, &s, pdMS_TO_TICKS(LOG_PERIOD_MS)) == pdTRUE) {

            system_state_t st;

            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

                st = g_state;

                xSemaphoreGive(g_data_mutex);

            } else {

                continue;

            }



            if (g_sd_ok && g_log_file) {

                fprintf(g_log_file, "%s,%lld,%s,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.1f,%d\n",

                        s.datetime,

                        (long long)s.timestamp_ms,

                        state_name(st.state),

                        s.coffee_temp_c,

                        s.drum_temp_c,

                        s.weight_g,

                        s.hx711_ok ? 1 : 0,

                        st.heater_on ? 1 : 0,

                        st.motor_on ? 1 : 0,

                        st.fan_on ? 1 : 0,

                        st.target_temp_c,

                        g_rtc_ok ? 1 : 0);

                fflush(g_log_file);

            }

        }

    }

}



// =====================================================

/** @} */

/** @name Punto de entrada
 *  @brief Inicialización general y creación de tareas FreeRTOS.
 *  @{ */
// APP MAIN

// =====================================================



/**
 * @brief Punto de entrada principal del firmware.
 *
 * Crea mutex y cola, inicializa GPIO, OLED, RTC, SPI, microSD y HX711. Luego
 * crea las tareas FreeRTOS y elimina la tarea principal.
 */
void app_main(void)

{

    ESP_LOGW(TAG, "Iniciando tostadora automatica ESP32 + FreeRTOS");

    ESP_LOGW(TAG, "Actuadores: GPIO25 calefactor, GPIO26 motor, GPIO27 ventilador");



    g_data_mutex = xSemaphoreCreateMutex();

    g_i2c_mutex = xSemaphoreCreateMutex();

    g_log_queue = xQueueCreate(10, sizeof(sensor_sample_t));

    if (!g_data_mutex || !g_i2c_mutex || !g_log_queue) {

        ESP_LOGE(TAG, "No se pudieron crear recursos FreeRTOS");

        return;

    }



    gpio_outputs_init();

    gpio_inputs_init();



    esp_err_t err = i2c_oled_init();

    if (err != ESP_OK) {

        ESP_LOGW(TAG, "OLED no disponible: %s. Continua por consola.", esp_err_to_name(err));

    }



    // RTC DS3231 comparte el mismo bus I2C con la OLED y el SHT31 futuro.

    rtc_ds3231_init();



    err = spi_shared_init();

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "SPI no inicializado. Termocuplas/SD no funcionaran.");

    } else {

        sdcard_init();

    }



    hx711_init();

    ESP_LOGI(TAG, "Esperando 3 segundos para estabilizar HX711. No poner peso.");

    vTaskDelay(pdMS_TO_TICKS(3000));

    hx711_tare();



    xTaskCreatePinnedToCore(monitoring_task, "monitoring", 4096, NULL, 5, NULL, 0);

    xTaskCreatePinnedToCore(control_task,    "control",    4096, NULL, 6, NULL, 0);

    xTaskCreatePinnedToCore(buttons_task,    "buttons",    3072, NULL, 7, NULL, 0);

    xTaskCreatePinnedToCore(encoder_task,    "encoder",    3072, NULL, 4, NULL, 1);

    if (g_oled_ok) {

        xTaskCreatePinnedToCore(oled_task,       "oled",       4096, NULL, 3, NULL, 1);

    } else {

        ESP_LOGW(TAG, "No se crea tarea OLED porque no se detecto pantalla");

    }

    xTaskCreatePinnedToCore(logging_task,    "logging",    4096, NULL, 2, NULL, 1);



#if AUTO_START_FOR_TESTS

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

        g_state.state = STATE_ROASTING;

        g_state.roast_start_ms = esp_timer_get_time() / 1000;

        xSemaphoreGive(g_data_mutex);

        ESP_LOGW(TAG, "AUTO_START_FOR_TESTS=1: inicia tostado automaticamente");

    }

#endif



    ESP_LOGW(TAG, "Sistema listo. START en GPIO34 inicia. STOP en GPIO35 enfria/detiene.");

    vTaskDelete(NULL);

}

/** @} */

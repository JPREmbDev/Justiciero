#include "classBmi160.h"        // Incluye la clase para el sensor IMU
#include "classServo.h"         // Incluye la clase para control de servomotores (no utilizada actualmente)
#include "motorController.h"    // Incluye la clase para controlar los motores
#include "pidController.h"      // Incluye la clase del controlador PID
#include "esp_log.h"            // Para funciones de registro en el ESP32
#include <freertos/FreeRTOS.h>  // Para funciones del sistema operativo FreeRTOS
#include <cmath>                // Para funciones matemáticas como abs()
#include <stdio.h>              // Para snprintf

extern "C" void app_main();     // Declaración de la función principal para compatibilidad con C

/* MOTOR 1 - DERECHO */
constexpr gpio_num_t    motor1Ain1      = GPIO_NUM_33;    // Pin de control de dirección 1 del motor derecho
constexpr gpio_num_t    motor1Ain2      = GPIO_NUM_25;    // Pin de control de dirección 2 del motor derecho
constexpr gpio_num_t    motor1Pwm       = GPIO_NUM_32;    // Pin PWM para controlar velocidad del motor derecho
constexpr ledc_timer_t  rightTimer      = LEDC_TIMER_2;   // Timer para el PWM del motor derecho

/* MOTOR 2 - IZQUIERDO */
constexpr gpio_num_t    motor2Bin1      = GPIO_NUM_26;    // Pin de control de dirección 1 del motor izquierdo
constexpr gpio_num_t    motor2Bin2      = GPIO_NUM_27;    // Pin de control de dirección 2 del motor izquierdo
constexpr gpio_num_t    motor2Pwm       = GPIO_NUM_14;    // Pin PWM para controlar velocidad del motor izquierdo
constexpr ledc_timer_t  leftTimer       = LEDC_TIMER_1;   // Timer para el PWM del motor izquierdo

/* CONFIGURACIÓN DEL CONTROL */
constexpr float SAMPLE_TIME = 0.01f;          // Intervalo de tiempo entre iteraciones (10ms = 100Hz)
constexpr float MAX_SAFE_ANGLE = 30.0f;       // Ángulo máximo permitido antes de detener el robot por seguridad
constexpr float INITIAL_KP = 40.0f;           // Componente proporcional del PID - responde al error actual
constexpr float INITIAL_KI = 5.0f;            // Componente integral del PID - reduce errores permanentes
constexpr float INITIAL_KD = 1.0f;            // Componente derivativo del PID - reduce oscilaciones
constexpr float MAX_MOTOR_SPEED = 1023.0f;    // Valor máximo del PWM (10 bits: 0-1023)
constexpr float ANGLE_OFFSET = 0.0f;          // Corrección para alinear el sensor con la vertical "real"

// Objetos globales
Bmi160 imu;                     // Objeto para el sensor IMU BMI160
motorController motor;          // Objeto para controlar los motores
PidController pidController(    // Objeto para el algoritmo de control PID
    INITIAL_KP,                 // Ganancia proporcional inicial
    INITIAL_KI,                 // Ganancia integral inicial
    INITIAL_KD,                 // Ganancia derivativa inicial
    MAX_MOTOR_SPEED,            // Límite máximo de la señal de salida
    ANGLE_OFFSET                // Ángulo objetivo (generalmente cerca de 0)
);

// Función para esperar a que el robot esté en posición vertical antes de iniciar el control
/*
    Espera hasta que el robot esté suficientemente vertical (±5°) antes de activar el control.
    Impide que los motores se activen bruscamente si el robot está inclinado al inicio.
    Tiene un tiempo límite de 5 segundos para encontrar la posición vertical.
    Proporciona retroalimentación mediante logs para que el usuario sepa el estado.
*/
bool waitForUprightPosition() {
    ESP_LOGI("BALANCE", "Esperando posición vertical...");   // Mensaje de log informativo
    Bmi160::AngleData angles = {0};                         // Estructura para almacenar los ángulos
    
    for (int i = 0; i < 500; i++) {  // Bucle con timeout (5 segundos = 500 * 10ms)
        if (imu.getAngles(angles, SAMPLE_TIME) == 0) {      // Obtener ángulos del IMU (retorna 0 si éxito)
            ESP_LOGI("BALANCE", "Pitch: %.2f", angles.pitch); // Mostrar ángulo actual
            
            // Verificar si el robot está cerca de la posición vertical
            if (abs(angles.pitch - ANGLE_OFFSET) < 5.0f) {   // Tolerancia de ±5 grados
                ESP_LOGI("BALANCE", "¡Posición vertical detectada!");
                return true;                                // Éxito: robot en posición vertical
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));                     // Esperar 10ms antes de la siguiente lectura
    }
    
    // Si llega aquí, se agotó el tiempo de espera
    ESP_LOGW("BALANCE", "Tiempo de espera agotado. Coloca el robot en posición vertical.");
    return false;                                          // Fallo: no se alcanzó posición vertical
}

// Función para convertir la señal de control PID en comandos para los motores
/*
    Primero verifica si el ángulo de inclinación es seguro; si no lo es, detiene los motores.
    Determina la dirección basada en el signo de la señal de control:
        Positivo = avanzar (para contrarrestar caída hacia adelante)
        Negativo = retroceder (para contrarrestar caída hacia atrás)
    Convierte el valor absoluto de la señal en la velocidad del motor (PWM).
    Aplica la misma dirección y velocidad a ambos motores para un movimiento recto.
*/
void applyMotorControl(float controlSignal, Bmi160::AngleData &angles) {
    // Verificar si el ángulo es seguro antes de aplicar potencia
    if (abs(angles.pitch - ANGLE_OFFSET) > MAX_SAFE_ANGLE) {
        motor.setSpeed(0, 0);   // Detener ambos motores si el ángulo es peligroso
        ESP_LOGW("BALANCE", "¡Ángulo de inclinación excesivo! Pitch=%.2f", angles.pitch);
        return;                 // Salir sin aplicar potencia
    }
    
    // Determinar dirección basada en la señal de control
    // Si controlSignal > 0, el robot debe moverse hacia adelante para compensar
    bool forward = controlSignal > 0;
    
    // Convertir valor absoluto a velocidad (0-1023)
    // La magnitud de la señal determina cuánta potencia aplicar
    uint16_t speed = abs(controlSignal);
    
    // Aplicar dirección y velocidad a ambos motores
    motor.setDirection(forward, forward);  // Misma dirección para ambos motores
    motor.setSpeed(speed, speed);          // Misma velocidad para ambos motores
    
    // Log de depuración de bajo nivel para monitorear comandos de motor
    ESP_LOGD("BALANCE", "Motor: Dir=%s, Speed=%u", forward ? "FWD" : "REV", speed);
}

void app_main() {

    /* 
        En esta fase inicial:
            Se muestra un mensaje de inicio y se espera 1 segundo para que los sistemas se estabilicen.
            Se configuran los pines GPIO para controlar los motores.
            Se inicializan los timers PWM para regular la velocidad de los motores.
            Se detienen los motores por seguridad.
    */
    ESP_LOGI("BALANCE", "Iniciando robot equilibrista");  // Mensaje inicial
    vTaskDelay(pdMS_TO_TICKS(1000));                     // Espera 1 segundo para estabilizarse

    // Inicializar los controladores de los motores
    motor.gpioPinDefinition(motor1Ain1, motor1Ain2, motor1Pwm, motor2Bin1, motor2Bin2, motor2Pwm);
    motor.initHw(leftTimer, rightTimer);
    
    // Asegurar que los motores estén detenidos al inicio por seguridad
    motor.setSpeed(0, 0);
    
    /*
    Esta sección configura el sensor BMI160:
        Define la configuración SPI con todos los pines necesarios.
        Inicializa la comunicación con el sensor.
        Verifica si la inicialización fue exitosa; si no lo fue, termina el programa.
    */

    // Configuración del sensor IMU mediante interfaz SPI
    Bmi160SpiConfig config = {
        .spiHost = SPI3_HOST,            // Controlador SPI a utilizar
        .csPin = GPIO_NUM_21,            // Pin de selección de chip
        .sclkPin = GPIO_NUM_18,          // Pin de reloj SPI
        .misoPin = GPIO_NUM_19,          // Pin MISO (Master In Slave Out)
        .mosiPin = GPIO_NUM_23,          // Pin MOSI (Master Out Slave In)
        .spiSpeed = 4*1000*1000          // Velocidad de comunicación SPI: 4MHz
    };
    
    // Inicializar el sensor IMU y verificar si hay errores
    if (imu.init(config) != 0) {
        ESP_LOGE("IMU", "Error inicializando IMU!!! Error CRÍTICO, deteniendo Ejecución!!!");  // Log de error
        // TODO: Añadir parpadeo de LED
        for(;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));  // Evitamos que el bucle consuma el 100% de la CPU
        }
    }
    
    /*
    Aquí se preparan las variables para el sistema de control:
        angles: Almacenará las lecturas de orientación del IMU.
        controlSignal: Almacenará la salida del PID para controlar los motores.
        systemActive: Bandera que indica si el control de equilibrio está activo.
        Se llama a waitForUprightPosition() para esperar a que el robot esté vertical.
        Si se detecta posición vertical, se activa el sistema y se reinicia el PID.
    */

    // Variables para el control del equilibrio
    Bmi160::AngleData angles = {0};      // Estructura para almacenar los ángulos
    float controlSignal = 0.0f;          // Señal de control calculada por el PID
    bool systemActive = false;           // Bandera para indicar si el control está activo
    
    // Esperar que el robot esté en posición vertical para iniciar
    if (waitForUprightPosition()) {
        systemActive = true;             // Activar el sistema si está vertical
        pidController.reset();           // Reiniciar el controlador PID (eliminar valores acumulados)
    }
    
    // Iniciar el temporizador del historial PID antes del bucle
    pidController.startHistoryTimer();
    
    // Bucle principal
    ESP_LOGI("BALANCE", "Iniciando bucle de control");
    while (true) {

        /*
            Se leen los datos del sensor IMU para obtener los ángulos actuales.
            Si hay un error en la lectura, se registra una advertencia y se reintenta.
            Se registra el ángulo pitch en logs de depuración para monitoreo continuo.
        */
        // Obtener ángulos actuales del IMU
        if (imu.getAngles(angles, SAMPLE_TIME) != 0) {
            ESP_LOGW("BALANCE", "Error leyendo ángulos");  // Advertencia si hay error
            vTaskDelay(pdMS_TO_TICKS(10));                // Esperar y reintentar
            continue;                                     // Saltar al siguiente ciclo
        }
        
        // Log de bajo nivel para ver el pitch continuamente
        ESP_LOGD("BALANCE", "Ángulo pitch: %.2f°", angles.pitch);
        
        /*
        Si el sistema no está activo (porque el robot se cayó o tuvo una inclinación excesiva):
            Comprueba si el robot ha vuelto a una posición suficientemente vertical.
            Si está vertical, reactiva el control y reinicia el PID.
            Si sigue inclinado, mantiene los motores apagados y continúa monitoreando.
        */

        if (!systemActive) {
            // Comprobar si el robot vuelve a estar en posición vertical
            // para reactivar el control automáticamente
            if (abs(angles.pitch - ANGLE_OFFSET) < MAX_SAFE_ANGLE / 2) {  // Tolerancia de 15°
                ESP_LOGI("BALANCE", "Sistema reactivado");
                systemActive = true;                   // Reactivar el sistema
                pidController.reset();                 // Reiniciar PID para eliminar acumulación
            } else {
                motor.setSpeed(0, 0);                 // Mantener motores apagados
                vTaskDelay(pdMS_TO_TICKS(100));       // Esperar más tiempo (reducir CPU)
                continue;                             // Ir al siguiente ciclo
            }
        }
        
        /*
            Se calcula la señal de control utilizando el algoritmo PID basado en el ángulo pitch.
            Se registra periódicamente (cada 100 ciclos ≈ 1 segundo) el estado actual para monitoreo.
            Se aplica la señal de control a los motores llamando a la función applyMotorControl().
        */
        // Calcular señal de control PID usando el ángulo pitch
        controlSignal = pidController.compute(angles.pitch, SAMPLE_TIME);
        // Log periódico del estado (cada 100 iteraciones ≈ 1 segundo)
        static int logCounter = 0;
        if (++logCounter >= 100) {
            ESP_LOGI("BALANCE", "Pitch: %.2f°, Control: %.2f", angles.pitch, controlSignal);
            logCounter = 0;
        }

        // Aplicar control a los motores
        applyMotorControl(controlSignal, angles);
        vTaskDelay(pdMS_TO_TICKS(10));  // Esperar 5 segundos para simular el control

        // --- Streaming de datos en tiempo real ---
        // Imprimir datos clave en formato CSV para Python en CADA ciclo
        printf("DATA,%lu,%.4f,%.4f\n",
            esp_log_timestamp(), // Timestamp absoluto
            angles.pitch,
            controlSignal);
        // ----------------------------------------

        /*
        Verifica si el robot se ha inclinado demasiado o se ha caído, en cuyo caso:
            Detiene los motores inmediatamente como medida de seguridad.
            Desactiva el sistema de control para evitar movimientos bruscos.
        Mantiene una frecuencia de ejecución constante de 100Hz (10ms) mediante vTaskDelay():
            Esta temporización precisa es crítica para el correcto funcionamiento del PID.
            Permite que el sistema operativo FreeRTOS ejecute otras tareas entre ciclos.
        */
        // Verificar si el robot se cayó o tiene inclinación excesiva
        if (abs(angles.pitch - ANGLE_OFFSET) > MAX_SAFE_ANGLE) {
            ESP_LOGW("BALANCE", "Robot cayó o inclinación excesiva. Deteniendo motores.");
            motor.setSpeed(0, 0);                  // Detener motores inmediatamente
            systemActive = false;                  // Desactivar el sistema de control

            // Volcar el historial PID a la consola
            pidController.dumpHistoryToSerial();
            // pidController.clearHistory(); // Opcional: reset ya lo hace, pero por si acaso

            // Pausa larga o reinicio podría ir aquí
            vTaskDelay(pdMS_TO_TICKS(10000)); // Esperar 10s antes de potencialmente reactivar

        }
        
        // Temporización precisa para el bucle de control
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms = 100Hz - frecuencia crucial para PID estable
    }
}
#include "pidController.h"
#include <algorithm>    // Para std::min, std::max
#include <cmath>        // Para abs()
#include "esp_log.h"
#include "esp_timer.h" // Añadido para esp_log_timestamp()

static const char* TAG = "PID";

/**
 * @brief Constructor del controlador PID.
 * @param kp Ganancia proporcional inicial.
 * @param ki Ganancia integral inicial.
 * @param kd Ganancia derivativa inicial.
 * @param outputLimit Límite máximo de la señal de salida (valor absoluto).
 * @param setpoint Valor objetivo que el sistema debe alcanzar (punto de ajuste).
 * @note Los valores iniciales son importantes para el comportamiento inicial del sistema.
 */
PidController::PidController(float kp, float ki, float kd, 
                           float outputLimit, float setpoint) :
    m_kp(kp),
    m_ki(ki),
    m_kd(kd),
    m_outputLimit(outputLimit),
    m_setpoint(setpoint),
    m_lastInput(0.0f),
    m_integral(0.0f),
    m_lastError(0.0f),
    m_historyStartTime(0), // Inicializar
    m_isHistoryTimerStarted(false) // Inicializar
{
    // Constructor
    m_history.reserve(MAX_HISTORY_POINTS); // Reservar espacio para el historial
}

/**
 * @brief Configura los parámetros de sintonización del controlador PID.
 * @param kp Ganancia proporcional (respuesta proporcional al error actual).
 * @param ki Ganancia integral (respuesta a la acumulación de error pasado).
 * @param kd Ganancia derivativa (respuesta a la tasa de cambio del error).
 * @note Los valores negativos se convierten automáticamente a cero.
 */
void PidController::setTunings(float kp, float ki, float kd) {
    // Asegurar que no hay valores negativos
    m_kp = kp >= 0 ? kp : 0;
    m_ki = ki >= 0 ? ki : 0;
    m_kd = kd >= 0 ? kd : 0;
    
    ESP_LOGI(TAG, "PID sintonizado con Kp=%.2f, Ki=%.2f, Kd=%.2f", m_kp, m_ki, m_kd);
}

/**
 * @brief Establece el límite máximo para la señal de salida del controlador.
 * @param limit Valor máximo (positivo) para la salida. La salida se limitará a [-limit, +limit].
 * @note El límite debe ser positivo. Si se proporciona un valor negativo o cero, se ignora.
 */
void PidController::setOutputLimit(float limit) {
    if (limit > 0) {
        m_outputLimit = limit;
    }
}

/**
 * @brief Establece el valor objetivo (setpoint) que el sistema debe alcanzar.
 * @param setpoint Valor deseado para la variable controlada.
 * @note Para un robot equilibrista, normalmente el setpoint es 0 (posición vertical).
 */
void PidController::setSetpoint(float setpoint) {
    m_setpoint = setpoint;
}

void PidController::startHistoryTimer() {
    m_historyStartTime = esp_log_timestamp();
    m_isHistoryTimerStarted = true;
    ESP_LOGI(TAG, "Temporizador de historial iniciado.");
}

void PidController::clearHistory() {
    m_history.clear();
    ESP_LOGI(TAG, "Historial PID limpiado.");
}

void PidController::dumpHistoryToSerial() const {
    if (m_history.empty()) {
        ESP_LOGI(TAG, "El historial PID está vacío.");
        return;
    }
    ESP_LOGI(TAG, "--- INICIO DATOS PID (Timestamp_ms,Input,Error,PTerm,ITerm,DTerm,Output) ---");
    // Usar printf para salida directa a UART, más fiable para grandes volcados
    printf("Timestamp_ms,Input,Error,PTerm,ITerm,DTerm,Output\n");
    for (const auto& state : m_history) {
         printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               state.timestamp, state.input, state.error,
               state.pTerm, state.iTerm, state.dTerm, state.output);
    }
     printf("--- FIN DATOS PID ---\n");
    // ESP_LOGI(TAG, "--- FIN DATOS PID ---"); // Puede ser menos fiable con muchos datos
}


/**
 * @brief Calcula la señal de control PID basada en el valor de entrada actual.
 * @param input Valor actual de la variable controlada (ángulo de inclinación).
 * @param dt Intervalo de tiempo en segundos desde la última iteración.
 * @return float Señal de control calculada, limitada al rango configurado.
 * @note La función utiliza derivada sobre la medición en lugar de sobre el error
 *       para evitar picos derivativos ("derivative kick") cuando cambia el setpoint.
 */
float PidController::compute(float input, float dt) {
    // Calcular el error
    float error = m_setpoint - input;

    // Término proporcional
    float pTerm = m_kp * error;

    // Término integral con anti-windup
    m_integral += error * dt;
    float maxIntegral = (m_ki != 0) ? (m_outputLimit / m_ki) : 0; // Evitar división por cero
    if (maxIntegral != 0) { // Aplicar límite solo si Ki no es cero
         m_integral = std::max(-maxIntegral, std::min(m_integral, maxIntegral));
    }
    float iTerm = m_ki * m_integral;

    // Término derivativo (sobre la medición)
    float dInput = (dt > 0) ? (input - m_lastInput) / dt : 0; // Evitar división por cero
    float dTerm = -m_kd * dInput;
    m_lastInput = input;

    // Calcular salida total
    float output = pTerm + iTerm + dTerm;

    // Limitar la salida
    output = std::max(-m_outputLimit, std::min(output, m_outputLimit));

    // Almacenar estado en el historial si hay espacio y el timer está iniciado
    if (m_isHistoryTimerStarted && m_history.size() < MAX_HISTORY_POINTS) {
        uint32_t currentTimestamp = esp_log_timestamp() - m_historyStartTime;
        m_history.push_back({currentTimestamp, input, error, pTerm, iTerm, dTerm, output});
    }

    ESP_LOGD(TAG, "PID: Input=%.2f, Error=%.2f, P=%.2f, I=%.2f, D=%.2f, Output=%.2f",
             input, error, pTerm, iTerm, dTerm, output);

    return output;
}

/**
 * @brief Reinicia el estado interno del controlador PID.
 * @note Útil cuando se reinicia el control o cuando el robot se ha caído y se vuelve a colocar.
 *       Esto evita comportamientos bruscos causados por la acumulación previa de error integral.
 */
void PidController::reset() {
    m_integral = 0.0f;
    m_lastInput = 0.0f;
    m_lastError = 0.0f;
    clearHistory(); // Limpiar historial al resetear
    // No reiniciamos el timer aquí, podría quererse seguir logueando si se reactiva
    ESP_LOGI(TAG, "PID reiniciado (historial limpiado).");
}
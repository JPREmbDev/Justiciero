#ifndef PID_CONTROLLER_H__
#define PID_CONTROLLER_H__

#include <stdint.h>
#include <vector>   
#include <stdio.h>  

class PidController {
public:

    // --- Estructura interna para datos históricos ---
    struct PidState {
        uint32_t timestamp; // Tiempo relativo en ms
        float input;
        float error;
        float pTerm;
        float iTerm;
        float dTerm;
        float output;
    };

    // --- Constante para tamaño del historial ---
    static const int MAX_HISTORY_POINTS = 500; // 5 segundos a 100Hz    

    /**
     * @brief Constructor del controlador PID
     * @param kp Ganancia proporcional inicial
     * @param ki Ganancia integral inicial
     * @param kd Ganancia derivativa inicial
     * @param outputLimit Límite de la señal de salida
     * @param dt Tiempo entre iteraciones en segundos
     */
    PidController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, 
                 float outputLimit = 1023.0f, float setpoint = 0.0f);
    
    /**
     * @brief Configura los parámetros PID
     * @param kp Ganancia proporcional
     * @param ki Ganancia integral
     * @param kd Ganancia derivativa
     */
    void setTunings(float kp, float ki, float kd);
    
    /**
     * @brief Establece el límite de salida
     * @param limit Valor máximo (positivo y negativo) de la señal de salida
     */
    void setOutputLimit(float limit);
    
    /**
     * @brief Establece el valor deseado (setpoint)
     * @param setpoint Valor objetivo que el sistema debe alcanzar
     */
    void setSetpoint(float setpoint);
    
    /**
     * @brief Calcula la señal de control PID
     * @param input Valor actual de la variable a controlar
     * @param dt Tiempo transcurrido desde la última iteración en segundos
     * @return Señal de control calculada
     */
    float compute(float input, float dt);
    
    /**
     * @brief Resetea el estado interno del controlador
     */
    void reset();
    

    // --- Nuevos métodos para historial ---
    /**
     * @brief Imprime el historial de datos PID acumulado en formato CSV por el puerto serie.
     */
    void dumpHistoryToSerial() const;

    /**
     * @brief Limpia el historial de datos PID acumulado.
     */
    void clearHistory();

    /**
     * @brief Inicia la captura de tiempo para el historial. Llamar antes de empezar el bucle.
     */
    void startHistoryTimer();

    
private:
    // Parámetros del controlador
    float m_kp;             // Ganancia proporcional
    float m_ki;             // Ganancia integral
    float m_kd;             // Ganancia derivativa
    float m_outputLimit;    // Límite de la señal de salida
    float m_setpoint;       // Valor deseado
    
    // Variables de estado
    float m_lastInput;      // Valor anterior de entrada para calcular derivada
    float m_integral;       // Suma del error para el término integral
    float m_lastError;      // Error anterior para cálculo alternativo de derivada

    // --- Nuevas variables para historial ---
    std::vector<PidState> m_history;
    uint32_t m_historyStartTime; // Tiempo de inicio para timestamps relativos
    bool m_isHistoryTimerStarted;

};

#endif // PID_CONTROLLER_H__
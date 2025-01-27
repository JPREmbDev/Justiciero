# Driver SPI para BMI160 en ESP32
Documentación técnica y explicación del proyecto de comunicación con el sensor BMI160 mediante SPI.

---

## Capítulo 1: Configuración Básica e Inicialización

### Hardware Utilizado
- **Placa**: ESP32 (NodeMCU-32S)
- **Sensor**: BMI160 (Acelerómetro/Giroscopio)
- **Conexiones SPI**:
  | Pin ESP32 | Función  | Pin BMI160 |
  |-----------|----------|------------|
  | GPIO23    | MOSI     | SDO/SDI    |
  | GPIO19    | MISO     | SDA/SDO    |
  | GPIO18    | SCLK     | SCL        |
  | GPIO5     | CS       | CSB        |

---

### Explicación de Máscaras y Decisiones Técnicas

#### **Máscara `0x80`**
- **Propósito**: Habilitar modo de lectura.
- **Detalle**:  
  El BMI160 requiere que el bit 7 (MSB) de la dirección del registro sea `1` para operaciones de lectura (ver sección 5.2 del datasheet).  
  **Ejemplo**:  
  `0x7F (dirección) | 0x80 = 0xFF (operación de lectura)`.

#### **Modo 0 SPI 3**
- **Elección**:  
  El BMI160 soporta modos 0 (CPOL=0, CPHA=0) y 3 (CPOL=1, CPHA=1).  
  Se seleccionó el modo 0 por:
  - Compatibilidad con configuraciones estándar.
  - Menor consumo energético en estado inactivo (CLK en LOW).

#### **Frecuencia de 1 MHz**
- **Razón**:  
  - Límite máximo seguro para evitar errores de sincronización inicial.
  - Dentro del rango soportado por el sensor (hasta 10 MHz en modo SPI).
  - Permite margen para futuras optimizaciones.

#### **GPIO5 como CS**
- **Base**:  
  Según el diagrama de referencia del [Waveshare NodeMCU-32S](https://www.waveshare.com/img/devkit/accBoard/NodeMCU-32S/NodeMCU-32S-details-3.jpg), GPIO5 está libre y es adecuado para CS.

---

### Estructura del Código
```txt
/Proyecto
├── main.cpp            # Punto de entrada
├── localBmi160.cpp     # Driver SPI del BMI160
├── localBmi160.h       # Cabeceras
└── README.md           # Documentación
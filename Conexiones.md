# Tablas de Conexiones - Proyecto PTS900

## ESP32-C3 (Transmisor - Dispositivo del Árbitro)

| Periférico | Pin GPIO | Función | Notas |
| :--- | :--- | :--- | :--- |
| **Micrófono INMP441** | GPIO 1 | I2S SCK (Serial Clock) | Reloj de datos I2S |
| | GPIO 2 | I2S WS (Word Select/LRCK) | Selección de canal izquierdo/derecho |
| | GPIO 3 | I2S SD (Serial Data) | Datos de audio |
| **LoRa RF96 (SPI)** | GPIO 4 | SCK | Reloj SPI |
| | GPIO 5 | MISO | Datos entrada SPI |
| | GPIO 6 | MOSI | Datos salida SPI |
| | GPIO 7 | CS (Chip Select) | Selección de chip LoRa |
| **LoRa (Control)** | GPIO 10 | RST | Reset del módulo LoRa |
| | GPIO 20 | DIO0 | Interrupción de recepción/transmisión |
| **Botón Verde (Start)** | **GPIO 2** | Entrada digital | Pull-up interno. Envía 'S' al presionar |
| **Botón Rojo (Stop)** | **GPIO 9** | Entrada digital | Pull-up interno. Envía 'P' al presionar |
| **Monitoreo Batería** | **GPIO 0** | ADC | Divisor de tensión (Vbat/2) |
| **Energía** | VCC | 3.3V | Alimentación desde TP4056 |
| | GND | GND | Tierra común |
| | VBUS (5V) | USB/ Carga | Entrada de carga USB-C |

---

## ESP32-S3 (Receptor - Estación Base)

| Periférico | Pin GPIO | Función | Notas |
| :--- | :--- | :--- | :--- |
| **LoRa RF96 (SPI)** | GPIO 12 | SCK | Reloj SPI |
| | GPIO 13 | MISO | Datos entrada SPI |
| | GPIO 14 | MOSI | Datos salida SPI |
| | GPIO 15 | CS (Chip Select) | Selección de chip LoRa |
| **LoRa (Control)** | GPIO 16 | RST | Reset del módulo LoRa |
| | GPIO 17 | DIO0 | Interrupción de recepción/transmisión |
| **Energía** | VCC | 3.3V | Alimentación estable |
| | GND | GND | Tierra común |
| | USB | USB-C | Programación y alimentación |

---

## Resumen de Protocolos por Dispositivo

### ESP32-C3
- **I2S**: Micrófono digital INMP441
- **SPI**: Comunicación con módulo LoRa
- **GPIO**: Botones y control de LoRa
- **ADC**: Monitoreo de nivel de batería

### ESP32-S3
- **SPI**: Comunicación con módulo LoRa
- **I2C**: Pantalla OLED (opcional)
- **WiFi**: Servidor Web y modo AP
- **GPIO**: LEDs de estado y botón de control

---

## Notas de Diseño

1. **ESP32-C3**: Optimizado para bajo consumo. Opera a 80MHz para maximizar autonomía (~9 horas).

2. **ESP32-S3**: Utiliza doble núcleo:
   - **Core 0**: Radio LoRa (recepción continua)
   - **Core 1**: Servidor Web y lógica de cronometraje

3. **Distancia entre dispositivos**: Los módulos LoRa RF96 permiten comunicación hasta 1-2 km en línea de vista.

4. **Alimentación**:
   - C3: Batería LiPo 250-600mAh con TP4056
   - S3: Puede usar alimentación USB o batería LiPo más grande

# Control de Vuelo Adaptativo para Helicóptero 1-DOF 🚁

Este repositorio contiene el código fuente y documentación para el sistema de control digital de un helicóptero de un grado de libertad (1-DOF). El proyecto implementa estrategias de control avanzado sobre un **ESP32**, incluyendo **Gain Scheduling** y **PID de Velocidad**, supervisado mediante una **HMI en Python** con soporte para mando de consola (Xbox/PS4).

## 📋 Descripción

El objetivo del proyecto es estabilizar un brazo basculante propulsado por un motor *brushless*. Debido a la no-linealidad introducida por la gravedad y la aerodinámica, un PID clásico es insuficiente. Este sistema implementa:

1.  **Firmware (ESP32):** Ejecución de lazo cerrado en tiempo real (100Hz), filtrado de sensores (Kalman) y generación de PWM.
2.  **Software (PC):** Interfaz gráfica para telemetría, sintonización de ganancias "al vuelo" y cálculo de rectas de *Gain Scheduling*.
3.  **Control Manual:** Integración de Joystick USB con lógica de control incremental.

## 🚀 Características Principales

* **Arquitectura Dual-Core:** El ESP32 separa la lógica de control crítico (Core 1) de la comunicación Serial (Core 0).
* **Fusión Sensorial:** Filtro de Kalman para la IMU MPU6050.
* **Gain Scheduling:** Ajuste dinámico de Kp, Ki, Kd en función del ángulo de ataque, calculado por la HMI.
* **Modos de Operación:**
    * 🛠 **Manual:** Control directo del ciclo de trabajo (PWM).
    * 🤖 **Auto PID:** Control de posición angular con referencia dinámica.
* **Soporte HID:** Control de referencia mediante mando de videojuegos (Xbox/DualShock) vía USB con zona muerta por software.
* **Data Logging:** Grabación de experimentos en formato CSV para análisis en MATLAB/Python.

## 📂 Estructura del Repositorio

├── firmware/
│   └── PIDInicial/
│       └── PIDInicial.ino       # Código C++ para ESP32 (Arduino Framework)
├── hmi/
│   └── HMIPID.py                # Dashboard en Python (Tkinter + Matplotlib + Pygame)
└── README.md                    # Documentación del proyecto

## 🛠️ Requisitos de Hardware

* **Microcontrolador:** ESP32 DevKit V1.
* **Sensor:** MPU6050 (Acelerómetro + Giroscopio).
* **Actuador:** Motor Brushless A2212 (1000KV) + ESC 30A.
* **Fuente:** 12V DC (para el motor) y USB (para el ESP32).
* **Control:** Mando Xbox One o PS4 (conexión USB).

## 💻 Instalación y Uso

### 1. Firmware (ESP32)
1.  Abrir `firmware/PIDInicial.ino` en **Arduino IDE** o **PlatformIO**.
2.  Instalar dependencias necesarias (Librería `Wire`).
3.  Configurar la placa **DOIT ESP32 DEVKIT V1**.
4.  Subir el código a la placa.

### 2. HMI (Python)
Asegúrate de tener Python 3.10+ instalado. Instala las librerías requeridas:

pip install pyserial pygame matplotlib tk

### 3. Ejecución
1.  Conecta el ESP32 y el mando USB a la computadora.
2.  Ejecuta el script de la interfaz:
    python hmi/HMIPID.py
3.  En la interfaz:
    * Selecciona el **Puerto COM** del ESP32.
    * Haz clic en **CONECTAR**.
    * Usa el mando o los sliders para controlar el sistema.

## 🧠 Lógica de Control

### Gain Scheduling (Sintonización Dinámica)
La HMI calcula las ganancias óptimas basándose en la linealización de la planta en diferentes puntos de operación. Las rectas implementadas son:

Kp(theta) = -0.000439 * theta + 0.0308
Ki(theta) = -0.013179 * theta + 0.9246
Kd(theta) = -0.000049 * theta + 0.0034

Estos valores se envían al ESP32 automáticamente cuando el ángulo cambia, asegurando una respuesta uniforme en todo el rango de movimiento.

## 👥 Autores

Proyecto desarrollado para la asignatura de **Control Digital**.

* **Mateo Francisco Chimbo Quezada** - mateo.chimbo@ucuenca.edu.ec
* **Angel Ramiro Apolo Aguilar** - angel.apolo@ucuenca.edu.ec
* **Stalyn Antonio Ochoa Yanez** - stalyn.ochoa@ucuenca.edu.ec

---
*Universidad de Cuenca - Facultad de Ingeniería*

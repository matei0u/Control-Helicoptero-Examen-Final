# Control de Vuelo Adaptativo para Helicóptero 1-DOF 🚁

Este repositorio contiene el código fuente y documentación para el sistema de control digital de un helicóptero de un grado de libertad (1-DOF). El proyecto explora y compara estrategias de control avanzado sobre un **ESP32**, implementando tanto **Gain Scheduling** (PID Dinámico) como **Smart Clegg Integrator** (Control de Reinicio), supervisado mediante una **HMI en Python** con soporte para mando de consola.

## 📋 Descripción

El objetivo es estabilizar un brazo basculante propulsado por un motor *brushless*. Dado que el sistema presenta fuertes no-linealidades (zona muerta y gravedad variable), se han desarrollado dos arquitecturas de firmware distintas:

1.  **Firmware V1 (Gain Scheduling):** Ajuste dinámico de ganancias PID basado en el ángulo de ataque.
2.  **Firmware V2 (Smart Clegg):** Controlador PID no-lineal con reinicio del integrador condicional para eliminar sobretiros sin perder estabilidad en el "hover".
3.  **Software HMI (PC):** Interfaz gráfica para telemetría en tiempo real y sintonización de parámetros.

## 🚀 Características Principales

* **Arquitectura Dual-Core:** El ESP32 separa la lógica de control crítico (Core 1, 100Hz) de la comunicación (Core 0).
* **Fusión Sensorial:** Filtro de Kalman para la IMU MPU6050.
* **Estrategias de Control:**
    * *Gain Scheduling:* Linealización por tramos calculada en la HMI.
    * *Smart Clegg:* Reinicio inteligente del integrador (Reset Control) con zona muerta para evitar oscilaciones (Chattering).
* **Modos de Operación:**
    * 🛠 **Manual:** Control directo del PWM.
    * 🤖 **Auto:** Lazo cerrado de posición.
* **Soporte HID:** Control manual mediante mando de videojuegos (Xbox/PS4) vía USB.

## 🛠️ Requisitos de Hardware

* **Microcontrolador:** ESP32 DevKit V1.
* **Sensor:** MPU6050.
* **Actuador:** Motor Brushless A2212 (1000KV) + ESC 30A.
* **Fuente:** 12V DC.
* **Control:** Mando Xbox One o PS4 (USB).

## 💻 Instalación y Uso

1.  **Firmware:**
    * Elige la estrategia deseada (`PIDInicial` o `PIDClegg`).
    * Abre el archivo `.ino` correspondiente y súbelo al ESP32.
2.  **HMI (Python):**
    * Instala librerías: `pip install pyserial pygame matplotlib tk`
    * Ejecuta: `python hmi/HMIPID.py`
3.  **Operación:**
    * Conecta el mando y el ESP32.
    * En la HMI, selecciona el puerto COM y conecta.

## 🧠 Lógica de Control

### Estrategia 1: Gain Scheduling
La HMI calcula ganancias variables ($K_p, K_i, K_d$) según la ecuación de la recta de operación y las envía al ESP32 en tiempo real.

### Estrategia 2: Smart Clegg Integrator
Implementada en `PIDClegg.ino`. El integrador se reinicia (se hace cero) solo si se cumple una condición de inestabilidad, evitando el sobretiro (overshoot) tras una perturbación:

Si (Cruce por Cero) Y (|Error Anterior| > Umbral Estabilidad):
    Integral = Integral * Factor_Reinicio
Sino:
    Integral = Integral + (Ki * Error)  // Integración Normal

Esto permite que el helicóptero frene rápido al llegar a la referencia, pero mantenga la fuerza necesaria para flotar suavemente cuando el error es pequeño (< 0.3°).

## 👥 Autores

Proyecto desarrollado para la asignatura de **Control Digital**.

* **Mateo Francisco Chimbo Quezada** - *mateo.chimbo@ucuenca.edu.ec*
* **Angel Ramiro Apolo Aguilar** - *angel.apolo@ucuenca.edu.ec*
* **Stalyn Antonio Ochoa Yanez** - *stalyn.ochoa@ucuenca.edu.ec*

---
*Universidad de Cuenca - Facultad de Ingeniería*

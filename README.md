# ROLDAC – Robot Limpiador de Aguas Contaminadas

## Descripción General

**ROLDAC (Robot Limpiador de Aguas Contaminadas)** es un sistema autónomo diseñado para la detección y limpieza en cuerpos de agua.  
El proyecto integra visión por computadora, sensores de calidad del agua, propulsión con motores brushless, control de inmersión y radar ultrasónico, todo gestionado desde una **Raspberry Pi 5** mediante un servidor web en **Flask**.

El objetivo principal es ofrecer una plataforma de investigación y desarrollo orientada a la **automatización de limpieza acuática** con monitoreo ambiental en tiempo real.

---

## Arquitectura del Sistema

El sistema se compone de los siguientes módulos:

- **Servidor Central (Raspberry Pi 5)**  
  - Backend en Flask para comunicación y control.  
  - Captura de video con PiCamera2.  
  - Procesamiento con MobileNetSSD para detección de basura.  
  - Interfaz web interactiva (HTML/CSS/JS).  
  - Pantalla LCD I2C para mostrar dirección IP y puerto.  

- **Módulos Arduino**  
  - **Control de Motores**: 2 motores brushless + ESC.  
  - **Radar Ultrasónico**: HC-SR04 montado en servo.  
  - **Sensores de Agua**: pH, TDS, turbidez y temperatura.  
  - **Inmersión y Dirección**: bombas controladas por puente H L298N y servos para orientación.  

- **Interfaz Web**  
  - Stream de video en tiempo real con detección.  
  - Joystick virtual para dirección.  
  - Slider para velocidad.  
  - Visualización del radar en canvas.  
  - Panel de calidad de agua con indicadores.  

---

## Requisitos de Hardware

- Raspberry Pi 5 con Raspberry Pi OS.  
- PiCamera V3.  
- 4x Arduino Nano (motores, radar, sensores, inmersión).  
- 2x Motores brushless + ESC.  
- 2x Servos SG90/MG996R para dirección.  
- Sensor ultrasónico HC-SR04.  
- Bombas de agua con controlador L298N.  
- Pantalla LCD 16x2 I2C.  

---

## Requisitos de Software

- **Python 3.11+**  
- Dependencias principales:
  - Flask  
  - OpenCV  
  - NumPy  
  - Picamera2  
  - gpiozero  
  - pyserial  
  - netifaces  

Instalación de dependencias:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-picamera2 python3-gpiozero
pip install -r requirements.txt

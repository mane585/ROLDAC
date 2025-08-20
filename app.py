import cv2
import numpy as np
import time
import serial
import threading
import re
import netifaces as ni
from flask import Flask, render_template, Response, request, jsonify
from gpiozero import OutputDevice
from picamera2 import Picamera2
from lcd_i2c import mostrar_ip_puerto

# ===================== CONFIGURACIÓN INICIAL =====================
app = Flask(__name__)

# GPIO
PIN_CONTROL_1 = 23
PIN_CONTROL_2 = 24
PIN_CONTROL_3 = 25

# Dispositivos seriales
SERIAL_PORTS = {
    'motores': '/dev/arduino_motores',
    'sensores': '/dev/arduino_sensores',
    'radar': '/dev/arduino_radar'
}

# Modelo de detección
MODEL_CONFIG = {
    'prototxt': "models/MobileNetSSD_deploy.prototxt",
    'model': "models/MobileNetSSD_deploy.caffemodel",
    'classes': ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", 
                "bus", "car", "cat", "chair", "cow", "diningtable", "dog", 
                "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", 
                "train", "tvmonitor"],
    'trash_objects': ["bottle", "can", "cup"],
    'confidence_threshold': 0.45,
    'frame_skip': 2,
    'min_area': 500
}

# ===================== INICIALIZACIÓN =====================
# GPIO
pin_control1 = OutputDevice(PIN_CONTROL_1)
pin_control2 = OutputDevice(PIN_CONTROL_2)
pin_control3 = OutputDevice(PIN_CONTROL_3)

# Variables de estado
state = {
    'servo_actual': "frw",
    'deteccion_activa': True,
    'radar_active': False,
    'sensores_active': False,
    'last_detected': [],
    'fps_counter': 0,
    'last_fps_time': time.time(),
    'frame_counter': 0
}

# Datos de sensores
sensor_data = {
    "temp": None,
    "ph": None,
    "turb": None,
    "tds": None
}

# Radar
radar_data = {
    "angulo": 90,
    "distancia": 0,
    "status": "disconnected"
}

# ===================== FUNCIONES DE INICIALIZACIÓN =====================
def init_camera():
    """Inicializa la cámara Picamera2"""
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (320, 240)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.configure("preview")
    picam2.start()
    time.sleep(1)  # Warm-up
    return picam2

def init_model():
    """Carga el modelo de detección de objetos"""
    try:
        net = cv2.dnn.readNetFromCaffe(MODEL_CONFIG['prototxt'], MODEL_CONFIG['model'])
        print("[INFO] Modelo cargado exitosamente")
        return net
    except Exception as e:
        print(f"[ERROR] No se pudo cargar el modelo: {e}")
        return None

def init_serial(port_name):
    """Inicializa conexión serial con un dispositivo"""
    try:
        ser = serial.Serial(SERIAL_PORTS[port_name], 9600, timeout=1)
        time.sleep(2)  # Tiempo de inicialización
        print(f"[INFO] {port_name.capitalize()} conectado en {SERIAL_PORTS[port_name]}")
        return ser
    except Exception as e:
        print(f"[ERROR] No se pudo conectar {port_name}: {e}")
        return None

# ===================== FUNCIONES DE HILOS =====================
def read_sensor_data(ser):
    """Hilo para leer datos de sensores"""
    pattern = re.compile(r"TEMP:(?P<temp>\d+(?:\.\d+)?),PH:(?P<ph>\d+(?:\.\d+)?),TURB:(?P<turb>\d+(?:\.\d+)?),TDS:(?P<tds>\d+(?:\.\d+)?)")
    while state['sensores_active'] and ser and ser.is_open:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line and (match := pattern.match(line)):
                sensor_data.update({
                    "temp": float(match.group("temp")),
                    "ph": float(match.group("ph")),
                    "turb": float(match.group("turb")),
                    "tds": float(match.group("tds"))
                })
        except Exception as e:
            print(f"[ERROR] Lectura serial sensores: {e}")
            time.sleep(0.1)

def read_radar_data(ser):
    """Hilo para leer datos del radar"""
    while state['radar_active'] and ser and ser.is_open:
        try:
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                if line:
                    try:
                        angle, distance = map(float, line.split(','))
                        if 0 <= angle <= 180 and 0 <= distance <= 200:
                            radar_data.update({
                                "angulo": angle,
                                "distancia": distance,
                                "status": "receiving"
                            })
                    except ValueError:
                        continue
        except Exception as e:
            print(f"[ERROR] Lectura radar: {e}")
            time.sleep(0.1)

# ===================== FUNCIONES DE VISIÓN POR COMPUTADORA =====================
def detect_objects(frame, net):
    """Detecta objetos en un frame de video"""
    detected = []
    state['frame_counter'] += 1
    
    if state['frame_counter'] % MODEL_CONFIG['frame_skip'] != 0 or not net:
        return frame, detected

    try:
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > MODEL_CONFIG['confidence_threshold']:
                idx = int(detections[0, 0, i, 1])
                label = MODEL_CONFIG['classes'][idx]

                if label in MODEL_CONFIG['trash_objects']:
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")
                    area = (endX - startX) * (endY - startY)
                    
                    if area >= MODEL_CONFIG['min_area']:
                        startX, startY = max(0, startX), max(0, startY)
                        endX, endY = min(w-1, endX), min(h-1, endY)

                        color = (0, 255, 0)  # Verde por defecto
                        if label == "can": color = (0, 165, 255)
                        elif label == "cup": color = (255, 0, 0)

                        cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)
                        y = startY - 15 if startY - 15 > 15 else startY + 15
                        cv2.putText(frame, f"{label}: {confidence:.2f}",
                                   (startX, y), cv2.FONT_HERSHEY_SIMPLEX,
                                   0.5, color, 2)

                        detected.append({
                            "label": label,
                            "confidence": float(confidence),
                            "box": (startX, startY, endX, endY),
                            "area": area
                        })

        state['last_detected'] = detected
    except Exception as e:
        print(f"[ERROR] Detección: {e}")

    return frame, detected

def generate_frames(picam2, net):
    """Generador de frames para el streaming"""
    while True:
        try:
            frame = picam2.capture_array()
            
            # Contador FPS
            state['fps_counter'] += 1
            if time.time() - state['last_fps_time'] >= 1.0:
                state['fps_counter'] = 0
                state['last_fps_time'] = time.time()

            # Detección de objetos
            if state['deteccion_activa']:
                frame, _ = detect_objects(frame, net)
            else:
                state['last_detected'] = []

            # Codificar frame
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
        except Exception as e:
            print(f"[ERROR] Captura de frame: {e}")
            continue

# ===================== FUNCIONES DE CONTROL =====================
def set_servo_command(c1, c2, c3, delay=0.1):
   
    try:
        pin_control1.value = c3
        pin_control2.value = c2
        pin_control3.value = c1
        time.sleep(delay)
        # Actualizar estado global
        state['servo_actual'] = f"{int(c1)}{int(c2)}{int(c3)}"
    except Exception as e:
        print(f"[ERROR] Control de servos: {e}")
        raise

def send_motor_command(ser, speed, direction):
    """Envía comando a los motores"""
    if ser and ser.is_open:
        try:
            ser.write(f"{speed},{direction}\n".encode())
        except Exception as e:
            print(f"[ERROR] Envío a motores: {e}")

def evaluar_calidad(valor, rango_bueno, rango_medio):
    """Evalúa la calidad de un parámetro según rangos"""
    if valor is None:
        return "red"
    if rango_bueno[0] <= valor <= rango_bueno[1]:
        return "green"
    elif rango_medio[0] <= valor <= rango_medio[1]:
        return "yellow"
    else:
        return "red"

# ===================== Conexiones con el servidor =====================
@app.route('/')
def index():
    """Ruta principal que renderiza la interfaz web"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Streaming de video en tiempo real"""
    return Response(generate_frames(picam2, net), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/radar_data')
def get_radar():
    """Devuelve datos del radar en formato JSON"""
    return jsonify(radar_data)

@app.route('/sensor_data')
def get_sensors():
    """Devuelve datos de los sensores con indicadores de calidad"""
    return jsonify({
        "temp": sensor_data["temp"],
        "temp_color": evaluar_calidad(sensor_data["temp"], (10, 25), (7, 10)),
        "ph": sensor_data["ph"],
        "ph_color": evaluar_calidad(sensor_data["ph"], (6.5, 8.0), (6.0, 6.5)),
        "turb": sensor_data["turb"],
        "turb_color": evaluar_calidad(sensor_data["turb"], (0, 10), (10, 50)),
        "tds": sensor_data["tds"],
        "tds_color": evaluar_calidad(sensor_data["tds"], (0, 300), (300, 600))
    })

@app.route('/update_motors', methods=['POST'])
def update_motors():
    """Controla los motores principales"""
    data = request.get_json()
    speed = data.get("velocidad", 0)
    direction = data.get("direccion", 50)
    send_motor_command(arduino_motores, speed, direction)
    return jsonify(success=True)

@app.route('/control_servo', methods=['POST'])
def control_servo():
    """Controla los servos de dirección"""
    data = request.get_json()
    cmd = data.get("cmd")
    
    commands = {
        "up": (False, False, True),
        "frw": (False, False, False),
        "bkw": (False, True, True),
        "izq": (False, True, False),
        "der": (True, False, True)
    }
    
    if cmd in commands:
        set_servo_command(*commands[cmd])
        return jsonify(success=True, message=f"Dirección ajustada a {cmd}")
    return jsonify(success=False, error="Comando no válido. Use: up, frw, bkw, izq, der")

@app.route('/control_profundidad', methods=['POST'])
def control_profundidad():
    """Controla los motores de profundidad"""
    data = request.get_json()
    accion = data.get("accion")
    
    commands = {
        "subir": (True, False, False),
        "bajar": (True, True, True),
        "neutro": (True, True, False)
    }
    
    if accion in commands:
        set_servo_command(*commands[accion], delay=0.5)
        return jsonify(success=True, message=f"Profundidad ajustada a {accion}")
    return jsonify(success=False, error="Comando no válido. Use: subir, bajar, neutro")

@app.route('/toggle_detection', methods=['POST'])
def toggle_detection():
    """Activa/desactiva la detección de objetos"""
    state['deteccion_activa'] = not state['deteccion_activa']
    return jsonify({"detection": state['deteccion_activa']})

@app.route('/start_radar', methods=['POST'])
def start_radar():
    """Inicia el escaneo del radar"""
    if arduino_radar and arduino_radar.is_open:
        arduino_radar.write(b'START\n')
        radar_data["status"] = "scanning"
        return jsonify(success=True, status=radar_data["status"])
    return jsonify(success=False, status="disconnected")

@app.route('/stop_radar', methods=['POST'])
def stop_radar():
    """Detiene el escaneo del radar"""
    if arduino_radar and arduino_radar.is_open:
        arduino_radar.write(b'STOP\n')
        radar_data["status"] = "idle"
        return jsonify(success=True, status=radar_data["status"])
    return jsonify(success=False, status="disconnected")

@app.route("/reconectar_todos", methods=["POST"])
def reconnect():
    """Reinicia todas las conexiones seriales y relanza los hilos de sensores y radar"""
    global arduino_motores, arduino_sensores, arduino_radar

    try:
        # 1. Cerrar conexiones existentes
        for ser in [arduino_motores, arduino_sensores, arduino_radar]:
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception as e:
                print(f"[WARN] Error cerrando puerto: {e}")

        # 2. Reabrir conexiones
        arduino_motores = init_serial('motores')
        arduino_sensores = init_serial('sensores')
        arduino_radar   = init_serial('radar')

        # 3. Reiniciar banderas de estado
        state['sensores_active'] = bool(arduino_sensores)
        state['radar_active']    = bool(arduino_radar)

        # 4. Lanzar hilos nuevamente
        if arduino_sensores:
            threading.Thread(
                target=read_sensor_data, 
                args=(arduino_sensores,), 
                daemon=True
            ).start()

        if arduino_radar:
            threading.Thread(
                target=read_radar_data, 
                args=(arduino_radar,), 
                daemon=True
            ).start()

        return jsonify({
            "status": "ok",
            "msg": "Reconexión exitosa",
            "motores": arduino_motores.is_open if arduino_motores else False,
            "sensores": arduino_sensores.is_open if arduino_sensores else False,
            "radar": arduino_radar.is_open if arduino_radar else False
        })

    except Exception as e:
        print(f"[ERROR] Reconexión fallida: {e}")
        return jsonify({"status": "error", "msg": str(e)})


# ===================== INICIALIZACIÓN Y EJECUCIÓN =====================
if __name__ == '__main__':
    # Inicialización de hardware
    picam2 = init_camera()
    net = init_model()
    
    # Conexiones seriales
    arduino_motores = init_serial('motores')
    arduino_sensores = init_serial('sensores')
    arduino_radar = init_serial('radar')
    
    # Hilos para sensores y radar
    if arduino_sensores:
        state['sensores_active'] = True
        threading.Thread(target=read_sensor_data, args=(arduino_sensores,), daemon=True).start()
    
    if arduino_radar:
        state['radar_active'] = True
        threading.Thread(target=read_radar_data, args=(arduino_radar,), daemon=True).start()
    
    # Mostrar IP en LCD
    try:
        ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
        mostrar_ip_puerto(ip, "5000")
    except:
        print("No se pudo obtener la IP de wlan0")
    
    # Ejecutar aplicación
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        # Limpieza al cerrar
        state.update({
            'sensores_active': False,
            'radar_active': False
        })
        picam2.stop()
        for ser in [arduino_motores, arduino_sensores, arduino_radar]:
            if ser and ser.is_open:
                ser.close()
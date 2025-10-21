# app.py
# Roldac V2 – Motores duales (M1/M2) por serial, Servos por ángulo/presets y Profundidad PID en 4º Arduino
# Mantiene: video (Picamera2 + detección MobileNetSSD), sensores de agua, radar ultrasónico, LCD I2C
# 21/10/25

# ===================== IMPORTS =====================
import cv2
import numpy as np
import time
import serial
import threading
import re
import netifaces as ni
from flask import Flask, render_template, Response, request, jsonify
from picamera2 import Picamera2
from lcd_i2c import mostrar_ip_puerto

# ===================== CONFIGURACIÓN INICIAL =====================
app = Flask(__name__)

# Puertos seriales 
SERIAL_PORTS = {
    'motores': '/dev/arduino_motores',          # Arduino Motores (M1/M2 independientes)
    'sensores': '/dev/arduino_sensores',        # Arduino Sensores (TDS, turbidez, pH, temp)
    'radar': '/dev/arduino_radar',              # Arduino Radar (servo+HC-SR04)
    'servos_depth': '/dev/arduino_servos_depth' # Arduino Servos+Profundidad (2 servos + PID profundidad + puente H)
}
BAUDRATE = 9600

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

# ===================== ESTADO GLOBAL =====================
state = {
    # Motores duales
    'motores': {'m1': 0, 'm2': 0, 'ts': None},

    # Servos de dirección (en 4º Arduino)
    'servos': {'servo1': 180, 'servo2': 0, 'preset': None, 'ts': None},

    # Profundidad con PID (en 4º Arduino)
    'depth': {
        'objetivo_cm': 0.0,
        'profundidad_cm': 0.0,
        'error_cm': 0.0,
        'u': 0.0,
        'presion_raw': 0.0,
        'estado_bombas': 'neutro',
        'ts': None
    },

    # Flags de módulos
    'deteccion_activa': True,
    'radar_active': False,
    'sensores_active': False,

    # Otros
    'last_detected': [],
    'fps_counter': 0,
    'last_fps_time': time.time(),
    'frame_counter': 0
}

# Datos de sensores
sensor_data = {"temp": None, "ph": None, "turb": None, "tds": None}

# Radar
radar_data = {"angulo": 90, "distancia": 0, "status": "disconnected"}

# Locks
state_lock = threading.Lock()

# ===================== INICIALIZACIÓN DE HARDWARE =====================
def init_camera():
    """Inicializa la cámara Picamera2."""
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (320, 240)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.configure("preview")
    picam2.start()
    time.sleep(1)  # Warm-up
    return picam2

def init_model():
    """Carga el modelo de detección de objetos."""
    try:
        net = cv2.dnn.readNetFromCaffe(MODEL_CONFIG['prototxt'], MODEL_CONFIG['model'])
        print("[INFO] Modelo cargado exitosamente")
        return net
    except Exception as e:
        print(f"[ERROR] No se pudo cargar el modelo: {e}")
        return None

def init_serial(port_name):
    """Inicializa conexión serial con un dispositivo (REAL, sin Dummy)."""
    try:
        ser = serial.Serial(SERIAL_PORTS[port_name], BAUDRATE, timeout=0.1)
        time.sleep(2)  # tiempo de estabilización
        print(f"[INFO] {port_name} conectado en {SERIAL_PORTS[port_name]} @ {BAUDRATE}")
        return ser
    except Exception as e:
        print(f"[ERROR] No se pudo conectar {port_name}: {e}")
        return None

# ===================== HILOS DE LECTURA SERIAL =====================
def read_sensor_data(ser):
    """Hilo para leer datos de sensores (Arduino Sensores)."""
    pattern = re.compile(
        r"TEMP:(?P<temp>\d+(?:\.\d+)?),PH:(?P<ph>\d+(?:\.\d+)?),TURB:(?P<turb>\d+(?:\.\d+)?),TDS:(?P<tds>\d+(?:\.\d+)?)"
    )
    while state['sensores_active'] and ser and ser.is_open:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            match = pattern.match(line)
            if match:
                sensor_data.update({
                    "temp": float(match.group("temp")),
                    "ph": float(match.group("ph")),
                    "turb": float(match.group("turb")),
                    "tds": float(match.group("tds"))
                })
        except Exception as e:
            print(f"[ERROR] Lectura serial sensores: {e}")
            time.sleep(0.05)

def read_radar_data(ser):
    """Hilo para leer datos del radar (Arduino Radar)."""
    while ser and ser.is_open:
        if not state['radar_active']:
            time.sleep(0.05)
            continue
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            try:
                angle, distance = map(float, line.split(','))
                if 0 <= angle <= 180 and 0 <= distance <= 400:
                    radar_data.update({"angulo": angle, "distancia": distance, "status": "receiving"})
            except ValueError:
                continue
        except Exception as e:
            print(f"[ERROR] Lectura radar: {e}")
            time.sleep(0.05)

def read_servos_depth(ser):
    """
    Hilo de telemetría para Arduino Servos+Profundidad.
    Formatos esperados:
      D,<prof_cm>,<obj_cm>,<err_cm>,<u>,<pres_raw>,<estado>
      A,<servo1>,<servo2>,<preset>
    """
    while ser and ser.is_open:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if line.startswith("D,"):
                try:
                    _, prof, obj, err, u, pres, est = line.split(",", 6)
                    with state_lock:
                        state['depth']['profundidad_cm'] = float(prof)
                        state['depth']['objetivo_cm']    = float(obj)
                        state['depth']['error_cm']       = float(err)
                        state['depth']['u']              = float(u)
                        state['depth']['presion_raw']    = float(pres)
                        state['depth']['estado_bombas']  = est
                        state['depth']['ts']             = time.time()
                except Exception:
                    pass

            elif line.startswith("A,"):
                try:
                    _, a1, a2, preset = line.split(",", 3)
                    with state_lock:
                        state['servos']['servo1'] = int(float(a1))
                        state['servos']['servo2'] = int(float(a2))
                        state['servos']['preset'] = None if preset in ("", "none", "null") else preset
                        state['servos']['ts']     = time.time()
                except Exception:
                    pass

        except Exception as e:
            print(f"[ERROR] Lectura servos/depth: {e}")
            time.sleep(0.05)

# ===================== VISIÓN POR COMPUTADORA =====================
def detect_objects(frame, net):
    """Detecta objetos en un frame de video."""
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
                        endX, endY = min(w - 1, endX), min(h - 1, endY)

                        color = (0, 255, 0)
                        if label == "can":
                            color = (0, 165, 255)
                        elif label == "cup":
                            color = (255, 0, 0)

                        cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)
                        y = startY - 15 if startY - 15 > 15 else startY + 15
                        cv2.putText(frame, f"{label}: {confidence:.2f}",
                                    (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

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
    """Generador de frames para el streaming."""
    while True:
        try:
            frame = picam2.capture_array()

            # FPS
            state['fps_counter'] += 1
            if time.time() - state['last_fps_time'] >= 1.0:
                state['fps_counter'] = 0
                state['last_fps_time'] = time.time()

            # Detección
            if state['deteccion_activa']:
                frame, _ = detect_objects(frame, net)
            else:
                state['last_detected'] = []

            # Codificar
            ok, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                continue
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        except Exception as e:
            print(f"[ERROR] Captura de frame: {e}")
            continue

# ===================== UTILIDADES =====================
def evaluar_calidad(valor, rango_bueno, rango_medio):
    """Evalúa la calidad de un parámetro según rangos."""
    if valor is None:
        return "red"
    if rango_bueno[0] <= valor <= rango_bueno[1]:
        return "green"
    elif rango_medio[0] <= valor <= rango_medio[1]:
        return "yellow"
    else:
        return "red"

def clamp(x, lo, hi):
    try:
        x = float(x)
    except Exception:
        x = lo
    return max(lo, min(hi, x))

# ===================== RUTAS FLASK =====================
@app.route('/')
def index():
    """Ruta principal (Jinja debe tener index.html)."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Streaming de video en tiempo real."""
    return Response(generate_frames(picam2, net), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/sensor_data')
def get_sensors():
    """Datos de sensores con indicadores de calidad."""
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

@app.route('/radar_data')
def get_radar():
    """Devuelve datos del radar."""
    return jsonify(radar_data)

@app.route('/start_radar', methods=['POST'])
def start_radar():
    if arduino_radar and arduino_radar.is_open:
        try:
            arduino_radar.write(b'START\n')
            with state_lock:
                state['radar_active'] = True
            if not getattr(app, "_radar_thread", None) or not app._radar_thread.is_alive():
                app._radar_thread = threading.Thread(target=read_radar_data, args=(arduino_radar,), daemon=True)
                app._radar_thread.start()
            radar_data["status"] = "scanning"
            return jsonify(success=True, status=radar_data["status"])
        except Exception as e:
            return jsonify(success=False, error=str(e))
    return jsonify(success=False, status="disconnected")


@app.route('/stop_radar', methods=['POST'])
def stop_radar():
    """Detiene el escaneo del radar."""
    if arduino_radar and arduino_radar.is_open:
        try:
            arduino_radar.write(b'STOP\n')
            radar_data["status"] = "idle"
            state['radar_active'] = False
            return jsonify(success=True, status=radar_data["status"])
        except Exception as e:
            return jsonify(success=False, error=str(e))
    return jsonify(success=False, status="disconnected")

@app.route('/toggle_detection', methods=['POST'])
def toggle_detection():
    """Activa/desactiva la detección de objetos."""
    state['deteccion_activa'] = not state['deteccion_activa']
    return jsonify({"detection": state['deteccion_activa']})

# ---------- MOTORES DUALES (Arduino Motores) ----------
@app.route('/update_motors_dual', methods=['POST'])
def update_motors_dual():
    """
    Body JSON: { "velocidad1": 0-100, "velocidad2": 0-100 }
    Envía: M,<v1>,<v2>\n
    """
    if not (arduino_motores and arduino_motores.is_open):
        return jsonify(ok=False, error="Arduino Motores no conectado"), 500

    data = request.get_json(force=True, silent=True) or {}
    v1 = int(clamp(data.get("velocidad1", 0), 0, 100))
    v2 = int(clamp(data.get("velocidad2", 0), 0, 100))
    try:
        arduino_motores.write(f"M,{v1},{v2}\n".encode())
        with state_lock:
            state['motores']['m1'] = v1
            state['motores']['m2'] = v2
            state['motores']['ts'] = time.time()
        return jsonify(ok=True, m1=v1, m2=v2)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


# ---------- NUEVAS RUTAS: SERVOS (Arduino Servos+Profundidad) ----------
@app.route('/set_servos', methods=['POST'])
def set_servos():
    """
    Body JSON: { "servo1": 0-180, "servo2": 0-180 }
    Envía: S,<a1>,<a2>\n
    """
    if not (arduino_servos_depth and arduino_servos_depth.is_open):
        return jsonify(ok=False, error="Arduino Servos+Depth no conectado"), 500

    data = request.get_json(force=True, silent=True) or {}
    a1 = int(clamp(data.get("servo1", 90), 0, 180))
    a2 = int(clamp(data.get("servo2", 90), 0, 180))
    try:
        arduino_servos_depth.write(f"S,{a1},{a2}\n".encode())
        with state_lock:
            state['servos']['servo1'] = a1
            state['servos']['servo2'] = a2
            state['servos']['preset'] = None
            state['servos']['ts'] = time.time()
        return jsonify(ok=True, servo1=a1, servo2=a2)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

@app.route('/control_servo', methods=['POST'])
def control_servo():
    """
    Body JSON: { "cmd": "frw"|"up"|"bkw"|"izq"|"der" }
    Envía: C,<cmd>\n
    """
    if not (arduino_servos_depth and arduino_servos_depth.is_open):
        return jsonify(ok=False, error="Arduino Servos+Depth no conectado"), 500

    data = request.get_json(force=True, silent=True) or {}
    cmd = str(data.get("cmd", "")).lower()
    if cmd not in {"frw", "up", "bkw", "izq", "der"}:
        return jsonify(ok=False, error="cmd inválido"), 400

    try:
        arduino_servos_depth.write(f"C,{cmd}\n".encode())
        with state_lock:
            state['servos']['preset'] = cmd
            state['servos']['ts'] = time.time()
        return jsonify(ok=True, cmd=cmd)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500



# ---------- NUEVAS RUTAS: PROFUNDIDAD PID (Arduino Servos+Profundidad) ----------
@app.route('/set_depth', methods=['POST'])
def set_depth():
    """
    Body JSON: { "objetivo_cm": <num> }
    Envía: P,<objetivo_cm>\n
    Rango por defecto 0–200 cm 
    """
    if not (arduino_servos_depth and arduino_servos_depth.is_open):
        return jsonify(ok=False, error="Arduino Servos+Depth no conectado"), 500

    data = request.get_json(force=True, silent=True) or {}
    objetivo = clamp(data.get("objetivo_cm", 0), 0, 200)
    try:
        arduino_servos_depth.write(f"P,{objetivo}\n".encode())
        with state_lock:
            state['depth']['objetivo_cm'] = float(objetivo)
            state['depth']['ts'] = time.time()
        return jsonify(ok=True, objetivo_cm=float(objetivo))
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

@app.route('/depth_status')
def depth_status():
    with state_lock:
        d = state['depth'].copy()
    return jsonify(d)

@app.route('/pid_tune', methods=['POST'])
def pid_tune():
    """
    Body JSON: { "kp": <num>, "ki": <num>, "kd": <num> }
    Envía: T,<kp>,<ki>,<kd>\n
    """
    if not (arduino_servos_depth and arduino_servos_depth.is_open):
        return jsonify(ok=False, error="Arduino Servos+Depth no conectado"), 500

    data = request.get_json(force=True, silent=True) or {}
    try:
        kp = float(data.get("kp"))
        ki = float(data.get("ki"))
        kd = float(data.get("kd"))
    except Exception:
        return jsonify(ok=False, error="kp/ki/kd inválidos"), 400

    try:
        arduino_servos_depth.write(f"T,{kp},{ki},{kd}\n".encode())
        return jsonify(ok=True, kp=kp, ki=ki, kd=kd)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


        # --- Zero @ surface (capturar referencia) ---
@app.post('/depth_zero')
def depth_zero():
    """
    Envía: Z\n
    El Arduino debe promediar la lectura, fijar offset (P0), resetear integral y dejar SP=0.
    """
    if not (arduino_servos_depth and arduino_servos_depth.is_open):
        return jsonify(ok=False, error="Arduino Servos+Depth no conectado"), 500
    try:
        arduino_servos_depth.write(b"Z\n")
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


# ---------- Reconexión de TODOS los seriales ----------
@app.route("/reconectar_todos", methods=["POST"])
def reconnect():
    """Reinicia todas las conexiones seriales y relanza los hilos necesarios."""
    global arduino_motores, arduino_sensores, arduino_radar, arduino_servos_depth

    try:
        # 1) Cerrar
        for ser in [arduino_motores, arduino_sensores, arduino_radar, arduino_servos_depth]:
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception as e:
                print(f"[WARN] Error cerrando puerto: {e}")

        time.sleep(0.3)

        # 2) Reabrir
        arduino_motores      = init_serial('motores')
        arduino_sensores     = init_serial('sensores')
        arduino_radar        = init_serial('radar')
        arduino_servos_depth = init_serial('servos_depth')

        # 3) Reiniciar hilos
        state['sensores_active'] = bool(arduino_sensores and arduino_sensores.is_open)
        state['radar_active']    = bool(arduino_radar and arduino_radar.is_open)

        if state['sensores_active']:
            threading.Thread(target=read_sensor_data, args=(arduino_sensores,), daemon=True).start()

        if state['radar_active']:
            threading.Thread(target=read_radar_data, args=(arduino_radar,), daemon=True).start()

        if arduino_servos_depth and arduino_servos_depth.is_open:
            threading.Thread(target=read_servos_depth, args=(arduino_servos_depth,), daemon=True).start()

        return jsonify({
            "status": "ok",
            "msg": "Reconexión exitosa",
            "motores": bool(arduino_motores and arduino_motores.is_open),
            "sensores": bool(arduino_sensores and arduino_sensores.is_open),
            "radar": bool(arduino_radar and arduino_radar.is_open),
            "servos_depth": bool(arduino_servos_depth and arduino_servos_depth.is_open)
        })

    except Exception as e:
        print(f"[ERROR] Reconexión fallida: {e}")
        return jsonify({"status": "error", "msg": str(e)}), 500

# ===================== INICIALIZACIÓN Y EJECUCIÓN =====================
if __name__ == '__main__':
    # Inicialización de cámara y modelo
    picam2 = init_camera()
    net = init_model()

    # Conexiones seriales
    arduino_motores      = init_serial('motores')
    arduino_sensores     = init_serial('sensores')
    arduino_radar        = init_serial('radar')
    arduino_servos_depth = init_serial('servos_depth')

    # Hilos para sensores, radar y servos+profundidad (telemetría)
    if arduino_sensores and arduino_sensores.is_open:
        state['sensores_active'] = True
        threading.Thread(target=read_sensor_data, args=(arduino_sensores,), daemon=True).start()

    if arduino_radar and arduino_radar.is_open:
        state['radar_active'] = True
        threading.Thread(target=read_radar_data, args=(arduino_radar,), daemon=True).start()

    if arduino_servos_depth and arduino_servos_depth.is_open:
        threading.Thread(target=read_servos_depth, args=(arduino_servos_depth,), daemon=True).start()

    # Mostrar IP en LCD
    try:
        ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
        mostrar_ip_puerto(ip, "5000")
    except Exception:
        print("No se pudo obtener la IP de wlan0")

    # Ejecutar app
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        # Limpieza al cerrar
        state.update({'sensores_active': False, 'radar_active': False})
        try:
            picam2.stop()
        except Exception:
            pass
        for ser in [arduino_motores, arduino_sensores, arduino_radar, arduino_servos_depth]:
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass

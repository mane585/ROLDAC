
#include <Servo.h>

// --- Pines (como pediste)
const uint8_t SERVO_PIN = 11;
const uint8_t TRIG_PIN  = 9;
const uint8_t ECHO_PIN  = 10;

// --- Barrido
const uint8_t STEP_DEG     = 5;    // paso angular
const uint16_t STEP_MS     = 50;   // tiempo entre pasos (~20 Hz/servo)
const float MAX_CM_REPORT  = 400;  // tope que acepta tu backend

Servo radarServo;
bool scanning = false;
int angle = 0;
int step  = STEP_DEG;
unsigned long lastStepMs = 0;

// --- Utilidades
float measureOnceCM() {
  // Pulso de disparo
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout 30 ms (~5 m); devuelve 0 si no hay eco
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (us == 0) return 0.0f;         // sin eco
  float cm = us * 0.0343f / 2.0f;   // 343 m/s → 0.0343 cm/µs
  if (cm < 0) cm = 0;
  if (cm > MAX_CM_REPORT) cm = MAX_CM_REPORT;
  return cm;
}

float measureMedian3CM() {
  float a = measureOnceCM();
  float b = measureOnceCM();
  float c = measureOnceCM();
  // mediana “manual” para 3 valores
  if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
  if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
  return c;
}

void startScanning() {
  scanning = true;
  angle = 0;
  step = STEP_DEG;
  lastStepMs = 0;
  Serial.println("RADAR_STARTED");
}

void stopScanning() {
  scanning = false;
  radarServo.write(0);   // aparca en 0°
  Serial.println("RADAR_STOPPED");
}

void setup() {
  Serial.begin(9600);     
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  radarServo.attach(SERVO_PIN);
  radarServo.write(0);
  delay(500);

  Serial.println("RADAR_READY");
}

void loop() {
  // --- Procesar comandos en todo momento
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "START") {
      startScanning();
    } else if (cmd == "STOP") {
      stopScanning();
    }
  }

  // --- Barrido no bloqueante
  if (scanning) {
    unsigned long now = millis();
    if (now - lastStepMs >= STEP_MS) {
      lastStepMs = now;

      // Mover servo y medir
      radarServo.write(angle);
      // breve asentamiento mecánico (opcional, puedes bajarlo a 20 ms)
      delay(20);
      float cm = measureMedian3CM();

      // Emitir en el formato que parsea tu backend: "angulo,distancia\n"
      Serial.print(angle);
      Serial.print(",");
      Serial.println(cm, 1);   // 1 decimal está bien

      // Rebotar en 0–180°
      angle += step;
      if (angle >= 180 || angle <= 0) {
        step = -step;
        // clamp por si se pasa por redondeos
        if (angle > 180) angle = 180;
        if (angle < 0)   angle = 0;
      }
    }
  }
}

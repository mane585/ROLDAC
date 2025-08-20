#include <Servo.h>

// Pines para el L298N
const int ENA = 5;   // PWM motor A
const int IN1 = 6;   // Dirección motor A
const int IN2 = 7;
const int IN3 = 8;   // Dirección motor B
const int IN4 = 9;
const int ENB = 10;  // PWM motor B

// Pines para servos y control
const int pinServo1 = 3;
const int pinServo2 = 11;
const int pinControl1 = A0;
const int pinControl2 = A1;
const int pinControl3 = A2;

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);
  
  // Configurar pines del L298N
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Configurar pines de control con pull-up interna
  pinMode(pinControl1, INPUT_PULLUP);
  pinMode(pinControl2, INPUT_PULLUP);
  pinMode(pinControl3, INPUT_PULLUP);
  
  // Configurar servos
  servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  
  // Posición inicial
  servo1.write(180);  // Posición neutra
  servo2.write(0);
  
  // Detener motores al inicio
  detenerMotores();
  Serial.println("Sistema inicializado");
}

void loop() {
  // Leer pines de control (lógica invertida por pull-up)
  bool c1 = !digitalRead(pinControl1);
  bool c2 = !digitalRead(pinControl2);
  bool c3 = !digitalRead(pinControl3);

  // Control de servos
  if (!c3 && !c2 && c1) {      // UP
    servo1.write(90);
    servo2.write(90);
  } 
  else if (!c3 && !c2 && !c1) { // FRW (Adelante)
    servo1.write(180);
    servo2.write(0);
  } 
  else if (!c3 && c2 && c1) {   // BKW (Atrás)
    servo1.write(0);
    servo2.write(180);
  } 
  else if (!c3 && c2 && !c1) {  // IZQ (Izquierda)
    servo1.write(180);
    servo2.write(180);
  } 
  else if (c3 && !c2 && c1) {   // DER (Derecha)
    servo1.write(0);
    servo2.write(0);
  }

  // Control de motores
  if (c3 && !c2 && !c1) {       // SUBIR
    moverArriba();
  } 
  else if (c3 && c2 && c1) {    // BAJAR
    moverAbajo();
  } 
  else if (c3 && c2 && !c1) {   // NEUTRO
    detenerMotores();
  }

  delay(10); // Pequeña pausa para estabilidad
}

// Funciones de movimiento
void moverArriba() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void moverAbajo() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void detenerMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
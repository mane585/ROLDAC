/*#include <Servo.h>

Servo escIzq;
Servo escDer;
Servo servoIzq;
Servo servoDer;

const int pinEscIzq = 9;
const int pinEscDer = 10;
const int pinServoIzq = 8;   // Ajusta estos pines según tu conexión
const int pinServoDer = 5;

int velocidad = 0;
int direccion = 50;

int posicionServo = 0;  // Default 0°

void setup() {
  Serial.begin(9600);
  escIzq.attach(pinEscIzq);
  escDer.attach(pinEscDer);

  servoIzq.attach(pinServoIzq);
  servoDer.attach(pinServoDer);

  escIzq.writeMicroseconds(1000);
  escDer.writeMicroseconds(1000);

  servoIzq.write(posicionServo);
  servoDer.write(posicionServo);

  Serial.println("Arduino ESC motores y servos listo");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      if (line.startsWith("ORIENT:")) {
        int pos = line.substring(7).toInt();
        if (pos == 0 || pos == 90 || pos == 180) {
          posicionServo = pos;
          servoIzq.write(pos);
          servoDer.write(pos);
          Serial.print("Servo orientacion a ");
          Serial.println(pos);
        }
      } else {
        int comaIndex = line.indexOf(',');
        if (comaIndex > 0) {
          int vel = line.substring(0, comaIndex).toInt();
          int dir = line.substring(comaIndex + 1).toInt();

          if (vel >= 0 && vel <= 100 && dir >= 0 && dir <= 100) {
            velocidad = vel;
            direccion = dir;
            controlarMotores(velocidad, direccion);
          }
        }
      }
    }
  }
}

void controlarMotores(int vel, int dir) {
  if (vel < 10) {
    escIzq.writeMicroseconds(1000);
    escDer.writeMicroseconds(1000);
    Serial.println("Motores detenidos por baja velocidad");
    return;
  }

  // Mapeo individual: motor izquierdo desde 1250, derecho desde 1000
  int pwmBaseIzq = map(vel, 10, 100, 1250, 2000);
  int pwmBaseDer = map(vel, 10, 100, 1000, 2000);

  float factorIzq = 1.0;
  float factorDer = 1.0;

  if (dir < 50) {
    factorIzq = (float)dir / 50.0;
  } else if (dir > 50) {
    factorDer = (float)(100 - dir) / 50.0;
  }

  int pwmIzq = constrain(pwmBaseIzq * factorIzq, 1000, 2000);
  int pwmDer = constrain(pwmBaseDer * factorDer, 1000, 2000);

  escIzq.writeMicroseconds(pwmIzq);
  escDer.writeMicroseconds(pwmDer);

  Serial.print("PWM Izq: ");
  Serial.print(pwmIzq);
  Serial.print("  PWM Der: ");
  Serial.println(pwmDer);
}
*/

#include <Servo.h>

Servo escIzq;
Servo escDer;

const int pinEscIzq = 9;
const int pinEscDer = 10;

int velocidad = 0;
int direccion = 50;

void setup() {
  Serial.begin(9600);
  escIzq.attach(pinEscIzq);
  escDer.attach(pinEscDer);

  escIzq.writeMicroseconds(1000);
  escDer.writeMicroseconds(1000);

  Serial.println("Arduino ESC motores listo");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      int comaIndex = line.indexOf(',');
      if (comaIndex > 0) {
        int vel = line.substring(0, comaIndex).toInt();
        int dir = line.substring(comaIndex + 1).toInt();

        if (vel >= 0 && vel <= 100 && dir >= 0 && dir <= 100) {
          velocidad = vel;
          direccion = dir;
          controlarMotores(velocidad, direccion);
        }
      }
    }
  }
}



void controlarMotores(int vel, int dir) {
  if (vel < 10) {
    escIzq.writeMicroseconds(1000);
    escDer.writeMicroseconds(1000);
    Serial.println("Motores detenidos por baja velocidad");
    return;
  }

  // Mapeo individual: motor izquierdo desde 1250, derecho desde 1000
  int pwmBaseIzq = map(vel, 10, 100, 1250, 2000);
  int pwmBaseDer = map(vel, 10, 100, 1000, 2000);

  float factorIzq = 1.0;
  float factorDer = 1.0;

  if (dir < 50) {
    factorIzq = (float)dir / 50.0;
  } else if (dir > 50) {
    factorDer = (float)(100 - dir) / 50.0;
  }

  int pwmIzq = constrain(pwmBaseIzq * factorIzq, 1000, 2000);
  int pwmDer = constrain(pwmBaseDer * factorDer, 1000, 2000);

  escIzq.writeMicroseconds(pwmIzq);
  escDer.writeMicroseconds(pwmDer);

  Serial.print("PWM Izq: ");
  Serial.print(pwmIzq);
  Serial.print("  PWM Der: ");
  Serial.println(pwmDer);
}

/*
#include <ESP32Servo.h>

Servo escIzq;   // ESC motor izquierdo
Servo escDer;   // ESC motor derecho

// Pines para ESP32 (puedes cambiarlos según tu configuración)
const int pinEscIzq = 16;  // Pin PWM para ESC izquierdo (GPIO16)
const int pinEscDer = 17;  // Pin PWM para ESC derecho (GPIO17)

int velocidad = 0;  // 0-100
int direccion = 50; // 0-100 (50 es centro)

void setup() {
  Serial.begin(115200);  // ESP32 usa comúnmente 115200 baudios
  
  // Permitir asignación de todos los timers ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Configurar ESCs
  escIzq.setPeriodHertz(50);  // Frecuencia estándar para ESCs (50Hz)
  escDer.setPeriodHertz(50);
  
  escIzq.attach(pinEscIzq, 1000, 2000);  // Asignar pin con rango de pulsos (1000-2000μs)
  escDer.attach(pinEscDer, 1000, 2000);

  // Inicializar ESCs en señal de 0% velocidad (usualmente 1000us pulso)
  escIzq.writeMicroseconds(1000);
  escDer.writeMicroseconds(1000);

  Serial.println("ESP32 ESC motores listo");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      int comaIndex = line.indexOf(',');
      if (comaIndex > 0) {
        String velStr = line.substring(0, comaIndex);
        String dirStr = line.substring(comaIndex + 1);
        int vel = velStr.toInt();
        int dir = dirStr.toInt();

        // Validar rango
        if (vel >= 0 && vel <= 100 && dir >= 0 && dir <= 100) {
          velocidad = vel;
          direccion = dir;
          controlarMotores(velocidad, direccion);

          // Enviar confirmación
          Serial.print("Recibido Velocidad=");
          Serial.print(velocidad);
          Serial.print(" Dirección=");
          Serial.println(direccion);
        }
      }
    }
  }
}

// Función para controlar ESCs según velocidad y dirección
void controlarMotores(int vel, int dir) {
  // Convertir velocidad y dirección 0-100 a valores de PWM para ESC (1000us-2000us)
  // Se supone que 1000us = stop, 2000us = máxima velocidad

  // Calcular velocidades de cada motor según dirección:
  // Dirección 50 = ambos igual velocidad
  // Dirección < 50 = motor izquierdo más lento (gira a la izquierda)
  // Dirección > 50 = motor derecho más lento (gira a la derecha)

  float velNorm = map(vel, 0, 100, 1000, 2000);  // escala para ESC

  float factorIzq = 1.0;
  float factorDer = 1.0;

  if (dir < 50) {
    // Gira a la izquierda: motor izquierdo reduce velocidad
    factorIzq = (float)dir / 50.0;   // de 0 a 1 cuando dir va de 0 a 50
    factorDer = 1.0;
  }
  else if (dir > 50) {
    // Gira a la derecha: motor derecho reduce velocidad
    factorIzq = 1.0;
    factorDer = (float)(100 - dir) / 50.0; // de 1 a 0 cuando dir va de 50 a 100
  }
  else {
    factorIzq = 1.0;
    factorDer = 1.0;
  }

  int pwmIzq = constrain((int)(velNorm * factorIzq), 1100, 2000);
  int pwmDer = constrain((int)(velNorm * factorDer), 1000, 2000);

  escIzq.writeMicroseconds(pwmIzq);
  escDer.writeMicroseconds(pwmDer);

  Serial.print("PWM Izq: ");
  Serial.print(pwmIzq);
  Serial.print("  PWM Der: ");
  Serial.println(pwmDer);
}
*/
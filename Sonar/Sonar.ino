#include <Servo.h>

Servo radarServo;
const int trigPin = 9;
const int echoPin = 10;

void setup() {
  Serial.begin(9600);
  radarServo.attach(11);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  radarServo.write(0); // Inicia en 0°
  delay(1000);
  Serial.println("RADAR_READY");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "START") {
      Serial.println("RADAR_STARTED");

      int angle = 0;
      int step = 5;
      bool scanning = true;

      while (scanning) {
        // Mover servo
        radarServo.write(angle);

        // Medición ultrasónica
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duration = pulseIn(echoPin, HIGH, 30000); // timeout opcional
        float distance = duration * 0.034 / 2;

        Serial.print(angle);
        Serial.print(",");
        Serial.println(distance);
        delay(50);

        // Oscilar entre 0° y 180°
        angle += step;
        if (angle >= 180 || angle <= 0) {
          step = -step;
        }

        // Verificar si llegó comando STOP
        if (Serial.available() > 0) {
          String stopCmd = Serial.readStringUntil('\n');
          stopCmd.trim();
          if (stopCmd == "STOP") {
            radarServo.write(0);
            Serial.println("RADAR_STOPPED");
            delay(500);
            scanning = false;
          }
        }
      }
    }
  }
}

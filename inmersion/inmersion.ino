#include <Wire.h>
#include <Servo.h>
#include "MS5837.h"
#include <math.h>

// ======== Config ========
const long BAUD = 9600;

// Servos (opcionales)
const int PIN_S1 = 12;
const int PIN_S2 = 11;
const int S1_DEF = 180;
const int S2_DEF = 0;

// Electroválvulas (NC) y Bomba
// Llenado: S1+S3  |  Vaciado: S2+S4
const int PIN_V_S1_FILL  = 2;
const int PIN_V_S3_FILL  = 4;
const int PIN_V_S2_DRAIN = 7;
const int PIN_V_S4_DRAIN = 8;
const int PIN_PUMP_PWM   = 5;   // pin PWM

// Sensor BlueRobotics MS5837-02BA
MS5837 sensor;
const float FLUID_DENSITY = 997.0f;      // agua dulce
const float K_MBAR_TO_CM  = 1.019716f;   // 1 mbar ≈ 1.0197 cm H2O

// Rango y tiempos
const float MAX_DEPTH_CM = 200.0f;
const unsigned long LOOP_MS = 100;       // ~10 Hz
const unsigned long TELE_MS = 250;       // ~4 Hz
const unsigned long SWITCH_HOLD_MS = 120; // pausa mínima al cambiar de sentido

// PID básico
float Kp=2.0f, Ki=0.08f, Kd=0.80f;
float sp_cm=0, pv_cm=0, err_cm=0, integ=0, prev_pv=0, u=0;
const float I_CLAMP=800.0f;
const float DEADBAND_CM=1.0f;

// Zero relativo (superficie)
bool  zero_ok=false;
float P0_mbar=0;

// Filtro EMA
float ema_alpha=0.20f;

// Bomba (map directo de |u|)
const int PUMP_MIN_START=60;   // umbral arranque
const int PUMP_MAX=255;

// Sentido actual: -1 vaciar, 0 hold, +1 llenar
int path=0;
unsigned long switchHoldUntil=0;

// Servos (opc)
Servo s1, s2;
int lastA1=S1_DEF, lastA2=S2_DEF;
String lastPreset="none";

// Tiempos
unsigned long tLoop=0, tTele=0;

// ======== Utils ========
inline float clampf(float x, float lo, float hi){
  if(x<lo) return lo; if(x>hi) return hi; return x;
}
inline int signf(float x){ return (x>0)-(x<0); }

void valvesHold(){
  digitalWrite(PIN_V_S1_FILL, LOW);
  digitalWrite(PIN_V_S3_FILL, LOW);
  digitalWrite(PIN_V_S2_DRAIN, LOW);
  digitalWrite(PIN_V_S4_DRAIN, LOW);
}
void valvesFill(){
  digitalWrite(PIN_V_S2_DRAIN, LOW);
  digitalWrite(PIN_V_S4_DRAIN, LOW);
  digitalWrite(PIN_V_S1_FILL, HIGH);
  digitalWrite(PIN_V_S3_FILL, HIGH);
}
void valvesDrain(){
  digitalWrite(PIN_V_S1_FILL, LOW);
  digitalWrite(PIN_V_S3_FILL, LOW);
  digitalWrite(PIN_V_S2_DRAIN, HIGH);
  digitalWrite(PIN_V_S4_DRAIN, HIGH);
}
inline const char* estadoStr(){
  if(path>0) return "llenando";
  if(path<0) return "vaciando";
  return "hold";
}

void readPressure_mbar(float &Pm){
  sensor.read();         // no devuelve bool
  Pm = sensor.pressure(); // mbar absolutos
}
float readDepthCm(){
  float Pm=0; readPressure_mbar(Pm);
  if(!zero_ok){ return 0.0f; }
  float dP = Pm - P0_mbar; if(dP<0) dP=0;
  float raw_cm = dP * K_MBAR_TO_CM;
  pv_cm = ema_alpha*raw_cm + (1.0f-ema_alpha)*pv_cm;
  pv_cm = clampf(pv_cm, 0.0f, MAX_DEPTH_CM*1.2f);
  return pv_cm;
}

void zeroSurface(){
  // Promedia ~1 s
  const int N=40; float acc=0;
  for(int i=0;i<N;i++){ float Pm; readPressure_mbar(Pm); acc+=Pm; delay(25); }
  P0_mbar = acc/N; zero_ok=true;

  // Reset control
  sp_cm=pv_cm=prev_pv=0; integ=0; u=0;
  analogWrite(PIN_PUMP_PWM, 0); valvesHold(); path=0;
}

void pidStep(float dt){
  err_cm = sp_cm - pv_cm;

  // derivada sobre medida (más estable en cambios de SP)
  float dmeas = (pv_cm - prev_pv)/dt; prev_pv = pv_cm;

  integ += err_cm*dt; integ = clampf(integ, -I_CLAMP, I_CLAMP);

  float u_unsat = Kp*err_cm + Ki*integ - Kd*dmeas;
  u = clampf(u_unsat, -100.0f, 100.0f);
}

int uToPwm(float mag){
  if(mag<=1.0f) return 0;
  float frac = mag/100.0f;
  int pwm = (int)(PUMP_MIN_START + frac*(PUMP_MAX - PUMP_MIN_START));
  return pwm>PUMP_MAX?PUMP_MAX:(pwm<0?0:pwm);
}

void applyActuators(){
  // Banda muerta → hold
  if(fabsf(err_cm)<=DEADBAND_CM){
    analogWrite(PIN_PUMP_PWM, 0);
    valvesHold(); path=0;
    return;
  }

  const int want = signf(u); // +1 llenar, -1 vaciar

  // Cambio de sentido seguro y simple:
  // 1) si cambia el signo, parar y cerrar todo, esperar SWITCH_HOLD_MS
  if(want!=path){
    analogWrite(PIN_PUMP_PWM, 0);
    valvesHold();
    path=0;
    switchHoldUntil = millis()+SWITCH_HOLD_MS;
    return; // aplica tras la pausa en el siguiente ciclo
  }

  // 2) tras la pausa, abrir vía y dar PWM
  if(millis() < switchHoldUntil){
    analogWrite(PIN_PUMP_PWM, 0);
    return;
  }

  if(want>0){ valvesFill();  path=+1; }
  else      { valvesDrain(); path=-1; }

  analogWrite(PIN_PUMP_PWM, uToPwm(fabsf(u)));
}

void sendDepthTelemetry(float Pmbar){
  Serial.print("D,");
  Serial.print(pv_cm,1); Serial.print(",");
  Serial.print(sp_cm,0); Serial.print(",");
  Serial.print(err_cm,1); Serial.print(",");
  Serial.print(u,1);      Serial.print(",");
  Serial.print(Pmbar,1);  Serial.print(",");
  Serial.println(estadoStr());
}

void sendServosTelemetry(){
  Serial.print("A,"); Serial.print(lastA1); Serial.print(",");
  Serial.print(lastA2); Serial.print(",");
  Serial.println(lastPreset);
}

// ======== Setup ========
void setup(){
  Serial.begin(BAUD);
  Serial.setTimeout(20);

  // Servos (opcionales, dejan compat con tu UI)
  s1.attach(PIN_S1); s2.attach(PIN_S2);
  s1.write(S1_DEF);  s2.write(S2_DEF);

  pinMode(PIN_V_S1_FILL, OUTPUT);
  pinMode(PIN_V_S3_FILL, OUTPUT);
  pinMode(PIN_V_S2_DRAIN, OUTPUT);
  pinMode(PIN_V_S4_DRAIN, OUTPUT);
  pinMode(PIN_PUMP_PWM, OUTPUT);
  valvesHold(); analogWrite(PIN_PUMP_PWM, 0);

  Wire.begin();
  if(!sensor.init()){ /* si falla, seguiremos reportando 0 */ }
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(FLUID_DENSITY);

  tLoop = tTele = millis();
}

// ======== Loop ========
void loop(){
  // ---- Comandos serial ----
  if(Serial.available()){
    String line = Serial.readStringUntil('\n'); line.trim(); line.toLowerCase();

    if(line.startsWith("p,")){           // P,<cm>
      int c=line.indexOf(','); sp_cm = clampf(line.substring(c+1).toFloat(), 0, MAX_DEPTH_CM);
    }else if(line.startsWith("t,")){     // T,<kp>,<ki>,<kd>
      int c1=line.indexOf(','), c2=line.indexOf(',',c1+1);
      if(c1>0 && c2>c1){ Kp=line.substring(c1+1,c2).toFloat(); Ki=line.substring(c2+1).toFloat(); /* Kd en siguiente línea si mandas 3 valores */ }
      int c3=line.indexOf(',',c2+1);
      if(c3>c2){ Ki=line.substring(c2+1,c3).toFloat(); Kd=line.substring(c3+1).toFloat(); }
    }else if(line.startsWith("z")){      // Zero @ surface
      zeroSurface(); Serial.println("Z,OK");
    }else if(line.startsWith("s,")){     // S,<a1>,<a2> (opc)
      int c1=line.indexOf(','), c2=line.indexOf(',',c1+1);
      if(c1>0 && c2>c1){ lastA1=constrain(line.substring(c1+1,c2).toInt(),0,180);
                         lastA2=constrain(line.substring(c2+1).toInt(),0,180);
                         s1.write(lastA1); s2.write(lastA2); lastPreset="none"; sendServosTelemetry(); }
    }else if(line.startsWith("c,")){     // C,<preset> (opc)
      String cmd=line.substring(2); cmd.trim();
      if(cmd=="frw"){ lastA1=180; lastA2=0; }
      else if(cmd=="bkw"){ lastA1=0; lastA2=180; }
      else if(cmd=="der"){ lastA1=180; lastA2=180; }
      else if(cmd=="izq"){ lastA1=0; lastA2=0; }
      s1.write(lastA1); s2.write(lastA2); lastPreset=cmd; sendServosTelemetry();
    }
  }

  // ---- Control loop (10 Hz) ----
  const unsigned long now=millis();
  if(now - tLoop >= LOOP_MS){
    const float dt = (now - tLoop)/1000.0f;
    tLoop = now;

    float Pm=0; readPressure_mbar(Pm);
    readDepthCm();                 // actualiza pv_cm (EMA)

    if(!zero_ok && Pm>0){ P0_mbar=Pm; zero_ok=true; pv_cm=0; prev_pv=0; integ=0; }

    pidStep(dt);
    applyActuators();

    if(now - tTele >= TELE_MS){
      tTele = now;
      float P2=0; readPressure_mbar(P2);
      sendDepthTelemetry(P2);
    }
  }
}

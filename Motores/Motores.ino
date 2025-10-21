

#include <Servo.h>

const int pinEscIzq = 9;
const int pinEscDer = 10;

// Ventana típica de ESCs (us)
const int MIN_US = 1000;
const int MAX_US = 2000;

// Armado en el arranque: mantener mínimo durante unos segundos
const bool ARM_ON_BOOT       = true;
const unsigned long ARM_MS   = 3000;

// Failsafe: si no llegan comandos en este tiempo, cortar a MIN_US
const unsigned long FAILSAFE_MS = 30000;  // 30 s

// Suavizado (rampa)
const int RAMP_STEP_US         = 10;    // microsegundos por paso
const unsigned long RAMP_DT_MS = 10;    // periodo de rampa

// Serial
const unsigned long BAUD = 9600;

// Estado
Servo escIzq, escDer;

int targetLeftUS  = MIN_US;
int targetRightUS = MIN_US;
int currLeftUS    = MIN_US;
int currRightUS   = MIN_US;

unsigned long lastCmdMs  = 0;
unsigned long lastRampMs = 0;

// Buffer para leer líneas de Serial
static const size_t LINE_BUF_SZ = 32;
char lineBuf[LINE_BUF_SZ];
size_t lineLen = 0;

inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void writeOutputsNow() {
  escIzq.writeMicroseconds(currLeftUS);
  escDer.writeMicroseconds(currRightUS);
}

void rampOutputs() {
  const unsigned long now = millis();
  if (now - lastRampMs < RAMP_DT_MS) return;
  lastRampMs = now;

  // Izquierdo
  if (currLeftUS < targetLeftUS)
    currLeftUS = min(currLeftUS + RAMP_STEP_US, targetLeftUS);
  else if (currLeftUS > targetLeftUS)
    currLeftUS = max(currLeftUS - RAMP_STEP_US, targetLeftUS);

  // Derecho
  if (currRightUS < targetRightUS)
    currRightUS = min(currRightUS + RAMP_STEP_US, targetRightUS);
  else if (currRightUS > targetRightUS)
    currRightUS = max(currRightUS - RAMP_STEP_US, targetRightUS);

  writeOutputsNow();
}

void applyFailsafeIfStale() {
  const unsigned long now = millis();
  if (now - lastCmdMs > FAILSAFE_MS) {
    targetLeftUS  = MIN_US;
    targetRightUS = MIN_US;
  }
}

// Parsing de la línea: espera "M,<v1>,<v2>"
void handleLine(char* s) {
  // Quitar \r opcional
  size_t n = strlen(s);
  if (n && s[n-1] == '\r') s[n-1] = '\0';

  if (s[0] != 'M') {
    // formato desconocido -> ignorar
    return;
  }

  // Espera "M,<v1>,<v2>"
  char* p  = strchr(s, ',');
  if (!p) return;
  char* p2 = strchr(p + 1, ',');
  if (!p2) return;

  char* v1Str = p + 1;
  *p2 = '\0';
  char* v2Str = p2 + 1;

  int v1 = clampInt(atoi(v1Str), 0, 100);
  int v2 = clampInt(atoi(v2Str),  0, 100);

  // Mapear a microsegundos
  targetLeftUS  = map(v1, 0, 100, MIN_US, MAX_US);
  targetRightUS = map(v2, 0, 100, MIN_US, MAX_US);

  lastCmdMs = millis();

  // (Opcional) eco de depuración
  // Serial.print(F("ACK M,")); Serial.print(v1); Serial.print(','); Serial.println(v2);
}

void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      // Termina línea
      lineBuf[clampInt((int)lineLen, 0, (int)LINE_BUF_SZ-1)] = '\0';
      if (lineLen > 0) {
        handleLine(lineBuf);
      }
      lineLen = 0;  // reset buffer
    } else {
      if (lineLen < LINE_BUF_SZ - 1) {
        lineBuf[lineLen++] = c;
      } else {
        // Overflow: resetea para evitar basura
        lineLen = 0;
      }
    }
  }
}

void armESCsIfNeeded() {
  if (!ARM_ON_BOOT) return;
  // Mantener mínimo por ARM_MS
  escIzq.writeMicroseconds(MIN_US);
  escDer.writeMicroseconds(MIN_US);
  delay(ARM_MS);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(BAUD);
  // En Nano no bloquea, mantenido por compatibilidad

  escIzq.attach(pinEscIzq);  // Timer1
  escDer.attach(pinEscDer);

  currLeftUS = currRightUS = MIN_US;
  targetLeftUS = targetRightUS = MIN_US;
  writeOutputsNow();

  armESCsIfNeeded();

  lastCmdMs  = millis();
  lastRampMs = millis();

  // Indicar listo (parpadeo rápido)
  for (int i=0;i<3;i++){ digitalWrite(LED_BUILTIN, HIGH); delay(80); digitalWrite(LED_BUILTIN, LOW); delay(80); }
}

void loop() {
  readSerialLines();
  applyFailsafeIfStale();  // <- vuelve a mínimo si no llegan comandos
  rampOutputs();
}

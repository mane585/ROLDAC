 #include <OneWire.h>
#include <DallasTemperature.h>

// Pines
#define TDS_PIN A2
#define TURB_PIN A1
#define PH_PIN A0
#define TEMP_PIN 2

// Objeto DS18B20
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);

// Configuración TDS
#define TDS_SAMPLES 30
int tdsBuffer[TDS_SAMPLES];
int tdsIndex = 0;
unsigned long lastTdsTime = 0;

// Constantes de calibración (¡Ajustar!)
#define PH_NEUTRAL_VOLT 2.5    // Voltaje a pH 7.0
#define PH_SLOPE 0.18          // Pendiente del sensor
#define TURB_CLEAR_VOLT 4.1    // Voltaje en agua limpia

void setup() {
  Serial.begin(9600);
  delay(2000);
  tempSensor.begin();
  
  // Inicializar buffer TDS
  for(int i=0; i<TDS_SAMPLES; i++) {
    tdsBuffer[i] = analogRead(TDS_PIN);
  }
}

void loop() {
  // Muestreo NO BLOQUEANTE de TDS
  if(millis() - lastTdsTime >= 5) {  // 5ms entre muestras
    lastTdsTime = millis();
    tdsBuffer[tdsIndex] = analogRead(TDS_PIN);
    tdsIndex = (tdsIndex + 1) % TDS_SAMPLES;
  }

  // Enviar datos cada segundo (sincronizado)
  static unsigned long lastSend = 0;
  if(millis() - lastSend >= 1000) {
    lastSend = millis();
    
    // Leer temperatura
    tempSensor.requestTemperatures();
    float temp = tempSensor.getTempCByIndex(0);
    
    // Leer turbidez
    float turb = readTurbidity();
    
    // Leer pH (con mejor fórmula)
    float ph = readPH();
    
    // Calcular TDS
    float tds = calculateTDS(temp);
    
    // Mostrar resultados
    Serial.print("TEMP:"); Serial.print(temp, 1);
    Serial.print(",PH:"); Serial.print(ph, 2);
    Serial.print(",TURB:"); Serial.print(turb, 0);
    Serial.print(",TDS:"); Serial.println(tds, 1);
  }
}

float readTurbidity() {
  int raw = analogRead(TURB_PIN);
  float volt = raw * (5.0 / 1023.0);
  float NTU = 3000 - (volt * 1000); // Conversión aproximada
  
  
  return max(0.0, NTU);  // Evitar valores negativos
}

float readPH() {
  int raw = analogRead(PH_PIN);
  float volt = raw * (5.0 / 1023.0);
  return 7.0 + ((PH_NEUTRAL_VOLT - volt) / PH_SLOPE);
}

float calculateTDS(float temp) {
  // Calcular mediana
  int sorted[TDS_SAMPLES];
  memcpy(sorted, tdsBuffer, sizeof(tdsBuffer));
  
  // Ordenamiento optimizado
  for(int i=0; i<TDS_SAMPLES-1; i++) {
    for(int j=i+1; j<TDS_SAMPLES; j++) {
      if(sorted[i] > sorted[j]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  
  float medianVolt = sorted[TDS_SAMPLES/2] * (5.0 / 1024.0);
  float comp = 1.0 + 0.02 * (25.0 - temp);
  float compVolt = medianVolt / comp;
  
  return (133.42*pow(compVolt,3) - 255.86*sq(compVolt) + 857.39*compVolt) * 0.5;
}
/*// Pines ADC1 (ESP32 ADC)
const int pinTemp = 32;   // LM35 (0–1.5V)
const int pinPH   = 33;   // módulo pH con divisor
const int pinTurb = 34;   // turbidez con divisor
const int pinTDS  = 35;   // TDS con divisor

// Parámetros del ADC
const float VREF = 3.3;      // Voltaje referencia ESP32
const int ADC_RES = 4095;    // Resolución ADC 12 bits

// Función para leer promedio para estabilidad
int leerPromedio(int pin, int muestras = 10) {
  long suma = 0;
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(pin);
    delay(5);
  }
  return suma / muestras;
}

void setup() {
  Serial.begin(115200);  // Cambiado a 115200 baudios
  delay(1000);
}

void loop() {
  int rawTemp = leerPromedio(pinTemp);
  int rawPH   = leerPromedio(pinPH);
  int rawTurb = leerPromedio(pinTurb);
  int rawTDS  = leerPromedio(pinTDS);

  float voltTemp = rawTemp * VREF / ADC_RES;
  float voltPH   = rawPH   * VREF / ADC_RES;
  float voltTurb = rawTurb * VREF / ADC_RES;
  float voltTDS  = rawTDS  * VREF / ADC_RES;

  // Calcular valores (ajustar calibración)
  float temperatura = voltTemp * 100.0;   // LM35: 10mV/°C
  float ph = 3.5 * voltPH;                 // Ejemplo, ajustar según sensor
  float ntu = 3000 - (voltTurb * 1000);   // Estimación turbidez inversa
  float tds = voltTDS * 500;               // ppm

  // Enviar datos por Serial en formato esperado
  Serial.print("TEMP:"); Serial.print(temperatura, 1);
  Serial.print(",PH:"); Serial.print(ph, 2);
  Serial.print(",TURB:"); Serial.print((int)ntu);
  Serial.print(",TDS:"); Serial.println((int)tds);

  delay(500); // Cada medio segundo
}


// P R U E B A  S I M U L A D A

unsigned long lastMillis = 0;
int contador = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  if (now - lastMillis >= 500) { // enviar cada 500 ms
    lastMillis = now;

    // Simular datos variables con funciones sencillas
    float temperatura = 20.0 + 5.0 * sin(contador * 0.1);   // oscila entre 15 y 25°C
    float ph = 7.0 + 0.5 * cos(contador * 0.05);           // oscila entre 6.5 y 7.5
    int turbidez = 200 + (int)(50 * sin(contador * 0.2));  // oscila entre 150 y 250 NTU
    int tds = 300 + (int)(100 * cos(contador * 0.15));     // oscila entre 200 y 400 ppm

    Serial.print("TEMP:"); Serial.print(temperatura, 1);
    Serial.print(",PH:"); Serial.print(ph, 2);
    Serial.print(",TURB:"); Serial.print(turbidez);
    Serial.print(",TDS:"); Serial.println(tds);

    contador++;
  }
}
*/
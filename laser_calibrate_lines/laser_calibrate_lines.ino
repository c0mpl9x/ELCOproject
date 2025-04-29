// Laser Control for ESP32 with Optical Sensor Index
#include <Arduino.h>

// Pines de conexión
typedef uint16_t u16;
typedef uint32_t u32;
const int laserPin  = 25;  // Pin del láser
const int sensorPin = 14;  // Pin del interruptor óptico (index)

// Número de espejos/líneas
const u16 NUM_MIRRORS = 12;

// Variables de estado de la revolución
volatile unsigned long revolutionStart  = 0;
volatile unsigned long revolutionPeriod = 2000000;  // valor inicial estimado (2 s)
volatile bool newRevolution              = false;

// Ángulos en décimas de grado (recibidos por serial)
volatile u16 startAngleTenths[NUM_MIRRORS];
volatile u16 endAngleTenths[NUM_MIRRORS];

// Tiempos en microsegundos (calculados cada vuelta)
u32 startMicro[NUM_MIRRORS];
u32 endMicro[NUM_MIRRORS];

// ISR: detecta el índice de la revolución
void IRAM_ATTR sensorISR() {
  unsigned long now = micros();
  revolutionPeriod = now - revolutionStart;
  revolutionStart  = now;
  newRevolution    = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorISR, FALLING);

  // Inicializa ángulos a cero (todas las líneas deshabilitadas)
  for (u16 i = 0; i < NUM_MIRRORS; i++) {
    startAngleTenths[i] = 0;
    endAngleTenths[i]   = 0;
  }
  revolutionStart = micros();
}

void loop() {
  // 1) Leer comandos seriales: "UPDATE,s0,e0,s1,e1,...,s11,e11\n"
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("UPDATE")) {
      int values[NUM_MIRRORS * 2];
      int idx = 0;
      char *buf = const_cast<char*>(cmd.c_str());
      strtok(buf, ",");  // descarta "UPDATE"
      char *tok;
      while ((tok = strtok(NULL, ",")) != NULL && idx < NUM_MIRRORS * 2) {
        values[idx++] = atoi(tok);
      }
      if (idx == NUM_MIRRORS * 2) {
        for (u16 i = 0; i < NUM_MIRRORS; i++) {
          startAngleTenths[i] = values[2 * i];
          endAngleTenths[i]   = values[2 * i + 1];
        }
        Serial.println("OK");
      } else {
        Serial.println("ERR: Invalid count");
      }
    }
  }

  // 2) Al inicio de cada revolución, recalcula tiempos (µs) para cada espejo
  if (newRevolution) {
    newRevolution = false;
    // Tiempo por décima de grado
    float microStep = (float)revolutionPeriod / 3600.0;
    for (u16 i = 0; i < NUM_MIRRORS; i++) {
      // Si start == end → ventana de duración cero → línea deshabilitada
      startMicro[i] = (u32)(startAngleTenths[i] * microStep);
      endMicro[i]   = (u32)(endAngleTenths[i]   * microStep);
    }
  }

  // 3) Control en tiempo real: enciende/apaga láser según ventanas habilitadas
  unsigned long t = micros() - revolutionStart;
  bool on = false;
  for (u16 i = 0; i < NUM_MIRRORS; i++) {
    // Solo entramos si start < end (ventana válida) y t está dentro
    if (startMicro[i] < endMicro[i] && t >= startMicro[i] && t < endMicro[i]) {
      on = true;
      break;
    }
  }
  digitalWrite(laserPin, on ? HIGH : LOW);
}

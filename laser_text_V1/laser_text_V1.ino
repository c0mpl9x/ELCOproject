// Laser Control for ESP32 con Configuración Estática
#include <Arduino.h>

typedef uint16_t u16;
typedef uint32_t u32;
const int laserPin  = 25;
const int sensorPin = 14;
const u16 NUM_MIRRORS = 12;
const u16 NUM_CHARS = 4;
const u16 NUM_BITS = 6;

// Configuración en décimas de grado
const u16 startAngleTenths[NUM_MIRRORS] = {
  46,  3053, 2450, 1864, 1256, 641,
  2778, 3363, 2143, 1540, 959, 355
};
const u16 endAngleTenths[NUM_MIRRORS] = {
  196, 3203, 2600, 2014, 1406, 791,
  2928, 3513, 2293, 1690, 1109, 505
};

const u16 widthLineTenths = 150;
const u16 widthCharTenths = 30;
const u16 widthBitTenths = 5;
const u16 widthSpaceTenths = 10;

typedef uint16_t rowmask_t;
const u16 FONT_CHARS = 5;
const rowmask_t font[FONT_CHARS][NUM_MIRRORS] = {
  // H
  {0b110011,0b110011,0b110011,0b110011,0b111111,0b111111,0b110011,0b110011,0b110011,0b110011,0b110011,0b110011},
  // O
  {0b011110,0b110011,0b110011,0b110011,0b110011,0b110011,0b110011,0b110011,0b110011,0b110011,0b110011,0b011110},
  // L
  {0b110000,0b110000,0b110000,0b110000,0b110000,0b110000,0b110000,0b110000,0b110000,0b110000,0b111111,0b111111},
  // A
  {0b011110,0b110011,0b110011,0b110011,0b110011,0b111111,0b110011,0b110011,0b110011,0b110011,0b110011,0b110011},
  // Blanc
  {0b111111,0b111111,0b111111,0b111111,0b111111,0b111111,0b111111,0b111111,0b111111,0b111111,0b111111,0b111111}
};

volatile unsigned long revolutionStart  = 0;
volatile unsigned long revolutionPeriod = 2000000;
volatile bool newRevolution = false;
u32 startMicro[NUM_MIRRORS];
u32 endMicro[NUM_MIRRORS];
u32 startChar[NUM_CHARS][NUM_MIRRORS];
u32 endChar[NUM_CHARS][NUM_MIRRORS];
u32 startBit[NUM_BITS][NUM_CHARS][NUM_MIRRORS];
u32 endBit[NUM_BITS][NUM_CHARS][NUM_MIRRORS];
void IRAM_ATTR sensorISR() {
  unsigned long now = micros();
  revolutionPeriod = now - revolutionStart;
  revolutionStart  = now;
  newRevolution    = true;
}

void setup() {
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorISR, FALLING);
  revolutionStart = micros();
}

void loop() {
  if (newRevolution) {
    newRevolution = false;
    float step = (float)revolutionPeriod / 3600.0;
    for (u16 i = 0; i < NUM_MIRRORS; i++) {
      startMicro[i] = (u32)(startAngleTenths[i] * step);
      endMicro[i]   = (u32)(endAngleTenths[i]   * step);
      for (u16 j = 0; j < NUM_CHARS; j++){
        startChar[j][i] = (u32)((startAngleTenths[i] + j * (widthCharTenths + widthSpaceTenths)) * step);
        endChar[j][i] = (u32)((startAngleTenths[i] + j * (widthCharTenths + widthSpaceTenths) + widthCharTenths) * step);
        for (u16 k = 0; k < NUM_BITS; k++){
          startBit[k][j][i] = (u32)((startAngleTenths[i] + j * (widthCharTenths + widthSpaceTenths) + k * widthBitTenths) * step);
          endBit[k][j][i] = (u32)((startAngleTenths[i] + j * (widthCharTenths + widthSpaceTenths) + widthBitTenths * (k + 1)) * step);
        }
      }
    }
  }
  unsigned long t = micros() - revolutionStart;
  bool laserOn = false;
  for (u16 i = 0; i < NUM_MIRRORS; i++) {
    for (u16 j = 0; j < NUM_CHARS; j++) {
      for (u16 k = 0; k < NUM_BITS; k++) {
        // Creamos la máscara: 
        // si k=0 queremos el bit más significativo, 
        // por lo que desplazamos (NUM_BITS-1-k)
        rowmask_t mask = 1 << (NUM_BITS - 1 - k);
        bool bitIsOne = (font[j][i] & mask) != 0;
        if (t >= startBit[k][j][i] && t <= endBit[k][j][i]) {
          laserOn = bitIsOne;
          break;
        }
      }
    }
  }
  digitalWrite(laserPin, laserOn ? HIGH : LOW);
    /*
  for (u16 i = 0; i < NUM_MIRRORS; i++) {
    if (t >= startMicro[i] && t < endMicro[i]) {
      // completar para que se encienda el laser si se está en el bit correspondiente al caracter Blanc
      
      laserOn = true; 
      break;
    }
  }
  */
}
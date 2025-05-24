#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "arduinoFFT.h"
#include <math.h>
#include "config.h"


#define YIELD_DELAY 1
static int yield_user_delay = YIELD_DELAY;
static int mode = 0;
// -------------------- Definiciones de pines y parámetros --------------------
#define TFT_CS   5    // Se definen antes de usarlos
#define TFT_DC   19
#define TFT_RST  33

// SDA 23
// SCL 18

#define MICRO_PIN 39

#define MAX_BANDS 16         // Número máximo de bandas que permitiremos
#define NUM_BANDS 32

#define SAMPLES 256          
#define SAMPLING_FREQUENCY 40000 // Hz (Nyquist = 20000 Hz)
//#define amplitude 200           // Factor de escala

// -------------------- Estado del botón BOOT (GPIO 0) ---------------
volatile bool bootButtonState = true;
bool prev_boton = true;

// Layout de la pantalla (rotada 90°: 320x240)
#define GRAPH_WIDTH 320                
#define INFO_WIDTH 45
#define SPECTRUM_WIDTH (GRAPH_WIDTH - INFO_WIDTH) // Área para las barras
#define GRAPH_Y 30                     // 30 px en la parte superior
#define INFO_Y 225                     // Zona de información inicia en y = 225
#define GRAPH_HEIGHT (220 - GRAPH_Y)   // 220 - 30 = 190 px
#define BAR_GAP 4

// 32 bandas log-espaciadas de 100 a 20000 Hz,
// redondeadas y con salto mínimo de 1 bin (78.125 Hz)
// con salto mínimo = 1 bin (156.25 Hz)
const float normal[NUM_BANDS] = {
    100.000f,256.250f,412.500f,568.750f,725.000f,881.250f,1037.500f,1193.750f,1350.000f,1506.250f,1662.500f,1818.750f,1975.000f,2131.250f,2287.500f,2443.750f,2600.000f,2756.250f,2912.500f,3068.750f,3225.000f,3620.448f,4295.256f,5095.840f,6045.644f,7172.480f,8509.345f,10095.385f,11977.045f,14209.423f,16857.890f,20000.000f
};

// 32 bandas log-espaciadas de 1000 a 4000 Hz,
// con salto mínimo = 1 bin (156.25 Hz)
const float silvar[NUM_BANDS] = {
    1000.000f,1156.250f,1312.500f,1468.750f,1625.000f,1781.250f,1937.500f,2093.750f,2250.000f,2406.250f,2562.500f,2718.750f,2875.000f,3031.250f,3187.500f,3343.750f,3500.000f,3656.250f,3812.500f,3968.750f,4125.000f,4281.250f,4437.500f,4593.750f,4750.000f,4906.250f,5062.500f,5218.750f,5375.000f,5531.250f,5687.500f,4000.000f
};
float bandLimits[NUM_BANDS] = {
    100.000f,256.250f,412.500f,568.750f,725.000f,881.250f,1037.500f,1193.750f,1350.000f,1506.250f,1662.500f,1818.750f,1975.000f,2131.250f,2287.500f,2443.750f,2600.000f,2756.250f,2912.500f,3068.750f,3225.000f,3620.448f,4295.256f,5095.840f,6045.644f,7172.480f,8509.345f,10095.385f,11977.045f,14209.423f,16857.890f,20000.000f
};

//-----------Defines for automatic gain control------/
int amplitude[3] = { 200, 200, 200 };  // Depending on your audio source level, you may need to increase this value
int filter[3] = { 2000, 2000, 5000 }; // To filter noise. The last one is the most important since it corresponds to the higher gain mode
int band_values = 0; // if this value is too low, the gain will increase

// -------------------- Variables globales para muestreo y FFT --------------------
unsigned int sampling_period_us;
unsigned long newTime, oldTime;

// -------------------- Variables globales para dibujo --------------------
byte peak[NUM_BANDS] = {0,0,0,0,0,0,0,0};
int prevBarHeight[NUM_BANDS] = {0,0,0,0,0,0,0,0};
int prevPeakY[NUM_BANDS] = { 
  GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y + GRAPH_HEIGHT,
  GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y + GRAPH_HEIGHT
};

// -------------------- Buffers para FFT --------------------
// -------------------- Buffers para FFT --------------------
// Usa float (4 bytes) y fuerza a SRAM interna
DRAM_ATTR static float vReal[SAMPLES];
DRAM_ATTR static float vImag[SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);


// -------------------- Doble búfer para muestreo y mutex --------------------
volatile int currentBuffer = 0;
int buffer0[SAMPLES];
int buffer1[SAMPLES];
volatile bool bufferReady[2] = {false, false};
SemaphoreHandle_t bufferMutex = NULL;

// -------------------- Medidas -----------------------------------
volatile unsigned long samplingTime = 0;
volatile unsigned long lastSamplingTime = 0;
volatile unsigned long procesingTime = 0;

// -------------------- Objeto para la pantalla --------------------
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

/////////////////////////////////////////////////////////////////////////

void change_band_limits(){
  if (bandLimits[0] == silvar[0]) {
    // Sustituye bandLimits por normal
    for (int i = 0; i < NUM_BANDS; i++) {
      bandLimits[i] = normal[i];
    }
  } else {
    for (int i = 0; i < NUM_BANDS; i++) {
      bandLimits[i] = silvar[i];
    }    
  }
  // drawFrequencyLabels();
}

void buttonPressed(){
  change_band_limits();
  tft.setCursor(SPECTRUM_WIDTH + 20, 200);
  tft.print("BOTON");
}
void buttonReleased(){
  tft.fillRect(SPECTRUM_WIDTH + 20, 200, 30, 8, ST77XX_BLACK);
}

// -------------------- Suavizado con curva Bézier (general, con 11 puntos) --------------------
struct Point {
  float x, y;
};

const int numOfHistoryValues = 11; 
float SMOOTHING = 0.5;
Point controlPoints[NUM_BANDS][numOfHistoryValues];
float valueHistory[NUM_BANDS][numOfHistoryValues];

float factorial(int n) {
  float f = 1;
  for (int i = 1; i <= n; i++) {
    f *= i;
  }
  return f;
}

float binomial(int n, int k) {
  return factorial(n) / (factorial(k) * factorial(n - k));
}

float bezierGeneral(float p[], int n, float t) {
  int order = n - 1;
  float result = 0;
  for (int i = 0; i < n; i++) {
    float bern = binomial(order, i) * pow(1 - t, order - i) * pow(t, i);
    result += bern * p[i];
  }
  return result;
}

void updateControlPoints(float newValue, int barIndex) {
  for (int i = numOfHistoryValues - 1; i > 0; i--) {
    valueHistory[barIndex][i] = valueHistory[barIndex][i - 1];
  }
  valueHistory[barIndex][0] = newValue;
  for (int i = 0; i < numOfHistoryValues; i++) {
    controlPoints[barIndex][i].y = valueHistory[barIndex][i];
  }
}

float getSmoothedValue(int barIndex, float t) {
  float p[numOfHistoryValues];
  for (int i = 0; i < numOfHistoryValues; i++) {
    p[i] = controlPoints[barIndex][i].y;
  }
  return bezierGeneral(p, numOfHistoryValues, t);
}

// -------------------- Funciones de pantalla --------------------
void drawFrequencyLabels() {
  int bandWidth = SPECTRUM_WIDTH / NUM_BANDS;
  tft.fillRect(0, INFO_Y + 2, SPECTRUM_WIDTH, 8, ST77XX_BLACK);
  for (int j = 0; j < NUM_BANDS; j++) {
    float f_max = bandLimits[j];
    char buf[10];
    if (f_max < 1000)
      sprintf(buf, "%.1f", f_max / 1000.0);
    else
      sprintf(buf, "%.0f", f_max / 1000.0);
    int xPos = j * bandWidth + (bandWidth / 2) - 10;
    tft.setCursor(xPos, INFO_Y + 2);
    tft.print(buf);
  }
}

void drawStaticInfo() {
  // drawFrequencyLabels();
  tft.setCursor(SPECTRUM_WIDTH, 0);
  tft.print("Tp:        ");
  tft.setCursor(SPECTRUM_WIDTH, 10);
  tft.print("Ts:        ");
  tft.setCursor(SPECTRUM_WIDTH, 20);
  tft.print("fps      ");
}

// -------------------- Tareas FreeRTOS --------------------

// Tarea de muestreo (núcleo 0)
void TaskSampling(void * parameter) {
  for (;;) {

     LASER_OFF();
    int *buf = (currentBuffer == 0) ? buffer0 : buffer1;

    for (int i = 0; i < SAMPLES; i++) {
      newTime = micros() - oldTime;
      oldTime = newTime;
      buf[i] = analogRead(MICRO_PIN);
      while (micros() < (newTime + sampling_period_us)) { yield(); LASER_OFF();}
    }
    xSemaphoreTake(bufferMutex, portMAX_DELAY);
    bufferReady[currentBuffer] = true;
    currentBuffer = (1 - currentBuffer);
    xSemaphoreGive(bufferMutex);

    samplingTime = micros() - lastSamplingTime;
    lastSamplingTime = micros();
    // yield();
    vTaskDelay(yield_user_delay);
  }
  vTaskDelay(1);
    
}

// Tarea de procesamiento (núcleo 1)
void TaskProcessing(void * parameter) {
  for (;;) {

    int localBuffer[SAMPLES];
    if (bufferReady[0] || bufferReady[1]) {

      LASER_OFF();
      xSemaphoreTake(bufferMutex, portMAX_DELAY);
      int readyBuffer = bufferReady[0] ? 0 : 1;
      memcpy(localBuffer, (readyBuffer == 0) ? buffer0 : buffer1, sizeof(int) * SAMPLES);
      bufferReady[readyBuffer] = false;
      xSemaphoreGive(bufferMutex);
      
      // Copia al arreglo FFT
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = localBuffer[i];
        vImag[i] = 0;
      }
      
      FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);
      FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);
      
      // Cálculo de valores por banda (índices 1 a SAMPLES / 2)
      int bandValues[NUM_BANDS] = {0};
      for (int i = 1; i < SAMPLES / 2; i++) {
        if (vReal[i] > filter[2]) {
          int value = (int)vReal[i] / amplitude[2];
          float f = i * ((float)SAMPLING_FREQUENCY / SAMPLES);
          for (int j = 0; j < NUM_BANDS; j++) {
            float lowerBound = (j == 0) ? 0 : bandLimits[j - 1];
            float upperBound = bandLimits[j];
            if (f >= lowerBound && f < upperBound) {
              if (value > bandValues[j])
                bandValues[j] = value;
              break;
            }
          }
        }
      }
      
      unsigned long procStart = micros();
      for (int i = 0; i < NUM_BANDS; i++) {
        updateControlPoints(bandValues[i], i);
        bandValues[i] = getSmoothedValue(i, 0.2);
      }

      unsigned long procCycle = micros() - procStart;

      // === DRAWING BANDS ===
      int bandWidth = SPECTRUM_WIDTH / NUM_BANDS;
      for (int band = 0; band < NUM_BANDS; band++) {
        int dsize = bandValues[band];
        if (dsize > 50) {
          dsize = 50;
        }
        int newBarHeight = (dsize * GRAPH_HEIGHT) / 50;
        int x = band * bandWidth + (BAR_GAP / 2);
        int effectiveWidth = bandWidth - BAR_GAP;
        
        if (newBarHeight > prevBarHeight[band]) {
          int diff = newBarHeight - prevBarHeight[band];
          int yStart = GRAPH_Y + GRAPH_HEIGHT - newBarHeight;
          tft.fillRect(x, yStart, effectiveWidth, diff, ST77XX_WHITE);
        } else if (newBarHeight < prevBarHeight[band]) {
          int diff = prevBarHeight[band] - newBarHeight;
          int yStart = GRAPH_Y + GRAPH_HEIGHT - prevBarHeight[band];
          tft.fillRect(x, yStart, effectiveWidth, diff, ST77XX_BLACK);
        }
        prevBarHeight[band] = newBarHeight;
        
        if (dsize > peak[band])
          peak[band] = dsize;
        else if (peak[band] > 0)
          peak[band]--;
        
        int newPeakY = GRAPH_Y + GRAPH_HEIGHT - (peak[band] * GRAPH_HEIGHT) / 50;
        if (newPeakY != prevPeakY[band]) {
          int barTop = GRAPH_Y + GRAPH_HEIGHT - newBarHeight;
          uint16_t bgColor = (prevPeakY[band] >= barTop) ? ST77XX_WHITE : ST77XX_BLACK;
          tft.fillRect(x, prevPeakY[band], effectiveWidth, 1, bgColor);
        }
        tft.drawFastHLine(x, newPeakY, effectiveWidth, ST77XX_RED);
        prevPeakY[band] = newPeakY;
      }

        bootButtonState = digitalRead(0); // LOW si está presionado
        if (bootButtonState == false && bootButtonState != prev_boton) {
          buttonPressed();
        } else if (bootButtonState == true && bootButtonState != prev_boton) {
          buttonReleased();
        }
        prev_boton = bootButtonState;

            

      unsigned long nowProcesingTime = micros() - procesingTime;
      procesingTime = micros();

      float fps = 1000000.0 / nowProcesingTime;

      unsigned long currentMillis = millis();
    
      // Actualiza la zona de información en pantalla cada 1 s:
      static unsigned long lastInfoUpdate = 0;
      if (currentMillis - lastInfoUpdate >= 1000) {
      lastInfoUpdate = currentMillis;
      // Se asume que en la última iteración de procesamiento se calculó el fps y cycleTime
      
      // Se muestra en pantalla:
      // Para ello, se usa el área a la derecha (a partir de SPECTRUM_WIDTH+30)
      // Se actualizan T (tiempo del ciclo en ms) y FPS
      float T_ms = procCycle / 1000.0; 

      tft.fillRect(SPECTRUM_WIDTH + 20, 0, 30, 8, ST77XX_BLACK);
      tft.setCursor(SPECTRUM_WIDTH + 20, 0);
      tft.print(T_ms, 0);

      float localSamplingTime = samplingTime / 1000.0;
      
      tft.fillRect(SPECTRUM_WIDTH + 20, 10, 30, 8, ST77XX_BLACK);
      tft.setCursor(SPECTRUM_WIDTH + 20, 10);
      tft.print(localSamplingTime, 0);

      tft.fillRect(SPECTRUM_WIDTH + 20, 20, 30, 8, ST77XX_BLACK);
      tft.setCursor(SPECTRUM_WIDTH + 20, 20);
      tft.print(fps, 0);         
      }
    } else {
      vTaskDelay(10);
    }
  // yield();
  vTaskDelay(yield_user_delay);
  }
  vTaskDelay(1);
    
}
// -------------------- Laser Text Code -------------------------------
// Laser Control for ESP32 xd

volatile unsigned long revolutionStart  = 0;
volatile unsigned long revolutionPeriod = 0;
volatile u8 revCount = 0;
volatile unsigned long period = 0;
volatile unsigned long sum = 0;
static uint8_t SCROLL_INTERVAL_REVS = 25;  // scroll cada 3 revs
static float offset_step = (float)(widthCharTenths + widthSpaceTenths) / (float)SCROLL_INTERVAL_REVS;
static float offset = 0;

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

volatile bool newRevolution = false;
volatile bool laser_user_on = false;
volatile bool laser_speed_on = false;
volatile bool motor_user_on = false;
volatile bool motor_speed_on = true;

u32 startMicro[NUM_MIRRORS];
u32 endMicro[NUM_MIRRORS];
u32 startChar[MAX_NUM_CHARS][NUM_MIRRORS];
u32 endChar[MAX_NUM_CHARS][NUM_MIRRORS];
u32 startBit[NUM_BITS][MAX_NUM_CHARS][NUM_MIRRORS];
u32 endBit[NUM_BITS][MAX_NUM_CHARS][NUM_MIRRORS];

// habilitadores de flip
bool flipHorizontal = false;
bool flipVertical   = false;
bool scroll         = false;

uint8_t       scrollCounter        = 0;

// tablas de mapeo
u8 rowMap[NUM_MIRRORS];
u8  bitMap[NUM_BITS];  // usamos u8 porque NUM_BITS ≤ 8

// Evento
struct Event { u32 timeUs; bool on; };
static Event events[2 * NUM_MIRRORS * MAX_NUM_CHARS * NUM_BITS * NUM_REPS];  // ajustar tamaño: planCycles*NUM_MIRRORS*MAX_NUM_CHARS*NUM_BITS*2
static u16   currentEvent = 0;
static u16 eventCount    = 0;   // cuántos eventos hay

void IRAM_ATTR sensorISR() {
  unsigned long now = micros();
  revolutionPeriod = now - revolutionStart;
  revolutionStart  = now;
  newRevolution    = true;
}

int compareEvents(const void* pa, const void* pb) {
  const Event* a = (const Event*)pa;
  const Event* b = (const Event*)pb;
  if (a->timeUs < b->timeUs) return -1;
  if (a->timeUs > b->timeUs) return  1;
  return 0;
}

String normalizeUTF8(const String &in) {
  String out;
  size_t i = 0;
  while (i < in.length()) {
    uint8_t b = in[i];
    // Detectar inicio de ñ (0xC3 0xB1) ó Ñ (0xC3 0x91)
    if (b == 0xC3 && i+1 < in.length()) {
      uint8_t b2 = in[i+1];
      if (b2 == 0xB1 || b2 == 0x91) {     // ñ
        out += (char)0xF1;
        i += 2;
        continue;
      }
    }
    // Sino, copiamos el byte tal cual
    out += (char)b;
    i++;
  }
  return out;
}


// Variables globales
int  messageLen = 0;  
rowmask_t messageBitmap[MAX_MESSAGE_LEN][NUM_MIRRORS];
char     fullMessage[MAX_MESSAGE_LEN+1];
uint8_t  windowStart = 0;
rowmask_t windowBitmap[MAX_NUM_CHARS][NUM_MIRRORS];
// Convierte 'A'–'Z' o 'a'–'z' a índice 0–25, y cualquier otro a índice 26 (espacio)
uint8_t charToIndex(char c) {
  // Letras
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a';

  // Corazones
  if (c == '<') return 26;   // HEART LEFT
  if (c == '>') return 27;   // HEART RIGHT

  // Dígitos
  if (c >= '0' && c <= '9') return 28 + (c - '0');

  // Puntuación
  if (c == ',')  return 38;
  if (c == '.')  return 39;
  if (c == ':')  return 40;
  if (c == ';')  return 41;
  if (c == '?')  return 42;
  if (c == '¿')  return 43;  // ojo con la codificación
  if (c == '!')  return 44;
  if (c == '¡')  return 45;  // ojo con la codificación
  if (c == '-')  return 46;
  if (c == '+')  return 47;
  if ((uint8_t)c == 0xF1) return 48;

  // Espacio o carácter no soportado
  return ALPHABET_SIZE;  // aquí puedes poner el índice de tu glyph "espacio" si lo tienes
}

// Llama a esta función para fijar el mensaje que se va a mostrar.
// msg: puntero a cadena C-terminada
// Devuelve messageLen y rellena fullMessage.
void setMessage(char* msg) {
  // 1) Guarda el texto crudo en fullMessage y su longitud
  strncpy(fullMessage, msg, MAX_MESSAGE_LEN);
  fullMessage[MAX_MESSAGE_LEN] = '\0';
  messageLen = strlen(fullMessage);
  scroll = (messageLen > num_chars);
  if (!scroll){
    for (messageLen; messageLen < num_chars; messageLen++){
      fullMessage[messageLen] = ' ';
    }
  }
  fullMessage[min(messageLen++, MAX_MESSAGE_LEN)] = '\0';
  // 2) Reinicia la ventana para que empiece desde el principio
  windowStart = 0;
  // 3) Si el mensaje no cabe, activar scroll. Desactivarlo si sí cabe
}
// Buffer y estado
static char serialBuf[MAX_MESSAGE_LEN+1];
static uint8_t serialPos = 0;
static bool    msgPending = false;

static char bleBuf[MAX_MESSAGE_LEN+1];

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer)    { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

// Callback cuando el móvil escribe en RX
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String rx = pChar->getValue();
    if (rx.length() == 0) return;
    // 1) Normalizamos UTF-8 -> Latin1
    String norm = normalizeUTF8(rx);
    // 2) Lo volcamos a tu buffer c-string
    size_t len = min((size_t)norm.length(), (size_t)MAX_MESSAGE_LEN);
    norm.toCharArray(bleBuf, len+1);
    handleCommand(bleBuf);
  }
};



// Llamar una sola vez por iteración de loop():
void pollSerial() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      // fin de línea
      if (serialPos > 0) {
        serialBuf[serialPos] = '\0';
        msgPending = true;
      }
      serialPos = 0;
    } else {
      // acumula caracter, sin overflow
      if (serialPos < MAX_MESSAGE_LEN) {
        serialBuf[serialPos++] = c;
      }
    }
  }
}
// Buffers para Bluetooth
static char btBuf[MAX_MESSAGE_LEN+1];
static uint8_t btPos = 0;
static bool    btMsgPending = false;

void updateWindowScroll() {
  for (uint8_t j = 0; j < num_chars; j++) {
    uint8_t idx = (windowStart + j) % messageLen;
    uint8_t alpha = charToIndex(fullMessage[idx]);
    for (uint8_t i = 0; i < NUM_MIRRORS; i++) {
      windowBitmap[j][i] = (alpha < ALPHABET_SIZE)
        ? pgm_read_word(&alphabet[alpha][i])
        : 0;
    }
  }
  if (scroll) windowStart = (windowStart + 1) % messageLen;
}


// // Create debounce instance (default constructor uses active HIGH)
// PinButton  blackButton(buttonBlackPin);
// // Create debounce instance (default constructor uses active HIGH)
// PinButton  yellowButton(buttonYellowPin);
PinButton buttonRST(buttonRSTPin, INPUT_PULLUP);

void updateNumChars(int new_num_chars){
  if (new_num_chars > 3 && new_num_chars <= MAX_NUM_CHARS) {
    num_chars = new_num_chars;
    widthCharTenths  = widthLineTenths / (num_chars + 1);
    widthBitTenths   = round(widthCharTenths / NUM_BITS);
    widthSpaceTenths = 3 * widthCharTenths / (num_chars - 1);
    offset_step = (float)(widthCharTenths + widthSpaceTenths) / (float)SCROLL_INTERVAL_REVS;
    Serial.print("num_chars updated! num_chars = ");
    Serial.println(num_chars);
  } else {
    Serial.print("NUM CHARS MENOR QUE 4 O MAYOR QUE "); Serial.println(MAX_NUM_CHARS);
  }

}

void blackPressHandle(){
  num_chars = (num_chars + 1) % MAX_NUM_CHARS;
  widthCharTenths  = widthLineTenths / (num_chars + 1);
  widthBitTenths   = round(widthCharTenths / NUM_BITS);
  widthSpaceTenths = 3 * widthCharTenths / (num_chars - 1);
  offset_step = (float)(widthCharTenths + widthSpaceTenths) / (float)SCROLL_INTERVAL_REVS;
  Serial.print("Button pressed! num_chars = ");
  Serial.println(num_chars);
}
// void blackDoublePressHandle(){
//   Serial.println("Button long pressed!");
//   scroll = !scroll;
//   digitalWrite(ledPin, scroll);
// }
void yellowPressHandle(){
  num_chars = (num_chars - 1 + MAX_NUM_CHARS) % MAX_NUM_CHARS;
  widthCharTenths  = widthLineTenths / (num_chars + 1);
  widthBitTenths   = round(widthCharTenths / NUM_BITS);
  widthSpaceTenths = 3 * widthCharTenths / (num_chars - 1);
  offset_step = (float)(widthCharTenths + widthSpaceTenths) / (float)SCROLL_INTERVAL_REVS;
  
  Serial.print("Button pressed! num_chars = ");
  Serial.println(num_chars);
}
void rstSingleClickHandle(){
  scroll = !scroll;
  Serial.println("boton rst presionado");
}
void rstPressHandle(){
  motor_user_on = !motor_user_on;
  // digitalWrite(motorPin, motor_user_on ? HIGH : LOW);
  if (motor_user_on) ledcWrite(motorPin, 200);
  else ledcWrite(motorPin, 0);
  Serial.println("boton rst long presionado");
}
void rstDoublePressHandle(){
  laser_user_on = !laser_user_on;
  if (laser_user_on) LASER_ON(); else LASER_OFF();
  Serial.println("boton rst doble presionado");
}
// Parsear comando recibido y actualizar estados
void handleCommand(char* cmdBuf) {
  String cmd = String(cmdBuf);
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "MOTOR ON") {
    motor_user_on = true;
    if (motor_user_on) ledcWrite(motorPin, 200);
    else ledcWrite(motorPin, 0);
  }
  else if (cmd == "MOTOR OFF") {
    motor_user_on = false;
    if (motor_user_on) ledcWrite(motorPin, 200);
    else ledcWrite(motorPin, 0);
  }
  else if (cmd == "MOTOR TOGGLE"){
    motor_user_on = !motor_user_on;
    if (motor_user_on) ledcWrite(motorPin, 200);
    else ledcWrite(motorPin, 0);
  }
  else if (cmd == "LASER ON") {
    laser_user_on = true;
    LASER_ON();
  }
  else if (cmd == "LASER OFF") {
    laser_user_on = false;
    LASER_OFF();
  }
  else if (cmd == "LASER TOGGLE") {
    laser_user_on = !laser_user_on;
    if (laser_user_on) LASER_ON(); else LASER_OFF();
  }
  else if (cmd == "SCROLL ON") {
    scroll = true;
  }
  else if (cmd == "SCROLL OFF") {
    scroll = false;
  } 
  else if(cmd == "SCROLL TOGGLE"){
    scroll = !scroll;
  }
  else if (cmd == "UP SCROLL"){
    SCROLL_INTERVAL_REVS++;
    offset_step = (float)(widthCharTenths + widthSpaceTenths) / (float)SCROLL_INTERVAL_REVS;
  }
  else if (cmd == "DOWN SCROLL"){
    if(SCROLL_INTERVAL_REVS > 3){
      SCROLL_INTERVAL_REVS--;
      offset_step = (float)(widthCharTenths + widthSpaceTenths) / (float)SCROLL_INTERVAL_REVS;
    }
  }
  else if (cmd == "UP NUM CHARS"){
    updateNumChars(num_chars + 1);
  }
  else if (cmd == "DOWN NUM CHARS"){
    updateNumChars(num_chars - 1);
  }  

  else if (cmd == "YIELD") {
    yield_user_delay = (yield_user_delay == YIELD_DELAY) ? YIELD_DELAY * 3 : YIELD_DELAY;
    char yieldBuff[32];
    int len = snprintf(yieldBuff, sizeof(yieldBuff), "yield: %d", yield_user_delay);
    // Envío vía BLE-UART (o SerialBT si usas Classic)
    if (deviceConnected) {
      pTxCharacteristic->setValue((uint8_t*)yieldBuff, len);
      pTxCharacteristic->notify();
    }  
  }


  else if (cmd.startsWith("NUM CHARS ")) {
    // extrae la parte numérica y la convierte a entero
    String numStr = cmd.substring(10);      // "10" en "NUM CHARS 10"
    int n = numStr.toInt();                 // n = 10
    if (n > 0) {
      updateNumChars(n);
    }
    else {
      // opcional: notificar parámetro inválido
      if (deviceConnected) {
        pTxCharacteristic->setValue("ERR: Bad NUM CHARS\n");
        pTxCharacteristic->notify();
      }
      return;
    }
  }
  else if (cmd == "DOWN OFFSET"){
    base_offset -= 1;
    // Prepara el buffer
    char offsetBuf[32];
    int len = snprintf(offsetBuf, sizeof(offsetBuf), "OFFSET: %d", base_offset);
    // Envío vía BLE-UART (o SerialBT si usas Classic)
    if (deviceConnected) {
      pTxCharacteristic->setValue((uint8_t*)offsetBuf, len);
      pTxCharacteristic->notify();
    }
  }
    else if (cmd == "UP OFFSET"){
    base_offset += 1;
    // Prepara el buffer
    char offsetBuf[32];
    int len = snprintf(offsetBuf, sizeof(offsetBuf), "OFFSET: %d", base_offset);
    // Envío vía BLE-UART (o SerialBT si usas Classic)
    if (deviceConnected) {
      pTxCharacteristic->setValue((uint8_t*)offsetBuf, len);
      pTxCharacteristic->notify();
    }    
  }
  else if (cmd.startsWith("OFFSET ")) {
    // extrae la parte numérica y la convierte a entero
    String numStr = cmd.substring(10);      // "10" en "NUM CHARS 10"
    int n = numStr.toInt();                 // n = 10
    if (n > 0) {
      base_offset = n;
    }
  }
  else if (cmd == "FLIPH"){
    flipHorizontal = !flipHorizontal;
    // Fill rowMap: si flipVertical invierte el orden de los espejos
    for (u8 i = 0; i < NUM_MIRRORS; i++) {
      rowMap[i] = flipVertical
        ? (NUM_MIRRORS - 1 - i)
        : i;
    }
    // Fill bitMap: si flipHorizontal invierte los bits de cada carácter
    for (u8 k = 0; k < NUM_BITS; k++) {
      bitMap[k] = flipHorizontal ? k : (NUM_BITS - 1 - k);  // MSB primero
    }
  }
  else if (cmd == "FLIPV"){
    flipVertical = !flipVertical;
    // Fill rowMap: si flipVertical invierte el orden de los espejos
    for (u8 i = 0; i < NUM_MIRRORS; i++) {
      rowMap[i] = flipVertical
        ? (NUM_MIRRORS - 1 - i)
        : i;
    }
    // Fill bitMap: si flipHorizontal invierte los bits de cada carácter
    for (u8 k = 0; k < NUM_BITS; k++) {
      bitMap[k] = flipHorizontal ? k : (NUM_BITS - 1 - k);  // MSB primero
    }    
  }
  else if (cmd == "FLIPHV" || cmd == "FLIPVH"){
    flipVertical = !flipVertical;
    flipHorizontal = !flipHorizontal;
    // Fill rowMap: si flipVertical invierte el orden de los espejos
    for (u8 i = 0; i < NUM_MIRRORS; i++) {
      rowMap[i] = flipVertical ? (NUM_MIRRORS - 1 - i) : i;
    }
    // Fill bitMap: si flipHorizontal invierte los bits de cada carácter
    for (u8 k = 0; k < NUM_BITS; k++) {
      bitMap[k] = flipHorizontal ? k : (NUM_BITS - 1 - k);  // MSB primero
    }    
  }  
  // —— FIN de NUM CHARS ——
  else {
    // Cualquier otro texto lo mandamos al láser
    setMessage(cmdBuf);
  }

  // Opcional: notificar OK al móvil
  if (deviceConnected) {
    pTxCharacteristic->setValue("OK\n");
    pTxCharacteristic->notify();
  }
}

static unsigned long lastRpmSendTime = 0;  // registro de la última vez que enviamos
static int numRevs = 0;

void LaserText(){
  unsigned long t = micros() - revolutionStart;
  if (newRevolution){
    if(mode==0) vTaskDelay(4);
    laser_speed_on = revolutionPeriod < 60000;
    //Serial.println(revolutionPeriod);
    newRevolution = false;
    LASER_OFF();
    // 1) Calcula RPM: 60s / (periodo en s)
    //    revolutionPeriod µs → revolutionPeriod/1e6 s
    float rpm = 60.0f * 1e6f / (float)revolutionPeriod;

    // Tiempo transcurrido desde el último control (segundos)
    unsigned long now_ms = micros();
    float dt = (now_ms - lastControlTime) / 1000000.0;
    lastControlTime = now_ms;
    
    // === PID ===
    float error = setpointRPM - rpm;
    if (motor_user_on) integral = integral + error * dt;
    float derivative = (error - lastError) / dt;
    
    float output = Kp*error + Ki*integral + Kd*derivative;
    lastError = error;
    
    // Convertimos a 0–255 y limitamos
    int pwm = constrain(int(output), 0, 255);
    if (motor_user_on) ledcWrite(motorPin, pwm); else ledcWrite(motorPin, 0);

    // motor_speed_on = !motor_speed_on;
    // digitalWrite(motorPin, motor_user_on && motor_speed_on  ? HIGH : LOW);

    unsigned long now = millis();
    if (now - lastRpmSendTime >= 2000) {
      lastRpmSendTime = now;  // actualiza el marcador

      // Prepara el buffer
      char rpmBuf[32];
      int len = snprintf(rpmBuf, sizeof(rpmBuf), "RPM: %.1f, PWM: %d", rpm, pwm);
      if(motor_user_on) Serial.println(rpmBuf);
      // Envío vía BLE-UART (o SerialBT si usas Classic)
      if (deviceConnected) {
        pTxCharacteristic->setValue((uint8_t*)rpmBuf, len);
        pTxCharacteristic->notify();
      }
    }
    
    if (scroll){
      if(flipHorizontal) offset -= offset_step; else offset += offset_step;
    }
    // 1) Solo actualizamos la ventana cuando lleguen a 0
    if (++scrollCounter >= SCROLL_INTERVAL_REVS) {
     scrollCounter = 0;
     offset = 0;
     updateWindowScroll();
    }
    // === COMPUTE NEW TIMINGS FOR THE REVOLUTION ===
      events[0].timeUs = 0;
      events[0].on = false;
      currentEvent = 1;
      float step = (float)revolutionPeriod / ( 3600.0 * 3 );
        for (u8 i = 0; i < NUM_MIRRORS; i++) {
          u8 row = rowMap[i];  // ya mapea flipVertical
          for (u8 j = 0; j < num_chars; j++){
            for (u8 k = 0; k < NUM_BITS; k++){
              u8 bitIdx = bitMap[k];            // ya mapea Horizontal
              rowmask_t mask = 1 << bitIdx;
              u8 j_inv = (num_chars - 1) - j;
              bool bitIsOne = (pgm_read_byte(&windowBitmap[flipHorizontal? j_inv : j][row]) & mask) != 0;
              if (bitIsOne) {
                events[currentEvent].timeUs = (u32)((pgm_read_word(&startAngleTenths[i]) + j * (widthCharTenths + widthSpaceTenths) + k * widthBitTenths + base_offset - offset) * step);
                events[currentEvent].on = true;
                currentEvent++;
                events[currentEvent].timeUs = (u32)((pgm_read_word(&startAngleTenths[i]) + j * (widthCharTenths + widthSpaceTenths) + widthBitTenths * (k + 1) + base_offset - offset) * step);
                events[currentEvent].on = false;
                currentEvent++;
              }
            }
          }
        }
      qsort(events, currentEvent, sizeof(events[0]), compareEvents);    
      // Filtrar: para cada grupo de eventos con el mismo timeUs,
      // si hay al menos un ON, se descartan todos los OFF de ese tiempo.
      u16 writeIdx = 0;
      u16 readIdx  = 0;
      while (readIdx < currentEvent) {
        u32 t = events[readIdx].timeUs;
        // 1) Averiguar si hay algún ON en este grupo
        bool hasOn = false;
        u32 endIdx = readIdx;
        while (endIdx < currentEvent && events[endIdx].timeUs == t) {
          if (events[endIdx].on) hasOn = true;
          endIdx++;
        }
        // 2) Copiar sólo lo que corresponda
        if (hasOn) {
          // copiar únicamente los ON
          for (u32 k = readIdx; k < endIdx; k++) {
            if (events[k].on) {
              events[writeIdx++] = events[k];
            }
          }
        } else {
          // no había ON, copiar todos (OFF)
          for (u32 k = readIdx; k < endIdx; k++) {
            events[writeIdx++] = events[k];
          }
        }
        // avanza al siguiente grupo de timeUs
        readIdx = endIdx;
      }
    eventCount = writeIdx;   // <- ¡guárdalo!
      //currentEvent = writeIdx;
      currentEvent = 0;
    }
  //unsigned long t = micros() - revolutionStart;
  //bool laserOn = false;
  if (currentEvent < eventCount && t >= events[currentEvent].timeUs) {
    if (events[currentEvent].on) {
      LASER_ON();
    } else {
      LASER_OFF();
    }
    currentEvent++;
  }
  // Lectura del serial
  static uint32_t lastSerialPoll = 0;
  uint32_t nowMs = millis();
  // 1) Sólo pollear el Serial cada 100 ms
  if (nowMs - lastSerialPoll >= SERIAL_POLL_INTERVAL_MS) {
    lastSerialPoll = nowMs;
    pollSerial();
  }
  // 1) Si hay mensaje por USB-Serial
  if (msgPending) {
  // 1) copia del buffer
    char buf[MAX_MESSAGE_LEN + 1];
    strncpy(buf, serialBuf, sizeof(buf));
    buf[sizeof(buf)-1] = '\0';

    bool sawCommand = false;
    const char* sep = " ,;"; 
    for (char* token = strtok(buf, sep); token; token = strtok(nullptr, sep)) {
      // 2) identifica y parsea con atof
      if (strncmp(token, "Kp=", 3) == 0) {
        Kp = atof(token + 3);
        sawCommand = true;
        Serial.print(F("Nuevo Kp = "));
        Serial.println(Kp, 6);        // muestra hasta 6 decimales
      }
      else if (strncmp(token, "Ki=", 3) == 0) {
        Ki = atof(token + 3);
        sawCommand = true;
        Serial.print(F("Nuevo Ki = "));
        Serial.println(Ki, 6);
      }
      else if (strncmp(token, "Kd=", 3) == 0) {
        Kd = atof(token + 3);
        sawCommand = true;
        Serial.print(F("Nuevo Kd = "));
        Serial.println(Kd, 6);
      }
      else if (strncmp(token, "rpm", 3) == 0) {
        setpointRPM = atof(token + 3);
        sawCommand = true;
        Serial.print(F("Nuevo RPM = "));
        Serial.println(setpointRPM, 6);
      }
    }
    if (sawCommand) {
      // Mostrar el estado completo tras aplicar cambios
      Serial.print(F("Estado PID → Kp="));
      Serial.print(Kp, 6);
      Serial.print(F("  Ki="));
      Serial.print(Ki, 6);
      Serial.print(F("  Kd="));
      Serial.println(Kd, 6);
    } else {
      setMessage(serialBuf);
    }      

    msgPending = false;
  }


  // blackButton.update();
  // if (blackButton.isLongClick()) blackPressHandle();
  // if (blackButton.isDoubleClick()) blackDoublePressHandle();
  // yellowButton.update();
  // if (yellowButton.isLongClick()) yellowPressHandle();
  buttonRST.update();
  if (buttonRST.isSingleClick()) rstSingleClickHandle();
  if (buttonRST.isLongClick()) rstPressHandle();
  if (buttonRST.isDoubleClick()) rstDoublePressHandle();
}

void TaskLaserText(void * parameter) {
for (;;) {
  LaserText();

}
vTaskDelay(1);

}


// -------------------- setup() y loop() principal --------------------
void setup() {
  Serial.begin(115200);
  // setCpuFrequencyMhz(240);            // Requiere <Arduino.h>
  
  pinMode(0, INPUT_PULLUP);
  pinMode(26, OUTPUT);
  // pinMode(TFT_VCC, OUTPUT);
  // digitalWrite(TFT_GND, LOW);
  digitalWrite(26, HIGH);
  SPI.begin();
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  
  drawStaticInfo();
  


// --------------- Laser Text Setup --------------------------------
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);

  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(encoderVCC, OUTPUT);
  digitalWrite(encoderVCC, HIGH);
  pinMode(encoderGND, OUTPUT);
  digitalWrite(encoderGND, LOW);
// === BLE START ===
  pinMode(buttonRSTPin, INPUT_PULLUP);
  //pinMode(motorPin, OUTPUT);
  Serial.print("PWM MOTOR: ");
  Serial.println(ledcAttach(motorPin, 500, 8));
  ledcWrite(motorPin, 0);

  pinMode(encoderPin, INPUT_PULLUP);

  BLEDevice::init("ESP32-Laser-BLE");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(UART_SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    UART_CHAR_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    UART_CHAR_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();
  BLEAdvertising *pAdv = pServer->getAdvertising();
  pAdv->addServiceUUID(UART_SERVICE_UUID);
  pAdv->start();

  // pinMode(buttonYellowPin, INPUT_PULLDOWN);
  // pinMode(buttonBlackPin, INPUT_PULLDOWN);
  // pinMode(ledPin, OUTPUT);
  // → Inicializa flip según algún pin o valor fijo
  flipHorizontal = FLIP_HORIZONTAL;
  flipVertical   = FLIP_VERTICAL;
  // Fill rowMap: si flipVertical invierte el orden de los espejos
  for (u8 i = 0; i < NUM_MIRRORS; i++) {
    rowMap[i] = flipVertical
      ? (NUM_MIRRORS - 1 - i)
      : i;
  }
  // Fill bitMap: si flipHorizontal invierte los bits de cada carácter
  for (u8 k = 0; k < NUM_BITS; k++) {
    bitMap[k] = flipHorizontal ? k : (NUM_BITS - 1 - k);  // MSB primero
  }
  setMessage("HOLA MUNDO");

  attachInterrupt(digitalPinToInterrupt(encoderPin), sensorISR, FALLING);
  revolutionStart = micros();


  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  
  bufferMutex = xSemaphoreCreateMutex();
  delay(1000);
  if (digitalRead(0)){
  // Crea las tareas en núcleos separados:
  // TaskSampling en núcleo 0, TaskProcessing en núcleo 1.
  xTaskCreate(TaskSampling, "TaskSampling", 2048, NULL, 2, NULL);
  xTaskCreate(TaskProcessing, "TaskProcessing", 8192, NULL, 2, NULL);
  // xTaskCreate(TaskLaserText, "TaskLaserText", 8192, NULL, 2, NULL);
  } else {
      mode = 1;
  }

}

void loop() {
if (mode == 1){
LaserText();
  }
}
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "arduinoFFT.h"
#include <math.h>

// -------------------- Definiciones de pines y parámetros --------------------
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   5    // Se definen antes de usarlos
#define TFT_DC   19
#define TFT_RST  26
#define TFT_GND  33
#define TFT_VCC  14
#define MICRO_PIN 39

#define GAIN_PIN 13

#define MAX_BANDS 16         // Número máximo de bandas que permitiremos
#define NUM_BANDS 8

#define SAMPLES 256          
#define SAMPLING_FREQUENCY 40000 // Hz (Nyquist = 20000 Hz)
//#define amplitude 200           // Factor de escala

// -------------------- Estado del botón BOOT (GPIO 0) ---------------
volatile bool bootButtonState = true;
bool prev_boton = true;

// Layout de la pantalla (rotada 90°: 320x240)
#define GRAPH_WIDTH 320                
#define INFO_WIDTH 100
#define SPECTRUM_WIDTH (GRAPH_WIDTH - INFO_WIDTH) // Área para las barras
#define GRAPH_Y 30                     // 30 px en la parte superior
#define INFO_Y 225                     // Zona de información inicia en y = 225
#define GRAPH_HEIGHT (220 - GRAPH_Y)   // 220 - 30 = 190 px
#define BAR_GAP 4

// Límites manuales para cada banda (en Hz)
const float silvar[NUM_BANDS] = {20, 60, 250, 500, 2000, 6000, 10000, 15000};
const float normal[NUM_BANDS] = {200, 500, 1000, 2000, 5000, 10000, 15000, 20000};
float bandLimits[NUM_BANDS] =  {200, 500, 1000, 2000, 5000, 10000, 15000, 20000};

/*-----------Defines for automatic gain control------*/
#define GAIN GAIN_PIN
int amplitude[3] = { 200, 200, 200 };  // Depending on your audio source level, you may need to increase this value
int filter[3] = { 2000, 2000, 5000 }; // To filter noise. The last one is the most important since it corresponds to the higher gain mode
int mode_index = 0; // indicates the gain mode of the microphone | 0 : 40 dB | 1 : 50 dB | 2 : 60 dB |
const char *gain_text[3] = {"40dB", "50dB", "60dB"};
unsigned long lastTimeGainLowered = 0;
unsigned long delayToLowerGainAgain = 200;
bool bandSaturated[NUM_BANDS] = { false }; // array to store if the bands are saturated to decrease the gain if so
int numberOfTimesSaturated = 0;  // number of saturations in periodForSaturation time
int saturationThreshold = 3; // maximum allowed saturations in periodForSaturation time
unsigned long periodForSaturation = 80;  // each period the number of times saturated resets to 0
unsigned long lastTimeForSaturation = 0; //
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
// Usa float (4 bytes) y fuerza a SRAM interna
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
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

/////////////////////////////////////////////////////////////////////////

//gain to 40dB
void check_gain() {
  switch(mode_index) {
    case 0:
      gain_high();  // Configura para 40dB
      break;
    case 1:
      gain_low();   // Configura para 50dB
      break;
    case 2:
      gain_float(); // Configura para 60dB (entrada flotante)
      break;
    default:
      // Si mode_index no coincide con ninguno, se puede configurar un modo por defecto
      gain_float();
      break;
  }
}




void gain_high(void) {
  pinMode(GAIN, OUTPUT);
  digitalWrite(GAIN, HIGH);  // Configures the GAIN pin to VCC
}

//gain to 50dB
void gain_low(void) {
  pinMode(GAIN, OUTPUT);
  digitalWrite(GAIN, LOW);  // Configures the GAIN pin to GND
}

//gain to 60dB
void gain_float(void) {
  pinMode(GAIN, INPUT); // Configures the GAIN pin as INPUT. This way, the internal pull-up resistor disconects, leaving the pin floating
}
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
  drawFrequencyLabels();
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

const int numOfHistoryValues = 11;  // Usamos 11 puntos de control (como en tu código original)
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
      sprintf(buf, "%.0f", f_max);
    else
      sprintf(buf, "%.1f", f_max / 1000.0);
    int xPos = j * bandWidth + (bandWidth / 2) - 10;
    tft.setCursor(xPos, INFO_Y + 2);
    tft.print(buf);
  }
}

void drawStaticInfo() {
  drawFrequencyLabels();
  tft.setCursor(SPECTRUM_WIDTH, 0);
  tft.print("Tp:        ms");
  tft.setCursor(SPECTRUM_WIDTH, 10);
  tft.print("Ts:        ms");
  tft.setCursor(SPECTRUM_WIDTH, 20);
  tft.print("FPS:      ");
  tft.setCursor(SPECTRUM_WIDTH, 30);
  tft.print("Gain:      ");
  tft.setCursor(SPECTRUM_WIDTH, 40);
  tft.print("mode_index:      "); 
  tft.setCursor(SPECTRUM_WIDTH, 50);
  tft.print("saturations:       "); 
}

// -------------------- Tareas FreeRTOS --------------------

// Tarea de muestreo (núcleo 0)
void TaskSampling(void * parameter) {
  for (;;) {

    int *buf = (currentBuffer == 0) ? buffer0 : buffer1;

    for (int i = 0; i < SAMPLES; i++) {
      newTime = micros() - oldTime;
      oldTime = newTime;
      buf[i] = analogRead(MICRO_PIN);
      while (micros() < (newTime + sampling_period_us)) { }
    }
    xSemaphoreTake(bufferMutex, portMAX_DELAY);
    bufferReady[currentBuffer] = true;
    currentBuffer = (1 - currentBuffer);
    xSemaphoreGive(bufferMutex);

    samplingTime = micros() - lastSamplingTime;
    lastSamplingTime = micros();

    vTaskDelay(1);
  }
}

// Tarea de procesamiento (núcleo 1)
void TaskProcessing(void * parameter) {
  for (;;) {
    int localBuffer[SAMPLES];
    if (bufferReady[0] || bufferReady[1]) {

      unsigned long procStart = micros();

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
        if (vReal[i] > filter[mode_index]) {
          int value = (int)vReal[i] / amplitude[mode_index];
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
      
      for (int i = 0; i < NUM_BANDS; i++) {
        updateControlPoints(bandValues[i], i);
        bandValues[i] = getSmoothedValue(i, 0.2);
      }
      // === DRAWING BANDS ===
      int bandWidth = SPECTRUM_WIDTH / NUM_BANDS;
      for (int band = 0; band < NUM_BANDS; band++) {
        int dsize = bandValues[band];
        if (dsize > 50) {
          dsize = 50;
          numberOfTimesSaturated++;
          tft.fillRect(SPECTRUM_WIDTH + 75, 50, 20, 8, ST77XX_BLACK);
          tft.setCursor(SPECTRUM_WIDTH + 75, 50);
          tft.print(numberOfTimesSaturated, 1);           
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


      int prev_index = mode_index;
      // decrease the gain if the audio is saturated and restart saturation count
      if (numberOfTimesSaturated > saturationThreshold) {
        mode_index = max(mode_index - 1, 0);
        check_gain();          
  
        numberOfTimesSaturated = 0;
        lastTimeForSaturation = millis();
          tft.fillRect(SPECTRUM_WIDTH + 75, 50, 20, 8, ST77XX_BLACK);
          tft.setCursor(SPECTRUM_WIDTH + 75, 50);
          tft.print(numberOfTimesSaturated, 1);     
      } /* else {
        mode_index = min(mode_index + 1, 2);
        check_gain();
      }
      if (prev_index != mode_index) {
        tft.fillRect(SPECTRUM_WIDTH + 70, 40, 10, 8, ST77XX_BLACK);
        tft.setCursor(SPECTRUM_WIDTH + 70, 40);
        tft.print(mode_index, 1);           
      } 
      
      */
      // increase the gain if there is no audio detected
      unsigned long currentTimeGain = millis();
      if ((currentTimeGain - lastTimeGainLowered) > delayToLowerGainAgain) {
            mode_index = min(mode_index + 1, 2);
            check_gain();
            lastTimeGainLowered = currentTimeGain;
      }
      if (prev_index != mode_index) {
        tft.fillRect(SPECTRUM_WIDTH + 40, 30, 50, 8, ST77XX_BLACK);
        tft.setCursor(SPECTRUM_WIDTH + 40, 30);
        tft.print(gain_text[mode_index]);    
        tft.fillRect(SPECTRUM_WIDTH + 70, 40, 10, 8, ST77XX_BLACK);
        tft.setCursor(SPECTRUM_WIDTH + 70, 40);
        tft.print(mode_index, 1);           
      } 
      // Reset the numberOfTimesSaturated periodically
      unsigned long currentTime = millis();
      if ((currentTime - lastTimeForSaturation) > periodForSaturation) {
        numberOfTimesSaturated = 0;
        lastTimeForSaturation = currentTime;
      }

        bootButtonState = digitalRead(0); // LOW si está presionado
        if (bootButtonState == false && bootButtonState != prev_boton) {
          buttonPressed();
        } else if (bootButtonState == true && bootButtonState != prev_boton) {
          buttonReleased();
        }
        prev_boton = bootButtonState;

            
      unsigned long procCycle = micros() - procStart;

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
      float T_ms = nowProcesingTime / 1000.0; 

      tft.fillRect(SPECTRUM_WIDTH + 40, 0, 25, 8, ST77XX_BLACK);
      tft.setCursor(SPECTRUM_WIDTH + 40, 0);
      tft.print(T_ms, 1);

      float localSamplingTime = samplingTime / 1000.0;
      
      tft.fillRect(SPECTRUM_WIDTH + 40, 10, 25, 8, ST77XX_BLACK);
      tft.setCursor(SPECTRUM_WIDTH + 40, 10);
      tft.print(localSamplingTime, 1);

      tft.fillRect(SPECTRUM_WIDTH + 40, 20, 25, 8, ST77XX_BLACK);
      tft.setCursor(SPECTRUM_WIDTH + 40, 20);
      tft.print(fps, 1);
      /*
      tft.fillRect(SPECTRUM_WIDTH + 40, 30, 30, 8, ST77XX_BLACK);
      tft.setCursor(SPECTRUM_WIDTH + 40, 30);
      tft.print(gain_text[mode_index]);

      tft.fillRect(SPECTRUM_WIDTH + 70, 40, 10, 8, ST77XX_BLACK);
      tft.setCursor(SPECTRUM_WIDTH + 70, 40);
      tft.print(mode_index, 1);  
      */          
      
      }



    } else {
      vTaskDelay(1);
    }
  }
}

// -------------------- setup() y loop() principal --------------------
void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);            // Requiere <Arduino.h>
  
  gain_low();

  pinMode(0, INPUT_PULLUP);
  pinMode(TFT_GND, OUTPUT);
  pinMode(TFT_VCC, OUTPUT);
  digitalWrite(TFT_GND, LOW);
  digitalWrite(TFT_VCC, HIGH);
    // SPI a 40 MHz
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  SPI.setFrequency(27000000);         // 27 MHz si tu cableado es largo
  tft.init(240, 320);
  tft.setSPISpeed(27000000);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  
  drawStaticInfo();
  
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  
  bufferMutex = xSemaphoreCreateMutex();
  
  // Crea las tareas en núcleos separados:
  // TaskSampling en núcleo 0, TaskProcessing en núcleo 1.
  xTaskCreate(TaskSampling, "TaskSampling", 2048, NULL, 2, NULL);
  xTaskCreate(TaskProcessing, "TaskProcessing", 8192, NULL, 2, NULL);
}

void loop() {
  // El loop principal queda vacío, ya que todas las tareas se encargan del trabajo.
}

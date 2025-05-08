// Laser Control for ESP32 xd
#include "config.h"

volatile unsigned long revolutionStart  = 0;
volatile unsigned long revolutionPeriod = 0;
volatile u8 revCount = 0;
volatile unsigned long period = 0;
volatile unsigned long sum = 0;


volatile bool newRevolution = false;
volatile bool laser_user_on = false;
volatile bool laser_speed_on = false;
volatile bool motor_user_on = false;

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

// Máximo número de caracteres en el mensaje

// Variables globales
#define MAX_MESSAGE_LEN 100
int  messageLen = 0;  
rowmask_t messageBitmap[MAX_MESSAGE_LEN][NUM_MIRRORS];
char     fullMessage[MAX_MESSAGE_LEN+1];
uint8_t  windowStart = 0;
rowmask_t windowBitmap[MAX_NUM_CHARS][NUM_MIRRORS];
// Convierte 'A'–'Z' o 'a'–'z' a índice 0–25, y cualquier otro a índice 26 (espacio)
uint8_t charToIndex(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a';
  if (c == '<')             return 26;   // <-- corazón
  if (c == '>')             return 27;
  return 28;  // índice 27 para espacio o carácter no válido
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



// Llamar *una sola* vez por iteración de loop():
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

void updateWindowScroll() {
  for (uint8_t j = 0; j < num_chars; j++) {
    uint8_t idx = (windowStart + j) % messageLen;
    uint8_t alpha = charToIndex(fullMessage[idx]);
    for (uint8_t i = 0; i < NUM_MIRRORS; i++) {
      windowBitmap[j][i] = (alpha < 28)
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

void blackPressHandle(){
  num_chars = (num_chars + 1) % MAX_NUM_CHARS;
  widthCharTenths  = widthLineTenths / (num_chars + 1);
  widthBitTenths   = round(widthCharTenths / NUM_BITS);
  widthSpaceTenths = widthCharTenths / (num_chars - 1);
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
  widthSpaceTenths = widthCharTenths / (num_chars - 1);
  
  Serial.print("Button pressed! num_chars = ");
  Serial.println(num_chars);
}
void rstSingleClickHandle(){
  scroll = !scroll;
  Serial.println("boton rst presionado");
}
void rstPressHandle(){
  motor_user_on = !motor_user_on;
  digitalWrite(motorPin, motor_user_on ? HIGH : LOW);
  Serial.println("boton rst long presionado");
}
void rstDoublePressHandle(){
  laser_user_on = !laser_user_on;
  if (laser_user_on) LASER_ON(); else LASER_OFF();
  Serial.println("boton rst doble presionado");
}

void setup() {
  Serial.begin(115200);
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);

  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(encoderVCC, OUTPUT);
  digitalWrite(encoderVCC, HIGH);
  pinMode(encoderGND, OUTPUT);
  digitalWrite(encoderGND, LOW);

  pinMode(buttonRSTPin, INPUT_PULLUP);
  pinMode(motorPin, OUTPUT);
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
}

void loop() {
  unsigned long t = micros() - revolutionStart;
  if (newRevolution){
    laser_speed_on = revolutionPeriod < 30000;
    //Serial.println(revolutionPeriod);
    newRevolution = false;
    LASER_OFF();
    // 1) Solo actualizamos la ventana cuando lleguen a 0
    if (++scrollCounter >= SCROLL_INTERVAL_REVS) {
     scrollCounter = 0;
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
              u8 bitIdx = bitMap[k];            // ya mapea flipHorizontal
              rowmask_t mask = 1 << bitIdx;
              u8 j_inv = (num_chars - 1) - j;
              bool bitIsOne = (pgm_read_byte(&windowBitmap[j_inv][row]) & mask) != 0;
              if (bitIsOne) {
                startBit[k][j][i] = (u32)((pgm_read_word(&startAngleTenths[i]) + j * (widthCharTenths + widthSpaceTenths) + k * widthBitTenths + offset) * step);
                endBit[k][j][i] = (u32)((pgm_read_word(&startAngleTenths[i]) + j * (widthCharTenths + widthSpaceTenths) + widthBitTenths * (k + 1) + offset) * step);
                events[currentEvent].timeUs = startBit[k][j][i];
                events[currentEvent].on = true;
                currentEvent++;
                events[currentEvent].timeUs = endBit[k][j][i];
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
      currentEvent = writeIdx;
      currentEvent = 0;
    }
  //unsigned long t = micros() - revolutionStart;
  //bool laserOn = false;
  if (t >= events[currentEvent].timeUs) {
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
  // 2) Si llegó un mensaje completo, setéalo
  if (msgPending) {
    setMessage(serialBuf);
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

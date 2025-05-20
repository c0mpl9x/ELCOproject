// Laser Control for ESP32 xd
#include "config.h"

volatile unsigned long revolutionStart  = 0;
volatile unsigned long revolutionPeriod = 0;
volatile u8 revCount = 0;
volatile unsigned long period = 0;
volatile unsigned long sum = 0;
static uint8_t SCROLL_INTERVAL_REVS = 15;  // scroll cada 3 revs



BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;


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
      if (b2 == 0xB1 || b2 === 0x91) {     // ñ
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
    widthSpaceTenths = widthCharTenths / (num_chars - 1);
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
// Parsear comando recibido y actualizar estados
void handleCommand(char* cmdBuf) {
  String cmd = String(cmdBuf);
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "MOTOR ON") {
    motor_user_on = true;
    digitalWrite(motorPin, motor_user_on ? HIGH : LOW);
  }
  else if (cmd == "MOTOR OFF") {
    motor_user_on = false;
    digitalWrite(motorPin, motor_user_on ? HIGH : LOW);
  }
  else if (cmd == "MOTOR TOGGLE"){
    motor_user_on = !motor_user_on;
    digitalWrite(motorPin, motor_user_on ? HIGH : LOW);
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
  }
  else if (cmd == "DOWN SCROLL"){
    if(SCROLL_INTERVAL_REVS > 3){
      SCROLL_INTERVAL_REVS--;
    }
  }
  else if (cmd == "UP NUM CHARS"){
    updateNumChars(num_chars + 1);
  }
  else if (cmd == "DOWN NUM CHARS"){
    updateNumChars(num_chars - 1);
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
void setup() {
  Serial.begin(115200);
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);

  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(encoderVCC, OUTPUT);
  digitalWrite(encoderVCC, HIGH);
  pinMode(encoderGND, OUTPUT);
  digitalWrite(encoderGND, LOW);
// === BLE START ===
  pinMode(buttonRSTPin, INPUT_PULLUP);
  pinMode(motorPin, OUTPUT);
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
}

// Variable global (fuera de loop / setup)
static unsigned long lastRpmSendTime = 0;  // registro de la última vez que enviamos

void loop() {
  unsigned long t = micros() - revolutionStart;
  if (newRevolution){
    laser_speed_on = revolutionPeriod < 30000;
    //Serial.println(revolutionPeriod);
    newRevolution = false;
    LASER_OFF();
    // 1) Calcula RPM: 60s / (periodo en s)
    //    revolutionPeriod µs → revolutionPeriod/1e6 s
    float rpm = 60.0f * 1e6f / (float)revolutionPeriod;

    // 2) Prepara el mensaje
    char rpmBuf[32];
    int len = snprintf(rpmBuf, sizeof(rpmBuf), "RPM: %.1f\n", rpm);

    // 3) Envíalo vía BLE-UART (si está conectado)
        // Solo envía si ha pasado >= 1000 ms desde el último envío
    unsigned long now = millis();
    if (now - lastRpmSendTime >= 1000) {
      lastRpmSendTime = now;  // actualiza el marcador

      // Prepara el buffer
      char rpmBuf[32];
      int len = snprintf(rpmBuf, sizeof(rpmBuf), "RPM: %.1f", rpm);

      // Envío vía BLE-UART (o SerialBT si usas Classic)
      if (deviceConnected) {
        pTxCharacteristic->setValue((uint8_t*)rpmBuf, len);
        pTxCharacteristic->notify();
      }
    }

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

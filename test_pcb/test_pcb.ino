const int laserPin = 25;
const int gndPin = 15;
const int vccPin = 12;
const int encoderPin = 13;
const int motorPin = 32;

const int J3pinDOT = 33;
const int J6pinDOT = 25;
const int J6pinNOdot = 26;
const int JTAGDot = 14;
const int JTAG2 = 12;
const int JTAG3 = 13;
const int JTAG4 = 15;

#define NUM_BOTONES 7
const int botonPin[NUM_BOTONES] = {0, JTAGDot, JTAG2, JTAG3, JTAG4, J6pinNOdot, J3pinDOT};
int boton[NUM_BOTONES];
unsigned long T = 2000; // Milliseconds
bool motor = false;
void setup() {
  pinMode(laserPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(encoderPin, INPUT);

  for (int i = 0; i < NUM_BOTONES; i++){
    pinMode(botonPin[i], INPUT_PULLUP);
  }


  Serial.begin(115200);
}
bool botonPrev[NUM_BOTONES] = {false};

bool encoderPrev = false;
unsigned long next = 0;
void loop() {
  bool encoder = digitalRead(encoderPin);
  
  if (encoderPrev != encoder) {
    encoderPrev = encoder;
    Serial.print("Encoder: ");
    Serial.println(encoder);
  }
  for (int i = 0; i < NUM_BOTONES; i++){
    boton[i] = !digitalRead(botonPin[i]);
    if(botonPrev[i] != boton[i]){
          botonPrev[i] = boton[i];
          Serial.print("Boton ");
          Serial.print(botonPin[i]);
          Serial.print(": ");
          Serial.println(boton[i]);
    }
  }
  digitalWrite(laserPin, boton[0] ? HIGH : LOW);
  unsigned long now = millis();
  if (now > next) {
    next = now + T;
    motor = !motor;
    digitalWrite(motorPin, motor);
  }
}

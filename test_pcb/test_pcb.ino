const int laserPin = 25;
const int gndPin = 15;
const int vccPin = 12;
const int encoderPin = 13;
const int motorPin = 35;
const int J3pin2 = 32;
const int J3pin1 = 33;
const int J6pinDOT = 25;
const int J6pinNOdot = 26;

const int JTAGDot = 14;
const int JTAG2 = 12;
const int JTAG3 = 13;
const int JTAG4 = 15;

const int botonPin[6] = {0, JTAGDot, JTAG2, JTAG3, JTAG4, J6pinNOdot};
int boton[6];
unsigned long T = 2000; // Milliseconds
bool motor = false;
void setup() {
  pinMode(laserPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(J3pin2, OUTPUT);
  pinMode(J3pin1, OUTPUT);
  pinMode(gndPin, OUTPUT);

  for (int i = 0; i < 6; i++){
    pinMode(botonPin[i], INPUT_PULLUP);
  }

  digitalWrite(gndPin, LOW);
  pinMode(vccPin, OUTPUT);
  digitalWrite(vccPin, HIGH);
  pinMode(encoderPin, INPUT);

  Serial.begin(115200);
}
bool botonPrev[6] = {false};

bool encoderPrev = false;
unsigned long next = 0;
void loop() {
  //bool boton = !digitalRead(0);
  bool encoder = digitalRead(encoderPin);
  digitalWrite(laserPin, boton ? HIGH : LOW);
  if (encoderPrev != encoder) {
    encoderPrev = encoder;
    Serial.print("Encoder: ");
    Serial.println(encoder);
  }
  if (botonPrev != boton) {
    botonPrev = boton;
    Serial.print("Boton: ");
    Serial.println(boton);
  }
  for (int i = 0; i < 6; i++){
    boton[i] digitalRead(botonPin[i]);
    if(botonPrev[i] != boton[i]){
          botonPrev[i] = boton[i];
          Serial.print("Boton: ");
          Serial.println(boton);
    }
  }
  unsigned long now = millis();
  if (now > next) {
    next = now + T;
    motor = !motor;
    digitalWrite(motorPin, motor);
    digitalWrite(J3pin1, motor);
    digitalWrite(J3pin2, motor);
    Serial.print("Motor: "); Serial.println(motor);
  }
}

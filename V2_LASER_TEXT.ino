#include <Arduino.h>
// #include <PGMWrap.h>
#include <string.h>  // Para usar strlen

// Se elimina SoftwareSerial y librerías de AVR para ESP32.
// Se usa un puerto serial hardware (Serial1) asignado a pines 8 (RX) y 9 (TX).
HardwareSerial mySerial(1);

// Pines asignados (ajusta según tu placa ESP32)
const int mirrorPin = 3; // Sensor contador de espejos
const int revPin    = 2; // Sensor de revolución completa
const int laserPin  = 6; // Salida del láser

// Macros de retardo. (En ESP32, los ciclos nop serán mucho más rápidos)
#define NOP asm volatile ("nop")
#define LetterSpace { NOP; NOP; NOP; NOP; NOP; }

// En ESP32 no se necesita pgm_read_dword_near; se define para hacer una desreferenciación directa:
#ifndef pgm_read_dword_near
#define pgm_read_dword_near(x) (*(x))
#endif

volatile int mirrorFlag = 0;      // Cuenta cuántos espejos han pasado por el sensor inferior
int mirrorFlagOld = 0;            // Almacena el valor anterior del contador

int centerVal = 1800;             // Ajuste izquierda/derecha (puede requerir tuning en ESP32)
int avgWidth  = 36;               // Ancho promedio (ajustar según sea necesario)

const byte numChars = 32;         // Número máximo de caracteres
char myData[numChars];            // Cadena para almacenar datos recibidos por serial
boolean newData = false;          // Indicador de nuevos datos

const int rows = 12;              // Número de filas (debe ser igual al número de espejos)
const int columns = 9;            // Número de segmentos de línea

int mCount = 0;                   // Contador de la longitud del mensaje

// *****************************************************************
// DEFINICIÓN DE LOS CARACTERES (MATRICES de 12x8)
// Se conservan los arreglos originales con PROGMEM. En ESP32 es soportado
// aunque en muchos casos no es necesario. Puedes eliminarlos si lo prefieres.
// *****************************************************************

//These are the Characters that can be displayed represented as 2D array
const int space[ rows ][ columns ] = { //Space bar
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0}
};

const int a[ rows ][ columns ] = { //A
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1}
};

const int b[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 1, 1, 1, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 0, 0, 1, 1, 1, 0},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 0}
};

const int c[ rows ][ columns ] = {
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 0, 0, 0, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0}
};

const int d[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 1, 1, 1, 0, 0}
};

const int e[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1}
};

const int f[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0}
};

const int g[ rows ][ columns ] = {
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 1, 1, 1, 1},
{1, 1, 0, 0, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0}
};

const int h[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1}
};

const int i[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1}
};

const int j[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 0, 0, 0, 1, 1, 0, 0},
{0, 0, 0, 0, 1, 1, 0, 0},
{0, 0, 0, 0, 1, 1, 0, 0},
{0, 0, 0, 0, 1, 1, 0, 0},
{0, 0, 0, 0, 1, 1, 0, 0},
{0, 0, 0, 0, 1, 1, 0, 0},
{0, 0, 0, 0, 1, 1, 0, 0},
{1, 1, 0, 0, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 0, 0, 0}
};

const int k[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 1, 1, 1, 0},
{1, 1, 0, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 0, 0, 0},
{1, 1, 1, 1, 0, 0, 0, 0},
{1, 1, 1, 1, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 0, 0, 0},
{1, 1, 0, 1, 1, 1, 0, 0},
{1, 1, 0, 0, 1, 1, 1, 0},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1}
};

const int l[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1}
};

const int m[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1}
};

const int n[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 0, 1, 1},
{1, 1, 1, 1, 0, 0, 1, 1},
{1, 1, 1, 1, 0, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 0, 1, 1, 1, 1},
{1, 1, 0, 0, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1}
};

const int o[ rows ][ columns ] = {
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0},
{0, 0, 1, 1, 1, 1, 0, 0}
};

const int p[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0}
};

const int q[ rows ][ columns ] = {
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 1, 1, 1},
{1, 1, 0, 0, 1, 1, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 0},
{0, 1, 1, 1, 1, 1, 1, 1},
{0, 0, 1, 1, 1, 0, 1, 1}
};

const int r[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 0, 0, 0},
{1, 1, 0, 1, 1, 1, 0, 0},
{1, 1, 0, 0, 1, 1, 1, 0},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1}
};

const int s[ rows ][ columns ] = {
{0, 0, 1, 1, 1, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{0, 0, 1, 1, 1, 1, 1, 1},
{0, 0, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 1, 1, 1, 0, 0}
};

const int t[ rows ][ columns ] = {
 {1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0}
};

const int u[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0}
};

const int v[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{0, 1, 1, 0, 0, 1, 1, 0},
{0, 1, 1, 0, 0, 1, 1, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0}
};

const int w[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0}
};

const int x[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0},
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1}
};

const int y[ rows ][ columns ] = {
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0},
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0}
};

const int z[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 1, 1, 1, 0},
{0, 0, 0, 1, 1, 1, 0, 0},
{0, 0, 1, 1, 1, 0, 0, 0},
{0, 1, 1, 1, 0, 0, 0, 0},
{1, 1, 1, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1}
};

const int zero[ rows ][ columns ] = {
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 1, 1, 1, 1},
{1, 1, 0, 0, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 1, 0, 1, 1},
{1, 1, 0, 1, 0, 0, 1, 1},
{1, 1, 1, 1, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0},
{0, 0, 1, 1, 1, 1, 0, 0}
};

const int one[ rows ][ columns ] = {
{0, 0, 0, 0, 1, 1, 1, 1},
{0, 0, 0, 0, 1, 1, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1}
};

const int two[ rows ][ columns ] = {
{0, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 1, 1, 1, 0},
{0, 0, 0, 1, 1, 1, 0, 0},
{0, 0, 1, 1, 1, 0, 0, 0},
{0, 1, 1, 1, 0, 0, 0, 0},
{1, 1, 1, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1}
};

const int three[ rows ][ columns ] = {
{0, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 1, 1, 1, 0},
{0, 0, 0, 1, 1, 1, 0, 0},
{0, 0, 0, 1, 1, 1, 0, 0},
{0, 0, 0, 0, 1, 1, 1, 0},
{0, 0, 0, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 0},
{0, 1, 1, 1, 1, 1, 0, 0}
};

const int four[ rows ][ columns ] = {
{0, 0, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 1, 1, 1, 1},
{0, 0, 0, 1, 1, 1, 1, 1},
{0, 0, 1, 1, 1, 0, 1, 1},
{0, 1, 1, 1, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 0, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1}
};

const int five[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 1, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 0},
{0, 0, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 0},
{0, 1, 1, 1, 1, 1, 0, 0}
};

const int six[ rows ][ columns ] = {
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 1, 1, 1, 0, 0},
{1, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0},
{0, 0, 1, 1, 1, 1, 0, 0}
};

const int seven[ rows ][ columns ] = {
{1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 1, 1, 1, 0},
{0, 0, 0, 1, 1, 1, 0, 0},
{0, 0, 1, 1, 1, 0, 0, 0},
{0, 1, 1, 1, 0, 0, 0, 0},
{1, 1, 1, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0}
};

const int eight[ rows ][ columns ] = {
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{0, 1, 1, 0, 0, 1, 1, 0},
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 0, 0, 1, 1, 0},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0}
};

const int nine[ rows ][ columns ] = {
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{1, 1, 1, 0, 0, 1, 1, 1},
{0, 1, 1, 1, 1, 1, 1, 0},
{0, 0, 0, 1, 1, 1, 1, 0},
{0, 0, 0, 1, 1, 1, 0, 0},
{0, 0, 1, 1, 1, 0, 0, 0},
{0, 1, 1, 1, 0, 0, 0, 0},
{1, 1, 1, 0, 0, 0, 0, 0}
};

const int per[ rows ][ columns ] = {  //period
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0}
};

const int comma[ rows ][ columns ] = {  //period
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 1, 1, 0, 0, 0, 0},
{0, 1, 1, 0, 0, 0, 0, 0},
{0, 1, 0, 0, 0, 0, 0, 0}
};

const int sc[ rows ][ columns ] = { //semicolon
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0}
};

const int exc[ rows ][ columns ] = { //exclamation
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0}
};

const int ques[ rows ][ columns ] = { //question mark
{0, 0, 1, 1, 1, 1, 0, 0},
{0, 1, 1, 1, 1, 1, 1, 0},
{1, 1, 1, 0, 0, 1, 1, 1},
{1, 1, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 1, 1, 1},
{0, 0, 0, 0, 1, 1, 1, 0},
{0, 0, 0, 1, 1, 1, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0},
{0, 0, 0, 1, 1, 0, 0, 0}
};

const int checker[ rows ][ columns ] = { //question mark
{1, 0, 1, 0, 1, 0, 1, 0},
{0, 1, 0, 1, 0, 1, 0, 1},
{1, 0, 1, 0, 1, 0, 1, 0},
{0, 1, 0, 1, 0, 1, 0, 1},
{1, 0, 1, 0, 1, 0, 1, 0},
{0, 1, 0, 1, 0, 1, 0, 1},
{1, 0, 1, 0, 1, 0, 1, 0},
{0, 1, 0, 1, 0, 1, 0, 1},
{1, 0, 1, 0, 1, 0, 1, 0},
{0, 1, 0, 1, 0, 1, 0, 1},
{1, 0, 1, 0, 1, 0, 1, 0},
{0, 1, 0, 1, 0, 1, 0, 1}
};

const int block[ rows ][ columns ] = { //question mark
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1}
};
// Para efectos de este ejemplo, se asume que todos los arrays están definidos de forma similar.

int pulses[rows][columns];  // Matriz para almacenar pulsos

// *****************************************************************
// FUNCIONES DE CONFIGURACIÓN E INTERRUPCIONES
// *****************************************************************

void IRAM_ATTR ISR_mirror() {
  mirrorFlag++;
}

void IRAM_ATTR ISR_rev() {
  mirrorFlag = 0;
}

void setup() {
  // Inicializa el serial USB para depuración y el serial hardware para otros fines.
  Serial.begin(9600);
  mySerial.begin(115200, SERIAL_8N1, 8, 9);  // Ajusta los pines RX y TX según tu hardware
  
  // Configura pines
  pinMode(mirrorPin, INPUT_PULLUP);
  pinMode(revPin, INPUT_PULLUP);
  pinMode(laserPin, OUTPUT);
  
  // Configura interrupciones (la función digitalPinToInterrupt funciona en ESP32)
  attachInterrupt(digitalPinToInterrupt(mirrorPin), ISR_mirror, RISING);
  attachInterrupt(digitalPinToInterrupt(revPin), ISR_rev, RISING);
}

void loop() {
  receiveData();
  showNewData();
  determineMirror();
}

// *****************************************************************
// FUNCIONES PRINCIPALES
// *****************************************************************

// Función que determina el retardo basado en la posición del espejo (para alinear el texto)
void determineMirror() {
  if (mirrorFlag != mirrorFlagOld) {
    mirrorFlagOld = mirrorFlag;
    int textWidth = strlen(myData) * avgWidth;
    switch (mirrorFlag) {
      case 1:
        nopTimer(centerVal - textWidth);
        encodePulses(0);
        break;
      case 2:
        nopTimer(49 + centerVal - textWidth);
        encodePulses(1);
        break;
      case 3:
        nopTimer(85 + centerVal - textWidth);
        encodePulses(2);
        break;
      case 4:
        nopTimer(57 + centerVal - textWidth);
        encodePulses(3);
        break;
      case 5:
        nopTimer(-24 + centerVal - textWidth);
        encodePulses(4);
        break;
      case 6:
        nopTimer(9 + centerVal - textWidth);
        encodePulses(5);
        break;
      case 7:
        nopTimer(15 + centerVal - textWidth);
        encodePulses(6);
        break;
      case 8:
        nopTimer(122 + centerVal - textWidth);
        encodePulses(7);
        break;
      case 9:
        nopTimer(58 + centerVal - textWidth);
        encodePulses(8);
        break;
      case 10:
        nopTimer(98 + centerVal - textWidth);
        encodePulses(9);
        break;
      case 11:
        nopTimer(38 + centerVal - textWidth);
        encodePulses(10);
        break;
      case 12:
        nopTimer(10 + centerVal - textWidth);
        encodePulses(11);
        break;
      default:
        break;
    }
  }
}

// Esta función codifica cada pulso en función de los caracteres recibidos.
// Se utilizan llamadas a "digitalWrite" en lugar de manipular registros directamente.
void encodePulses(int mirrorNum) {
  noInterrupts(); // Deshabilita interrupciones para una temporización precisa
  mCount = strlen(myData) - 1;  // Se excluye el carácter final de "ruido"
  
  while (mCount > -1) {
    int segment = 7;
    // Se recorre de derecha a izquierda (según la dirección del giro del motor)
    while (segment > -1) {
      switch (myData[mCount]) {
        case ' ':
          pulses[mirrorNum][segment] = space[mirrorNum][segment];
          break;
        case '/':
          pulses[mirrorNum][segment] = checker[mirrorNum][segment];
          break;
        case '|':
          pulses[mirrorNum][segment] = block[mirrorNum][segment];
          break;
        case 'a':
          pulses[mirrorNum][segment] = a[mirrorNum][segment];
          break;
        case 'b':
          pulses[mirrorNum][segment] = b[mirrorNum][segment];
          break;
        case 'c':
          pulses[mirrorNum][segment] = c[mirrorNum][segment];
          break;
        case 'd':
          pulses[mirrorNum][segment] = d[mirrorNum][segment];
          break;
        case 'e':
          pulses[mirrorNum][segment] = e[mirrorNum][segment];
          break;
        case 'f':
          pulses[mirrorNum][segment] = f[mirrorNum][segment];
          break;
        case 'g':
          pulses[mirrorNum][segment] = g[mirrorNum][segment];
          break;
        case 'h':
          pulses[mirrorNum][segment] = h[mirrorNum][segment];
          break;
        case 'i':
          pulses[mirrorNum][segment] = i[mirrorNum][segment];
          break;
        case 'j':
          pulses[mirrorNum][segment] = j[mirrorNum][segment];
          break;
        case 'k':
          pulses[mirrorNum][segment] = k[mirrorNum][segment];
          break;
        case 'l':
          pulses[mirrorNum][segment] = l[mirrorNum][segment];
          break;
        case 'm':
          pulses[mirrorNum][segment] = m[mirrorNum][segment];
          break;
        case 'n':
          pulses[mirrorNum][segment] = n[mirrorNum][segment];
          break;
        case 'o':
          pulses[mirrorNum][segment] = o[mirrorNum][segment];
          break;
        case 'p':
          pulses[mirrorNum][segment] = p[mirrorNum][segment];
          break;
        case 'q':
          pulses[mirrorNum][segment] = q[mirrorNum][segment];
          break;
        case 'r':
          pulses[mirrorNum][segment] = r[mirrorNum][segment];
          break;
        case 's':
          pulses[mirrorNum][segment] = s[mirrorNum][segment];
          break;
        case 't':
          pulses[mirrorNum][segment] = t[mirrorNum][segment];
          break;
        case 'u':
          pulses[mirrorNum][segment] = u[mirrorNum][segment];
          break;
        case 'v':
          pulses[mirrorNum][segment] = v[mirrorNum][segment];
          break;
        case 'w':
          pulses[mirrorNum][segment] = w[mirrorNum][segment];
          break;
        case 'x':
          pulses[mirrorNum][segment] = x[mirrorNum][segment];
          break;
        case 'y':
          pulses[mirrorNum][segment] = y[mirrorNum][segment];
          break;
        case 'z':
          pulses[mirrorNum][segment] = z[mirrorNum][segment];
          break;
        case '0':
          pulses[mirrorNum][segment] = zero[mirrorNum][segment];
          break;
        case '1':
          pulses[mirrorNum][segment] = one[mirrorNum][segment];
          break;
        case '2':
          pulses[mirrorNum][segment] = two[mirrorNum][segment];
          break;
        case '3':
          pulses[mirrorNum][segment] = three[mirrorNum][segment];
          break;
        case '4':
          pulses[mirrorNum][segment] = four[mirrorNum][segment];
          break;
        case '5':
          pulses[mirrorNum][segment] = five[mirrorNum][segment];
          break;
        case '6':
          pulses[mirrorNum][segment] = six[mirrorNum][segment];
          break;
        case '7':
          pulses[mirrorNum][segment] = seven[mirrorNum][segment];
          break;
        case '8':
          pulses[mirrorNum][segment] = eight[mirrorNum][segment];
          break;
        case '9':
          pulses[mirrorNum][segment] = nine[mirrorNum][segment];
          break;
        case ':':
          pulses[mirrorNum][segment] = sc[mirrorNum][segment];
          break;
        case '.':
          pulses[mirrorNum][segment] = per[mirrorNum][segment];
          break;
       case ',':
          pulses[mirrorNum][segment] = comma[mirrorNum][segment];
          break;
        case '!':
          pulses[mirrorNum][segment] = exc[mirrorNum][segment];
          break;
        case '?':
          pulses[mirrorNum][segment] = ques[mirrorNum][segment];
        default:
          pulses[mirrorNum][segment] = space[mirrorNum][segment];
          break;
      }

      
      // Enciende o apaga el láser según el valor del pulso
      if (pulses[mirrorNum][segment] > 0) {
        digitalWrite(laserPin, HIGH);
        LetterSpace;
        LetterSpace;
      }
      else {
        digitalWrite(laserPin, LOW);
        LetterSpace;
        LetterSpace;
      }
      segment--;
    }
    digitalWrite(laserPin, LOW);
    LetterSpace;
    LetterSpace;
    mCount--;
  }
  interrupts(); // Vuelve a habilitar las interrupciones
}

// Función para recibir datos del Serial
void receiveData() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      myData[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      myData[ndx] = '\0';  // Finaliza la cadena
      ndx = 0;
      newData = true;
    }
  }
}

// Función para indicar (por ahora sin acción) que hay nuevos datos
void showNewData() {
  if (newData == true) {
    newData = false;
  }
}

// Función de retardo que utiliza un bucle NOP para una temporización más precisa que delayMicroseconds.
// Los valores aquí podrán necesitar reajuste en el ESP32.
void nopTimer(int howLong) {
  for (int t = 0; t < howLong; t++) {
    NOP;
  }
}

/* ESP8266/32 Audio Spectrum Analyser on an SSD1306 Display using an Adafruit MAX9814
 * The MIT License (MIT) Copyright (c) 2017 by David Bird. 
 * The formulation and display of an AUdio Spectrum using an ESp8266 or ESP32 and SSD1306 or SH1106 OLED Display using a Fast Fourier Transform
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files 
 * (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to 
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:  
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * See more at http://dsbird.org.uk 
*/

/* Original code from https://github.com/G6EJD
 * It can be founded here: https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display
 * Modified by Jorge Alejandro Estefania Hidalgo, student of the ETSIT UPM to make an animation similar to Apple music lockscreen widget.
 * There are 6 bands in this version
 * The gain is automatically controlled to adjust for audio saturation. For this to work, connect the GAIN pin of the microphone to PIN 13, or other, on the esp32
*/
#include <Wire.h>
#include "arduinoFFT.h"  // Standard Arduino FFT library https://github.com/kosme/arduinoFFT
/*------------Display-------------------------------------*/
#include "SSD1306.h"            // https://github.com/squix78/esp8266-oled-ssd1306
SSD1306 display(0x3c, 21, 22);  // 0.96" OLED display object definition (address, SDA, SCL) Connect OLED SDA , SCL pins to ESP SDA, SCL pins
/*------------Defines for mic-----------------------------*/
#define SAMPLES 256               // Must be a power of 2
#define SAMPLING_FREQUENCY 40000  // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
/*------------Mic Pin------------------------------*/
const int micPin = 34;
/*------------Defines for fft-----------------------------*/
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
/*-----------Defines for automatic gain control------*/
#define GAIN 13
int amplitude[3] = { 600, 550, 500 };  // Depending on your audio source level, you may need to increase this value
int filter[3] = { 1500, 1500, 5000 }; // To filter noise. The last one is the most important since it corresponds to the higher gain mode
int mode_index = 0; // indicates the gain mode of the microphone | 0 : 40 dB | 1 : 50 dB | 2 : 60 dB |
unsigned long lastTimeGainLowered = 0;
unsigned long delayToLowerGainAgain = 100;
bool bandSaturated[6] = { false }; // array to store if the bands are saturated to decrease the gain if so
int numberOfTimesSaturated = 0;  // number of saturations in periodForSaturation time
int saturationThreshold = 3; // maximum allowed saturations in periodForSaturation time
unsigned long periodForSaturation = 80;  // each period the number of times saturated resets to 0
unsigned long lastTimeForSaturation = 0; //
int band_values = 0; // if this value is too low, the gain will increase

/*---------Bezier Curve for smoothing the bar movement---------------------*/
struct Point {
  float x, y;
};

const int numOfHistoryValues = 5;
Point controlPoints[6][numOfHistoryValues];  // Control points for each band
float valueHistory[6][numOfHistoryValues];   // History values for each band
// Function to compute the point in the Bezier curve
float bezier(float p0, float p1, float p2, float p3, float t) {
  float u = 1 - t;
  float tt = t * t;
  float uu = u * u;
  float uuu = uu * u;
  float ttt = tt * t;

  return uuu * p0 + 3 * uu * t * p1 + 3 * u * tt * p2 + ttt * p3;
}
void updateControlPoints(float newValue, int barIndex) {
  // Rotate the array of the history values
  for (int i = 3; i > 0; i--) {
    valueHistory[barIndex][i] = valueHistory[barIndex][i - 1];
  }
  valueHistory[barIndex][0] = newValue;

  // Update the control points
  for (int i = 0; i < numOfHistoryValues; i++) {
    controlPoints[barIndex][i].y = valueHistory[barIndex][i];
  }
}
float getSmoothedValue(int barIndex, float t) {
  // Computes the smoothed value using the bezier curve
  return bezier(controlPoints[barIndex][0].y, controlPoints[barIndex][1].y, controlPoints[barIndex][2].y, controlPoints[barIndex][3].y, t);
}

/////////////////////////////////////////////////////////////////////////
void gain_high(void) {
  pinMode(GAIN, OUTPUT);
  digitalWrite(GAIN, HIGH);  // Configures the GAIN pin to VCC
  mode_index = 0;
}
void gain_low(void) {
  pinMode(GAIN, OUTPUT);
  digitalWrite(GAIN, LOW);  // Configures the GAIN pin to GND
  mode_index = 1;
}
void gain_float(void) {
  pinMode(GAIN, INPUT); // Configures the GAIN pin as INPUT. This way, the internal pull-up resistor disconects, leaving the pin floating
  mode_index = 2;
}

void setup() {
  Serial.begin(115200);
  pinMode(micPin, INPUT);

  gain_low();
  Wire.begin(21, 22);  // SDA, SCL on esp32
  display.init();
  display.flipScreenVertically();  // Adjust to suit or remove
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
}

void loop() {
// this loop might not be necessary but if it works, dont touch it ;)
band_values = 0;
for (int bucle = 0; bucle < 30; bucle++){
  display.clear();
  // Sample the audio signal
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long newTime = micros();
    vReal[i] = analogRead(micPin);
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) { /* do nothing to wait */
    }
  }
  // Compute the fft
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);
  // Get the raw value of each band
  double max[6] = { 0 };
  for (int i = 2; i < (SAMPLES / 2); i++) {  // Don't use sample 0 and only the first SAMPLES/2 are usable.
    int band = getBand(i);
    if (vReal[i] > max[band]) max[band] = vReal[i];
  }
  // Filter the noise and update the control points for the smoothing
  for (int i = 0; i < 6; i++) {
    max[i] -= filter[mode_index];
    if (max[i] < 0) max[i] = 0;
    updateControlPoints(max[i], i);
  }
  // Compute the smoothed values for each band
  float smoothedValues[6];
  for (int i = 0; i < 6; i++) {
    smoothedValues[i] = getSmoothedValue(i, 0.5);  // Usa t=0.5 para un valor intermedio
  }
  // draw all the bands in the display
  for (int i = 0; i < 6; i++) {
    displayBand(i, (int)smoothedValues[i]);
    if (bandSaturated[i]) numberOfTimesSaturated++;  // count saturations
  }
  // decrease the gain if the audio is saturated and restart saturation count
  if (numberOfTimesSaturated > saturationThreshold) {
    switch (mode_index) {
      case 1:
        gain_high();
        break;
      case 2:
        gain_low();
        break;
    }
    numberOfTimesSaturated = 0;
    lastTimeForSaturation = millis();
  }

  // Reset the numberOfTimesSaturated periodically
  unsigned long currentTime = millis();
  if ((currentTime - lastTimeForSaturation) > periodForSaturation) {
    numberOfTimesSaturated = 0;
    lastTimeForSaturation = currentTime;
  }
  // increase the gain if there is no audio detected
  unsigned long currentTimeGain = millis();
  if ((currentTimeGain - lastTimeGainLowered) > delayToLowerGainAgain) {
    switch (mode_index) {
      case 0: // 40 dB
        if (band_values < 150) gain_low(); // 50 dB        
        break;
      case 1: // 50 dB
        if (band_values < 150) gain_float(); // 60 dB        
        break;
    }
    lastTimeGainLowered = currentTimeGain;
  }  

    display.display();
}
}
// draw the band with the demanded size
void displayBand(int band, int dsize) {
  const int dmax = 21;
  // if (dsize < filter[mode_index]) dsize = 0; // TODO
  dsize /= amplitude[mode_index];
  bandSaturated[band] = false;
  if (dsize > dmax) {
    dsize = dmax;
    bandSaturated[band] = true;
  }    
  band_values += dsize;
  display.fillCircle(31 + 5 + 11 * band, 32 + dsize, 5);
  display.fillCircle(31 + 5 + 11 * band, 32 - dsize, 5);
  display.fillRect(31 + 11 * band, 32 - dsize, 10, 2 * dsize);
}

// Configure this thresholds to change the frecuency of each band. I dont know what frequencies corresponds to this values so I just guessed
#define threshold_1 3
#define threshold_2 5
#define threshold_3 20
#define threshold_4 45
#define threshold_5 70

int getBand(int i) {
  if (i <= threshold_1) return 0;
  if (i > threshold_1 && i <= threshold_2) return 1;
  if (i > threshold_2 && i <= threshold_3) return 2;
  if (i > threshold_3 && i <= threshold_4) return 3;
  if (i > threshold_4 && i <= threshold_5) return 4;
  if (i > threshold_5) return 5;
  return -1;
}
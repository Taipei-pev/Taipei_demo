#include <FastLED.h>
#include <colorutils.h>

// Sonar Config
#define SONAR_COUNT 14
#define SONAR_READINGS 14

// IMU Pins
#define Y 15

// IMU Calibration
#define ZERO_Y 0.21 // Accel at rest
#define NOISE 0.02 // Noise

//define pins
#define topLeftPin      22
#define topRightPin     23
#define midLeftPin      24
#define midRightPin     25
#define bottomLeftPin   26
#define bottomRightPin   27

// How many NeoPixels are attached to the Arduino?
#define topLeftLength      39
#define topRightLength      70
#define midLeftLength       42
#define midRightLength      71
#define bottomLeftLength    40
#define bottomRightLength   64

#define topCenter      55
#define midCenter      57
#define bottomCenter      53

#define gap            30

#define width          7

#define MAXWAVES       3

// The Fast LED library needs an array for each led strip

CRGB topLeft[topLeftLength];
CRGB topRight[topRightLength];
CRGB midLeft[midLeftLength];
CRGB midRight[midRightLength];
CRGB bottomLeft[bottomLeftLength];
CRGB bottomRight[bottomRightLength];


int positions[SONAR_COUNT] = {5, 15 , 25};
int distances[14] = { 9999, 9999, 9999 , 9999, 9999, 9999 , 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999};

int delayval = 10; // delay for some time

int speedIndex = 0;
int delayCount = 0;

int topWaves[MAXWAVES];
int midWaves[MAXWAVES];
int bottomWaves[MAXWAVES];
int waveCount = 0;
float prox = 1;
int proxDelayVal = 2;
int waxing = -1;
int waveSpeed = 2;

CRGB baseColor = CRGB::Black;
CRGB waveColor(255, 255, 255);
CRGB proxColor(255, 255, 0);

//TODO: Are these the positions of the sonars? I'm going to assume that these correspond to the nearest LED to the sonar
//Banting; yep. there are sonars postion that these correspond to the LED number. It Counterclockwise.
uint8_t proximity_right[14] = {3, 10, 17, 27, 38, 44, 51, 59, 66, 37, 27, 17 , 11}; //Set the sensor and led start position
// 67 在左側  左側底 40顆 設定每個感測器 偵測時LED起始位置(14顆感測器)

// Code for handlingthe sonar shockwave animation
int sonarWaveReady[SONAR_COUNT] = {1};
int sonarWaves[SONAR_COUNT] = {0};
int lowThreshold = 500;
int highThreshold = 800;
int sonarWaveWidth = 10;

#define prox_count 8
//How many prox-LED will bright.
#define prox_range 600

void setup() {
  Serial.begin(9600);

  // Setup IMU
  pinMode(Y, INPUT);

  // Setup Sonars (only using 3 right now)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);
  pinMode(A14, INPUT);

  // Setup LEDs. FastLED requires the strip type, the pin, and the length
  FastLED.addLeds<NEOPIXEL, topLeftPin>(topLeft, topLeftLength);
  FastLED.addLeds<NEOPIXEL, topRightPin>(topRight, topRightLength);
  FastLED.addLeds<NEOPIXEL, midLeftPin>(midLeft, midLeftLength);
  FastLED.addLeds<NEOPIXEL, midRightPin>(midRight, midRightLength);
  FastLED.addLeds<NEOPIXEL, bottomLeftPin>(bottomLeft, bottomLeftLength);
  FastLED.addLeds<NEOPIXEL, bottomRightPin>(bottomRight, bottomRightLength);
}

// Read all the sonars and
// return an array of distances in millimeters
void readDistances(int *distances) {

  float voltages[SONAR_COUNT] = { 0.0, 0.0, 0.0 };

  // Average a few readings to reduce noise
  for (int i = 0; i < SONAR_READINGS; i++) {
    voltages[0] += analogRead(A0);
    voltages[1] += analogRead(A1);
    voltages[2] += analogRead(A2);
    voltages[3] += analogRead(A3);
    voltages[4] += analogRead(A4);
    voltages[5] += analogRead(A5);
    voltages[6] += analogRead(A6);
    voltages[7] += analogRead(A7);
    voltages[8] += analogRead(A8);
    voltages[9] += analogRead(A9);
    voltages[10] += analogRead(A10);
    voltages[11] += analogRead(A11);
    voltages[12] += analogRead(A12);
    voltages[13] += analogRead(A13);
    //    Serial.println(voltages[0]);
    //    Serial.println(F("1"));
    //    delay(100);
  }

  for (int s = 0; s < SONAR_COUNT; s++) {
    voltages[s] /= SONAR_READINGS;
    voltages[s] *= (5.0 / 1023.0);
    //    Serial.println(voltages[0]);
    //    Serial.println(F("2"));



    // Convert to mm (5mm = 5V/1024):
    distances[s] = voltages[s] * (1024.0 / 5.0) * 5;
    //    Serial.println(distances[0]);
    //    Serial.println(F("3"));



  }
  /*Serial.print(" (0) :: ");
  Serial.print(distances[0]);
  Serial.print(F(", "));
  Serial.print(distances[1]);
  Serial.print(F(", "));
  Serial.print(distances[2]);
  Serial.print(F(", "));
  Serial.print(distances[3]);
  Serial.print(F(", "));
  Serial.println(distances[4]);
  Serial.print(" (5) :: ");
  Serial.print(distances[5]);
  Serial.print(F(", "));
  Serial.print(distances[6]);
  Serial.print(F(", "));
  Serial.print(distances[7]);
  Serial.print(F(", "));
  Serial.print(distances[8]);
  Serial.print(F(", "));
  Serial.println(distances[9]);
  Serial.print(" (10) :: ");
  Serial.print(distances[10]);
  Serial.print(F(", "));
  Serial.print(distances[11]);
  Serial.print(F(", "));
  Serial.print(distances[12]);
  Serial.print(F(", "));
  Serial.println(distances[13]);
  delay(1000);*/
}

// Return linear acceleration in Gs (9.8m/s^2)
// at the moment we have the IMU hooked up so
// the Y axis points in the dir of positive accel
float readAccel() {
  // We only care about linear motion,
  // so only testing along the Y axis.
  float v = analogRead(Y) * (5.0 / 1024.0);

  // Convert voltage to acceleration:
  // 0v = -3g, 3v = 3g
  float a = (2.0 * v) - 3.0; // m/s^2

  // Calibrate this accelerometer
  a = a - ZERO_Y;

  // Ignore noise in the signal
  if (abs(a) < NOISE) {
    a = 0.0;
  }

  return a;
}

void loop() {

  readDistances(distances);

  float accel = readAccel();
  if (accel > 0.0 || accel < 0.0) {
    Serial.print(F("Accel = ")); Serial.println(accel);
  }

  if (delayCount % delayval == 0) {

    clearLedStrips();
    stepWaves();
    if (speedIndex % gap == 0 && waveCount < MAXWAVES) {
      waveCount++;
      topWaves[waveCount - 1] = 0;
      midWaves[waveCount - 1] = 0;
      bottomWaves[waveCount - 1] = 0;
    }

    speedIndex = (speedIndex + 1) % (topCenter);
  }

  if (delayCount % proxDelayVal == 0) {
    prox = (prox + waxing * .1);
    if (prox > 5.0) {
      waxing = -1;
    }
    if (prox < 1) {
      waxing = 1;
    }
  }

  /* for (int i = 0; i < 3; i++) {
     //Serial.print("distance: ");
     //Serial.println((float)distances[i]/1000.0);
     float dist = (float)distances[i];
     displayProximity(dist, i);
   }*/
  //topRight.setPixelColor(0, 255, 0, 255);
  //topRight.setPixelColor(1, 255/2, 0, 255/2);
  //topRight.setPixelColor(2, 255/4, 0, 255/4);
  //topRight.setPixelColor(3, 255/8, 0, 255/8);
  //blendAndSetColors(topRight, 2, waveColor, 100.0); //for test LEDs
  proximity();
  FastLED.show();
  delayCount++;

}

void clearLedStrips() {
  //Serial.println("Clearing");
  fill_solid(topLeft, topLeftLength, baseColor);
  fill_solid(topRight, topRightLength, baseColor);
  fill_solid(midLeft, midLeftLength, baseColor);
  fill_solid(midRight, midRightLength, baseColor);
  fill_solid(bottomLeft, bottomLeftLength, baseColor);
  fill_solid(bottomRight, bottomRightLength, baseColor);
  //Serial.println("Cleared");
}

/*void displayProximity(float proximity, int position) {
  int centerPosition = positions[position];
  float maxProxWidth = 25;
  float maxDistance = 1500;
  float minDistance = 150;
  proximity = max(minDistance, min(maxDistance, proximity));
  int proxWidth = 200.0/(proximity) * 25;
  //int proxWidth = -1 * (maxProxWidth / (maxDistance - minDistance)) * (proximity - minDistance) + maxProxWidth;
  for (int i = centerPosition - proxWidth; i < centerPosition + proxWidth; i++) {
    float intensity = 1.0 - ((float)abs(centerPosition - i)) / ((float)proxWidth);
    int index = i;
    if (index < 0) {
      ////Serial.println(intensity);
      index = -1 * index;
      uint8_t blue = midLeft.getPixelColor(index);
      uint8_t red = midLeft.getPixelColor(index) >> 16;
      uint8_t green = midLeft.getPixelColor(index) >> 8;
      blue = intensity * proxColor[2] + (1.0 - intensity) * blue;
      red = intensity * proxColor[0] + (1.0 - intensity) * red;
      green = intensity * proxColor[1] + (1.0 - intensity) * green;
      ////Serial.println(color);
      midLeft.setPixelColor(index, midRight.Color(red, green/2, blue)); // Moderately bright green color.
    }
    else {
      ////Serial.println(intensity);
      uint8_t blue = midRight.getPixelColor(index);
      uint8_t red = midRight.getPixelColor(index) >> 16;
      uint8_t green = midRight.getPixelColor(index) >> 8;
      blue = intensity * proxColor[2] + (1.0 - intensity) * blue;
      red = intensity * proxColor[0] + (1.0 - intensity) * red;
      green = intensity * proxColor[1] + (1.0 - intensity) * green;
      ////Serial.println(color);
      midRight.setPixelColor(index, midRight.Color(red, green/2, blue)); // Moderately bright green color.
    }



  }

}*/

void blendAndSetColors(CRGB *strip, int index, CRGB& color, float alpha) {
  uint8_t blue = strip[index].b;
  uint8_t red = strip[index].r;
  uint8_t green = strip[index].g;
  blue = lerp8by8(blue, color.b, alpha);
  red = lerp8by8(red, color.r, alpha);
  green = lerp8by8(green, color.g, alpha);
  //Serial.print("RGB:: ");Serial.print(" r:: ");Serial.print(red);Serial.print(" g:: ");Serial.print(green);Serial.print(" b:: ");Serial.println(blue);
  /*Serial.print("Color:: ");
  Serial.println(color);*/
  strip[index].setRGB(red, green, blue); // Moderately bright green color.
}

void stepWaves() {
  for (int i = 0; i < waveCount; i++) {
    //Serial.print("stepping wave ");
    //Serial.println(i);
    lightRightSide(topWaves[i], topRight, topCenter);
    lightLeftSide(topWaves[i], topLeft, topCenter, topLeftLength, topRightLength, topRight);

    lightRightSide(midWaves[i], midRight, midCenter);
    lightLeftSide(midWaves[i], midLeft, midCenter, midLeftLength, midRightLength, midRight);

    lightRightSide(bottomWaves[i], bottomRight, bottomCenter);
    lightLeftSide(bottomWaves[i], bottomLeft, bottomCenter, bottomLeftLength, bottomRightLength, bottomRight);

    // Need the if statements to check when we put negative speeds, since modulo will give us negative numbers
    topWaves[i] = (topWaves[i] + waveSpeed) % (topCenter);
    if (topWaves[i] < 0) {
      topWaves[i] += topCenter;
    }

    midWaves[i] = (midWaves[i] + waveSpeed) % (midCenter);
    if (midWaves[i] < 0) {
      midWaves[i] += midCenter;
    }

    bottomWaves[i] = (bottomWaves[i] + waveSpeed) % (bottomCenter);
    if (bottomWaves[i] < 0) {
      bottomWaves[i] += bottomCenter;
    }
  }
  for (int i = 0; i < SONAR_COUNT; i++) {
    stepSonarWave(i);
  }
}

void lightRightSide(int front, CRGB *strip, int center) {
  int back = front - width;
  //Serial.print("back :: ");Serial.println(back);
  //Serial.print("back :: ");Serial.println(back);
  for (int i = front; i > back; i--) {
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    float intensity = ((float)(i - (front - width))) / ((float)width);
    int index = i;
    if (index < 0) {
      index += center;
    }
   // Serial.print("intensity :: ");Serial.println(intensity);
   // Serial.print("index :: ");Serial.println(index);
    blendAndSetColors(strip, index, waveColor, intensity*50);
   //Serial.print("index :: ");Serial.println(index);
   //delay(1000);
  }
  //Serial.println("lit right side");
}


void lightLeftSide(int front, CRGB *strip,
                   int center, int leftLength, int rightLength, CRGB *rightStrip) {

  int back = front - width;
  for (int i = front; i > back; i--) {
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    int index = i;
    float intensity = ((float)(i - (front - width))) / ((float)width);
    if (i < 0) {
      index += center;
      ////Serial.println(index);
    }

    ////Serial.println(intensity);
    if ( index >= leftLength) {
      index = rightLength - 1 - (index - leftLength);
      blendAndSetColors(rightStrip, index, waveColor, intensity*50);
    }
    else {
      blendAndSetColors(strip, index, waveColor, intensity*50);
    }

  }
  //Serial.println("Lit left side");
}
void proximity() {
  float maxProxWidth = 10;
  float maxDistance = 1500;
  float minDistance = 150;
  int value;
  for ( int j = 1 ; j <= 13 ; j++)              //right
  {
    int reading = max(minDistance, min(maxDistance, distances[j]));
    if (reading < lowThreshold && sonarWaveReady[j] && !sonarWaves[j]) {
      sonarWaveReady[j] = false;
      sonarWaves[j] = 1;
    }
    else if (reading > highThreshold) {
      sonarWaveReady[j] = true;
    }
    if (distances[j] < prox_range) {
      int first = proximity_right[j - 1];
      int last = proximity_right[j - 1] + prox_count;
      int last_left = proximity_right[j - 1] - prox_count;

      if ( j == 3 || j == 4) {
        last += 4;
      }

      if ( j <= 9) {
        value = distances[j] ;
        value = 110 - value/6;
        if(distances[0] > 450){value = 5;}
        for (int i = first ; i < last ; i++)
        {
          blendAndSetColors(topRight, i, proxColor, value);
          blendAndSetColors(midRight, i, proxColor, value);
          blendAndSetColors(bottomRight, i - 4, proxColor, value);
        }
        if ( j == 9 ) {                     //special case ,it near the door
        value = distances[j] ;
       value = 110 - value/6;
       if(distances[0] > 450){value = 5;}
          for (int i = 40 ; i > 36 ; i--)  // Bright 6 LED
          {
            blendAndSetColors(topLeft, i, proxColor, value);
            blendAndSetColors(midLeft, i, proxColor, value);
            blendAndSetColors(bottomLeft, i - 2, proxColor, value);
          }
          blendAndSetColors(bottomLeft, 39, proxColor, value);
        }
      }
      else if (j >= 10) {                         //left
        if ( j == 10  || j == 11 ) {
          last_left -= 3;
        }
        value = distances[j] ;
    value = 110 - value/6;
    if(distances[0] > 450){value = 5;}
        for (int i = first ; i > last_left ; i--)
        {
          blendAndSetColors(topLeft, i, proxColor, value);
          blendAndSetColors(midLeft, i, proxColor, value);
          blendAndSetColors(bottomLeft, i - 2, proxColor, value);
        }
      }
    }
  }
  if (distances[0] < prox_range) { // Front
    value = distances[0] ;
    value = 110 - value/5;
    if(distances[0] > 450){value = 5;}
    for (int i = 0 ; i < 5 ; i++)
    {
      blendAndSetColors(topLeft, max(i - 2, 0), proxColor, value);
      //Serial.println("front");delay(1000);
      blendAndSetColors(midLeft, i, proxColor, value);
      blendAndSetColors(bottomLeft, max(i - 2, 0), proxColor, value);
      blendAndSetColors(topRight, max(i - 2, 0), proxColor, value);
      blendAndSetColors(midRight, i, proxColor, value);
      blendAndSetColors(bottomRight, max(i - 2, 0), proxColor, value);
    }
  }
}

void stepSonarWave(int position) {
  if (!sonarWaves[position]) {
    return;
  }
  
  bool left = position > 9;
  float fade = ease8InOutApprox(1.0 - (float)sonarWaves[position] / 30.0);

  int lowFront = positions[position] - sonarWaves[position];
  for (int i = lowFront; i < (lowFront + sonarWaveWidth); i++) {
    float intensity = fade * ((float)((lowFront + sonarWaveWidth) - i)) / ((float)sonarWaveWidth);
    int index = i;
    if (index < 0) {
      index = -1 * index + 1;
      if (left) {
        blendAndSetColors(midRight, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(midLeft, index, proxColor, intensity);
      }
    }
    else {
      if (left) {
        blendAndSetColors(midLeft, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(midRight, index, proxColor, intensity);
      }
    }
    index = i + 4;
    if (index < 0) {
      index = -1 * index + 1;
      if (left) {
        blendAndSetColors(topRight, index, proxColor, intensity);
        blendAndSetColors(bottomRight, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(topLeft, index, proxColor, intensity);
        blendAndSetColors(bottomLeft, index, proxColor, intensity);
      }
    }
    else {
      if (left) {
        blendAndSetColors(topLeft, index, proxColor, intensity);
        blendAndSetColors(bottomLeft, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(topRight, index, proxColor, intensity);
        blendAndSetColors(bottomRight, index, proxColor, intensity);
      }
    }
  }

  int highFront = positions[position] + sonarWaves[position];
  for (int i = (highFront - sonarWaveWidth); i < highFront; i++) {
    float intensity = fade * ((float)((lowFront + sonarWaveWidth) - i)) / ((float)sonarWaveWidth);
    int index = i;
    if (left) {
      if (index >= midLeftLength) {
        index = (midRightLength - 1) - (index - midLeftLength);
        blendAndSetColors(midRight, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(midLeft, index, proxColor, intensity);
      }
      index = i - 4;
      if (index >= topLeftLength) {
        index = (topRightLength - 1) - (index - topLeftLength);
        blendAndSetColors(topRight, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(topLeft, index, proxColor, intensity);
      }
      index = i - 4;
      if (index > bottomLeftLength) {
        index = (bottomRightLength - 1) - (index - bottomLeftLength);
        blendAndSetColors(bottomRight, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(bottomLeft, index, proxColor, intensity);
      }
    }
    else {
      if (index >= midRightLength) {
        index = (midLeftLength - 1) - (index - midRightLength);
        blendAndSetColors(midLeft, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(midRight, index, proxColor, intensity);
      }
      index = i - 4;
      if (index >= topRightLength) {
        index = (topLeftLength - 1) - (index - topRightLength);
        blendAndSetColors(topLeft, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(topRight, index, proxColor, intensity);
      }
      index = i - 4;
      if (index > bottomRightLength) {
        index = (bottomLeftLength - 1) - (index - bottomRightLength);
        blendAndSetColors(bottomLeft, index, proxColor, intensity);
      }
      else {
        blendAndSetColors(bottomRight, index, proxColor, intensity);
      }
    }
  }
  if (sonarWaves[position] > 30) {
    sonarWaves[position] = 0;
  }
}


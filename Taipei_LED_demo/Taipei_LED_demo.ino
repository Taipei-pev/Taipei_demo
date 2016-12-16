#include <Adafruit_NeoPixel.h>

// Sonar Config
#define SONAR_COUNT 14
#define SONAR_READINGS 14

// IMU Pins
#define Y 15

// IMU Calibration
#define ZERO_Y 0.21 // Accel at rest
#define NOISE 0.02 // Noise

// Defint

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

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel topLeft = Adafruit_NeoPixel(topLeftLength, 22, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel topRight = Adafruit_NeoPixel(topRightLength, 23, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel midLeft = Adafruit_NeoPixel(midLeftLength, 24, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel midRight = Adafruit_NeoPixel(midRightLength, 25, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel bottomLeft = Adafruit_NeoPixel(bottomLeftLength, 26, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel bottomRight = Adafruit_NeoPixel(bottomRightLength, 27, NEO_GRB + NEO_KHZ800);
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

uint32_t baseColor = topLeft.Color(0, 0, 0);
uint8_t waveColor[] = {255, 255, 255};
uint8_t proxColor[] = {255, 255, 0};
uint8_t proximity_right[14] = {3, 10, 17, 27, 38, 44, 51, 59, 66, 37, 27, 17 ,11}; //Set the sensor and led start position  
// 67 在左側  左側底 40顆 設定每個感測器 偵測時LED起始位置(14顆感測器)

#define prox_count 8
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
  // Setup LEDs
  topLeft.begin();
  topRight.begin();
  midLeft.begin();
  midRight.begin();
  bottomLeft.begin();
  bottomRight.begin();
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
  proximity();
  topLeft.show();
  topRight.show();
  midLeft.show();
  midRight.show();
  bottomLeft.show();
  bottomRight.show();
  delayCount++;

}

void clearLedStrips() {
  //Serial.println("Clearing");
  for (int i = 0; i < midRightLength; i++) {
    topLeft.setPixelColor(i, baseColor);
    topRight.setPixelColor(i, baseColor);
    bottomLeft.setPixelColor(i, baseColor);
    bottomRight.setPixelColor(i, baseColor);
    midLeft.setPixelColor(i, baseColor);
    midRight.setPixelColor(i, baseColor);
  }
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

void stepWaves() {
  for (int i = 0; i < waveCount; i++) {
    //Serial.print("stepping wave ");
    //Serial.println(i);
    lightRightSide(topWaves[i], &topRight, topCenter);
    lightLeftSide(topWaves[i], &topLeft, topCenter, topLeftLength, topRightLength, &topRight);

    lightRightSide(midWaves[i], &midRight, midCenter);
    lightLeftSide(midWaves[i], &midLeft, midCenter, midLeftLength, midRightLength, &midRight);

    lightRightSide(bottomWaves[i], &bottomRight, bottomCenter);
    lightLeftSide(bottomWaves[i], &bottomLeft, bottomCenter, bottomLeftLength, bottomRightLength, &bottomRight);

    topWaves[i] = (topWaves[i] + waveSpeed) % (topCenter);
    midWaves[i] = (midWaves[i] + waveSpeed) % (midCenter);
    bottomWaves[i] = (bottomWaves[i] + waveSpeed) % (bottomCenter);
  }
}

void lightRightSide(int front, Adafruit_NeoPixel *strip, int center) {
  int back = front - width;
  for (int i = front; i > back; i--) {
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    float intensity = ((float)(i - (front - width))) / ((float)width);
    int index = i;
    if (index < 0) {
      index += center;
    }
    ////Serial.println(intensity);
    uint8_t blue = strip->getPixelColor(index);
    uint8_t red = strip->getPixelColor(index) >> 16;
    uint8_t green = strip->getPixelColor(index) >> 8;
    blue = intensity * waveColor[2] + (1.0 - intensity) * blue;
    red = intensity * waveColor[0] + (1.0 - intensity) * red;
    green = intensity * waveColor[1] + (1.0 - intensity) * green;
    //Serial.print("color: ");
    //Serial.println(color);
    strip->setPixelColor(index, strip->Color(red, green, blue)); // Moderately bright green color.

  }
  //Serial.println("lit right side");
}


void lightLeftSide(int front, Adafruit_NeoPixel *strip,
                   int center, int leftLength, int rightLength, Adafruit_NeoPixel *rightStrip) {

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
      uint8_t blue = rightStrip->getPixelColor(index);
      uint8_t red = rightStrip->getPixelColor(index) >> 16;
      uint8_t green = rightStrip->getPixelColor(index) >> 8;

      blue = intensity * waveColor[2] + (1.0 - intensity) * blue;
      red = intensity * waveColor[0] + (1.0 - intensity) * red;
      green = intensity * waveColor[1] + (1.0 - intensity) * green;

      rightStrip->setPixelColor(index, topRight.Color(red, green, blue));
    }
    else {

      uint8_t blue = strip->getPixelColor(index);
      uint8_t red = strip->getPixelColor(index) >> 16;
      uint8_t green = strip->getPixelColor(index) >> 8;

      blue = intensity * waveColor[2] + (1.0 - intensity) * blue;
      red = intensity * waveColor[0] + (1.0 - intensity) * red;
      green = intensity * waveColor[1] + (1.0 - intensity) * green;

      strip->setPixelColor(index, topLeft.Color(red, green, blue)); // Moderately bright green color.
    }

  }
  //Serial.println("Lit left side");
}
void proximity() {
  int red, green, blue;
  red = proxColor[0]; green = proxColor[1]; blue = proxColor[2];
  for ( int j = 1 ; j <= 13 ; j++)              //right 
  {
    if (distances[j] < prox_range) {
      int first = proximity_right[j-1];
      int last = proximity_right[j-1] + prox_count;
      int last_left = proximity_right[j-1] - prox_count;
      if( j == 3 || j == 4){last+=4;}
      if ( j <= 9) {
        for (int i = first ; i < last ; i++) 
        {
          topRight.setPixelColor(i, red, green, blue);
          midRight.setPixelColor(i, red, green, blue);
          bottomRight.setPixelColor(i-4, red, green, blue);
        }
       if( j == 9 ){                       //special case ,it near the door
         for (int i = 40 ; i > 36 ; i--)  // Bright 6 LED
         {
          topLeft.setPixelColor(i, red, green, blue);
          midLeft.setPixelColor(i, red, green, blue);
          bottomLeft.setPixelColor(i-2, red, green, blue);
         }
         bottomLeft.setPixelColor(39, red, green, blue);
        }
      }
      else if(j>=10){                             //left
      if( j == 10  || j == 11 ){last_left-=3;}
        for (int i = first ; i > last_left ; i--)  
        {
          topLeft.setPixelColor(i, red, green, blue);
          midLeft.setPixelColor(i, red, green, blue);
          bottomLeft.setPixelColor(i-2, red, green, blue);
        }
      }
    }
  }
  if(distances[0] < prox_range) {  // Front
    for (int i = 0 ; i < 5 ; i++) 
        {
          
          topRight.setPixelColor(i, red, green, blue);
          midRight.setPixelColor(i, red, green, blue);
          bottomRight.setPixelColor(i, red, green, blue);
          topLeft.setPixelColor(i, red, green, blue);
          midLeft.setPixelColor(i, red, green, blue);
          bottomLeft.setPixelColor(i, red, green, blue);
        }
  }


}

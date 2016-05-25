#include <Adafruit_NeoPixel.h>
#include "Adafruit_WS2801.h"
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include "SPI.h"
#include "Wire.h"

#define PIN 3
#define NUM_LEDS 24
#define LOW_READ 10
#define HIGH_READ 200

sensors_event_t event;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_RGB + NEO_KHZ800);
Adafruit_MMA8451 mma = Adafruit_MMA8451();

int fsrReading;      // the analog reading from. the FSR resistor divider
int fsrReadingBefore;
int pixelMapping;
int brightnessMapping;
int tmpvector = 0;

void setup(void) {

  Serial.begin(9600);   // We'll send debugging information via the Serial monitor
  strip.begin();
  strip.show();
  rainbowCycle(12);
  delay(1000);
  analogWrite(A1, 0);

  if (!mma.begin()) { Serial.println("Couldnt start"); } else {
    Serial.println("MMA8451 found!");
    Serial.print("Range = "); Serial.print(2 << mma.getRange());
    Serial.println("G");
  }

  mma.setRange(MMA8451_RANGE_4_G);

}

void loop(void) {

  fsrReading = 0;
  int i=0;

  mma.getEvent(&event);

  if (event.acceleration.x > 1.5 || event.acceleration.y > 1.5 || event.acceleration.z > 1.5) {

    while (i < 1000) {

      if (fsrReading < analogRead(A0)) { fsrReading = analogRead(A0); }
      i++;

    }

    if (fsrReading > LOW_READ) {

      pixelMapping = map(fsrReading,LOW_READ,HIGH_READ,1,12);
      brightnessMapping = map(fsrReading,LOW_READ,HIGH_READ,1,255);

      Serial.print("X: \t"); Serial.print(event.acceleration.x);
      Serial.print("\t");
      Serial.print("Y: \t"); Serial.print(event.acceleration.y);
      Serial.print("\t");
      Serial.print("Z: \t"); Serial.print(event.acceleration.z);
      Serial.print("\t");
      Serial.println("m/s^2 ");

      Serial.print("Analog reading = ");
      Serial.println(fsrReading);

      Serial.print("Pixel Mapping = ");
      Serial.println(pixelMapping);

      Serial.print("Brightness Mapping = ");
      Serial.println(brightnessMapping);
      Serial.println("");

      //strip.setBrightness(brightnessMapping);
      rainbowCycle(pixelMapping);

      if (pixelMapping >= 12) {

        Serial.println(tmpvector);

        analogWrite(A1, 1024);       // turn on pullup resistors
        Serial.println("Poofer!");
        delay(500);
        analogWrite(A1, 0);      // turn on pullup resistors

        Serial.print("X:\t"); Serial.print(event.data[0]);
        Serial.print("\tY:\t"); Serial.print(event.data[1]);
        Serial.print("\tZ:\t"); Serial.print(event.data[2]);
        Serial.print("\tMAG:\t"); Serial.print(tmpvector);
        Serial.println();

        delay(2000);

      }
      else if (pixelMapping >= 6) {

        delay(1000);

      } else {

        delay(500);
        
      }

    }

  }

  clear();

}

// Fill the dots one after the other with a color
void clear() {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, Color(0,0,0));
    strip.show();
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed
// along the chain
void rainbowCycle(int pixels) {

  pixels = pixels * 2;

  int i, j;

    for (i=0; i < pixels; i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel( ((i * 256 / pixels) + j) % 256) );
    }

    strip.show();   // write all the pixels out

}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#include <Adafruit_NeoPixel.h>

#define PIN 0

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(16, PIN);

uint32_t color  = 0x00ff96; // Start red

void setup() {
  pixels.begin();
  pixels.setBrightness(50); // 1/3 brightness
}

void loop() {
  /* Get a new sensor event */
  
  const byte r = 255;
  const byte g = 255;
  const byte b = 255;
  
  
    for(int i=0; i<16; i++) pixels.setPixelColor(i, r, g, b);
    pixels.show();
    delay(50);
}


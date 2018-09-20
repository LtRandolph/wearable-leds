#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <EEPROM.h>

#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <Adafruit_NeoPixel.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

#define PIN 0

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(16, PIN);

int mode = 0;

struct ClampValue
{
  float cur = 0;
  float vel = 0;

  void Tick(float dt, float targetVal)
  {
    const float targetVel = (targetVal - cur);
    vel += (targetVel - vel) * dt;
    cur += vel * dt;

    if (vel > 0 && cur > targetVal)
    {
      cur = targetVal;
      vel = 0;
    }
    if (vel < 0 && cur < targetVal)
    {
      cur = targetVal;
      vel = 0;
    }
    
    if (cur < 0) cur = 0;
    if (cur > 1) cur = 1;
  }
};

struct WrapValue
{
  float cur = 0;
  float vel = 0;

  void Tick(float dt, float targetVel)
  {
    vel += (targetVel - vel) * dt;
    cur += vel * dt;
    while (cur < 0) cur += 1;
    while (cur > 1) cur -= 1;
  }
};

ClampValue r;
ClampValue g;
ClampValue b;
ClampValue brightness;
WrapValue rot;

void setup()
{
  Serial.begin(9600);
  
  digitalWrite(7, LOW);
  //mode = EEPROM.read(0);
  //EEPROM.write(0, (mode + 1) % 3);
  mode = 2;
  pixels.begin();
  accel.begin();

  if (mode == 1)
  {
    pixels.setBrightness(15);
    for (int i=0; i<16; i++)
    {
      pixels.setPixelColor(i, 255, 215, 0);
    }
  }

  if (mode != 2)
  {
    pixels.show();
    noInterrupts();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
  }
}

const float rotDiv = 1.0 / 3.0;

const float theta = 30 * (PI / 180);
const float sinT = sin(theta);
const float cosT = cos(theta);

float xPrev = 0;
float yPrev = 0;
float zPrev = 0;

void loop()
{  
  sensors_event_t event;
  accel.getEvent(&event);

  const float xMag = min(1, abs(event.acceleration.x * cosT - event.acceleration.y * sinT) / 10);
  const float yMag = min(1, abs(event.acceleration.y * cosT + event.acceleration.x * sinT) / 10);
  const float zMag = min(1, abs(event.acceleration.z) / 10);

  const float xDiff = xMag - xPrev;
  const float yDiff = yMag - yPrev;
  const float zDiff = zMag - zPrev;
  xPrev = xMag;
  yPrev = yMag;
  zPrev = zMag;
  const float jerk = sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);

  Serial.print("Jerk: "); Serial.println(jerk);

  const float totalMag = sqrt(xMag * xMag + yMag * yMag + zMag * zMag);

  const float dt = totalMag * 0.25;

  r.Tick(dt, xMag);
  g.Tick(dt, yMag);
  b.Tick(dt, zMag);

  brightness.Tick(0.25, jerk * jerk * 2);

  pixels.setBrightness(max(5, 255 * brightness.cur));

  rot.Tick(dt, 0.25);

  float rotDivs[5];
  for (int i = 0; i < 5; ++i)
  {
    rotDivs[i] = 16 * rotDiv * ((-1 + i) + rot.cur);
  }
  
  for (int i=0; i<16; i++)
  {
    float dim = 1;
    for (int j = 0; j < 5; ++j)
    {
      dim = min(dim, pow(0.5 * abs(rotDivs[j] - i), 2));
    }
    dim = max(dim, 0.1);
    pixels.setPixelColor(i, dim * r.cur * 255, dim * g.cur * 255, dim * b.cur * 255);
  }
  pixels.show();
  delay(50);
}

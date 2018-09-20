#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <EEPROM.h>

#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <Adafruit_NeoPixel.h>

// Acceleration for dance mode. Magnet for compass mode. Each need a unique identifier.
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

#define PIN 0
#define NUM_LEDS 16

enum class Mode
{
  Dance = 0,
  Compass = 1,
  Off = 2,
  Count
};

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN);

Mode mode = Mode::Off;
bool isLeftGlove = false;

float sinT;
float cosT;

void setup()
{
  // Helpful for debugging. Turn off in production.
  Serial.begin(9600);

  digitalWrite(7, LOW);

  // Run this line once on only the left glove to permanently mark it as the left one.
  //EEPROM.write(1, 1);

  isLeftGlove = EEPROM.read(1) != 0;
  
  const float theta = (isLeftGlove ? 30 : 50) * (PI / 180);
  sinT = sin(theta);
  cosT = cos(theta);

  // Every time you power cycle (by disconnecting or hitting reset), cycle to the next mode.
  mode = (Mode)EEPROM.read(0);
  EEPROM.write(0, ((int)mode + 1) % (int)Mode::Count);
  
  pixels.begin();
  accel.begin();
  mag.begin();

  if (mode == Mode::Off)
  {
    pixels.show();
    noInterrupts();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
  }
}

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

#define NUM_DIVS 3

const float rotDiv = 1.0 / NUM_DIVS;

float xPrev = 0;
float yPrev = 0;
float zPrev = 0;

void LoopDance()
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

  // Divide the ring into a certain number of divs, with "before/after" bonus divisions to help with wrapping.
  float rotDivs[NUM_DIVS + 2];
  for (int i = -1; i < NUM_DIVS + 1; ++i)
  {
    rotDivs[i] = NUM_LEDS * rotDiv * (i + rot.cur);
  }

  for (int i = 0; i < NUM_LEDS; ++i)
  {
    float dim = 1;
    for (int j = 0; j < 5; ++j)
    {
      dim = min(dim, pow(0.5 * abs(rotDivs[j] - i), 2));
    }
    //dim = max(dim, 0.1);
    pixels.setPixelColor(i, dim * r.cur * 255, dim * g.cur * 255, dim * b.cur * 255);
  }
}

float GetAlphaFromDistance(float valA, float valB, float radius)
{
  return max(0, (radius - abs(valA - valB)) / radius);
}

void LoopCompass()
{
  sensors_event_t event;
  mag.getEvent(&event);

  float angle = atan2(event.magnetic.x, event.magnetic.y) + PI;
  angle /= (2 * PI); // normalize to (0,1) range
  // In my tests, I'm seeing it rotating the wrong way, so flip it.
  angle = 1.0f - angle;

  angle += isLeftGlove ? 0.5f : 0.0f;
  if (angle > 1.0f) angle -= 1.0f;
  
  const float markerCenter = angle * NUM_LEDS;
  const float markerRadius = 1.5f;

  for (int i = 0; i < NUM_LEDS; ++i)
  {
    // Watch for wrapping at the top of the range by subtracting NUM_LEDS from the center.
    float dim = max(GetAlphaFromDistance(i, markerCenter, markerRadius), GetAlphaFromDistance(i, markerCenter- NUM_LEDS, markerRadius));

    pixels.setPixelColor(i, dim * 70, dim * 70, dim * 120);
  }
}

void loop()
{
  if (mode == Mode::Dance)
  {
    LoopDance();
  }
  else if (mode == Mode::Compass)
  {
    LoopCompass();
  }
  
  pixels.show();
  delay(50);
}

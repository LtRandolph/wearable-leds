// Library for accelerometer chip.
#include <Adafruit_LSM303_U.h>

// Library for keeping state upon reboot.
#include <EEPROM.h>

// Library for writing to LEDs.
#include <Adafruit_NeoPixel.h>

// Libraries to allow setting "low-power mode".
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Acceleration for dance mode. Magnet for compass mode. Each need a unique identifier.
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// The control pin of the Neopixel ring.
#define PIN 0
// The number of LEDs on the Neopixel ring.
#define NUM_LEDS 16

// Modes that will be cycled between every time power is reset.
enum Mode
{
  kModeDance = 0,
  kModeCompass = 1,
  kModeOff = 2,
  kModeCount // convenience value to allow easy modulo
};

// The actual LED structure.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN);

// Declare some global variables for control.
Mode mode = kModeOff;
bool isLeftGlove = false;

// This mode makes the Arduino drain less power when the lEDs are off, but can make the Arduino
// unresponsive to new uploads. So only turn this on when you're heading out into the desert.
// To recover from this state, get the gloves into a state where neither the current state
// or the next state are low-power, so the IDE can do one cycle and have the Arduino stay on.
bool enableLowPowerOffMode = false;

// Based on which glove we're in, we'll compute these useful angle values once at startup.
float compassTheta;
float sinT;
float cosT;

void setup()
{
  // Helpful for debugging. Turn off in production.
  Serial.begin(9600);

  // Turn off the on-board LED.
  digitalWrite(7, LOW);

  // Run this line once on only the left glove to permanently mark it as the left one.
  //EEPROM.write(1, 1);
  // Run this line once on only the right glove to permanently mark it as the right one.
  //EEPROM.write(1, 0);

  // Based on having run the above line once (or more) on the left glove, we can read the value back
  // to see if we're on the left or right glove.
  isLeftGlove = EEPROM.read(1) != 0;

  // You'll need to adjust these values by hand to get your compass readings correct, and the behavior
  // of colors in dance mode to match. It will vary based on the geometry of how you sewed the gloves.
  // I tried really hard to find a "unifying theta" but the tunings didn't line up.
  compassTheta = (isLeftGlove ? 145 : 170) / 360.0;
  const float danceTheta = isLeftGlove ? 25 : 80;
  sinT = sin(danceTheta * (PI / 180));
  cosT = cos(danceTheta * (PI / 180));

  // Every time you power cycle (by disconnecting or hitting reset), cycle to the next mode.
  mode = EEPROM.read(0);
  EEPROM.write(0, (mode + 1) % kModeCount);

  // Debug mode to let you focus on one mode at a time.
  // mode = kModeCompass;
  
  pixels.begin();
  accel.begin();
  mag.begin();

  // If we're off, make sure the LEDs are turned off, then put the arduino to sleep for power saving.
  // Note that this may make the Arduino temporarily not respond to uploads from the IDE.
  if (mode == kModeOff && enableLowPowerOffMode)
  {
    pixels.show();
    noInterrupts();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
  }
}

// Struct to do damped motion towards a target value between 0 and 1. The damping is pretty simplistic
// but it seems to get the job done.
struct ClampValue
{
  float cur = 0;
  float vel = 0;

  void Tick(float dt, float targetVal)
  {
    const float targetVel = (targetVal - cur);
    vel += (targetVel - vel) * dt;
    cur += vel * dt;

    // Don't shoot past our target value; just stop at it.
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

    // Make sure to stay clamped.
    if (cur < 0) cur = 0;
    if (cur > 1) cur = 1;
  }
};

// Struct to "rotate" around a circle that goes from 0 to 1 and wraps. Wants a specified velocity, since there are no "special"
// values on a wrapped ring.
struct WrapValue
{
  float cur = 0;

  void Tick(float dt, float vel)
  {
    cur += vel * dt;

    // Wrap to stay between 0 and 1.
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
  // Get our current acceleration.
  sensors_event_t event;
  accel.getEvent(&event);

  // X and Y need to be rotated based on the way we sewed the glove together if we want holding the glove
  // vertically to mean the same thing for both hands. I like to tune it so vertical is green and horizontal
  // is red.
  const float xMag = min(1, abs(event.acceleration.x * cosT - event.acceleration.y * sinT) / 10);
  const float yMag = min(1, abs(event.acceleration.y * cosT + event.acceleration.x * sinT) / 10);
  const float zMag = min(1, abs(event.acceleration.z) / 10);

  // Now compute the values we need to see how much acceleration has changed since last tick (aka "jerk").
  const float xDiff = xMag - xPrev;
  const float yDiff = yMag - yPrev;
  const float zDiff = zMag - zPrev;
  const float jerk = sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);

  // Save off this tick's values.
  xPrev = xMag;
  yPrev = yMag;
  zPrev = zMag;

  const float dt = 0.25;

  // Aim the R/G/B at the magnitude of acceleration in that direction (roughly according to orientation to gravity).
  r.Tick(dt, xMag);
  g.Tick(dt, yMag);
  b.Tick(dt, zMag);

  // Aim the brightness at a value corresponding to jerk. Squaring it reduces how flickery it is at low values, since
  // it mostly just stays towards 0 unless you have high jerk. Times 2 just due to me messing around with it and liking
  // the feel of that number.
  brightness.Tick(dt, jerk * jerk * 2);

  // Make sure the pixels always stay on, but will be bright only if jerk is high.
  pixels.setBrightness(max(5, 255 * brightness.cur));

  // Rotate the pixels around the ring.
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
    pixels.setPixelColor(i, dim * r.cur * 255, dim * g.cur * 255, dim * b.cur * 255);
  }
}

// Ok this part sucks. Your Arduino and LEDs will have their own generated magnetic field that you need to
// subtract out. Run with FindCompassCalibration on and the glove far away from any other electronics.
// Then, rotate it as much as possible with the Serial monitor open, until you no longer see the max/min
// values changing. Then set the appropriate calibration values below for the glove to the 3 values printed
// in the second line. For verification, I typically see a range of about 90 micro-Teslas in X/Y axes.
float maxX = -1000; float minX = 1000;
float maxY = -1000; float minY = 1000;
float maxZ = -1000; float minZ = 1000;
void FindCompassCalibration(float x, float y, float z)
{
  maxX = max(maxX, x); minX = min(minX, x);
  maxY = max(maxY, y); minY = min(minY, y);
  maxZ = max(maxZ, z); minZ = min(minZ, z);

  Serial.print(minX); Serial.print(" "); Serial.print(maxX); Serial.print(" ");
  Serial.print(minY); Serial.print(" "); Serial.print(maxY); Serial.print(" ");
  Serial.print(minX); Serial.print(" "); Serial.println(maxZ);

  Serial.print(-(maxX + minX) / 2); Serial.print(" "); Serial.print(-(maxY + minY) / 2); Serial.print(" "); Serial.println(-(maxZ + minZ) / 2);
}

// These are my magnetic calibration values, as described above. You need to change these for an accurate compass.
float calibrationLeft[3] = { 4.77, 5.05, 61.43 };
float calibrationRight[3] = { 13.41, -12.09, 0.92 };

// Simple helper function to reduce copy paste below.
float GetAlphaFromDistance(float valA, float valB, float radius)
{
  return max(0, (radius - abs(valA - valB)) / radius);
}

void LoopCompass()
{
  sensors_event_t event;
  mag.getEvent(&event);

  // When you're calibrating your compass, comment this line out and follow the instructions above.
  // FindCompassCalibration(event.magnetic.x, event.magnetic.y, event.magnetic.z);

  float* calibration = isLeftGlove ? calibrationLeft : calibrationRight;

  event.magnetic.x += calibration[0];
  event.magnetic.y += calibration[1];
  event.magnetic.z += calibration[2];

  float angle = atan2(event.magnetic.x, event.magnetic.y) + PI;
  angle /= (2 * PI); // normalize to (0,1) range
  
  // In my tests, I'm seeing it rotating the wrong way, so flip it
  angle = 1.0f - angle;

  angle += compassTheta; // rotate by the appropriate amount for which glove we are
  if (angle > 1.0f) angle -= 1.0f; // loop back to a range from 0 to 1
  if (angle < 0.0f) angle += 1.0f; // loop back to a range from 0 to 1

  // We're going to show a marker where north is, which will fall off in brightness over the given radius
  const float markerCenter = angle * NUM_LEDS;
  const float markerRadius = 1.5f;

  for (int i = 0; i < NUM_LEDS; ++i)
  {
    // Watch for wrapping at the top of the range by subtracting NUM_LEDS from the center
    float dim = max(GetAlphaFromDistance(i, markerCenter, markerRadius), GetAlphaFromDistance(i, markerCenter- NUM_LEDS, markerRadius));

    // I like a dim red in case I'm preserving night vision.
    const byte color[3] = { 0x33, 0x00, 0x00 };

    pixels.setPixelColor(i, dim * color[0], dim * color[1], dim * color[2]);
  }
}

void loop()
{
  if (mode == kModeDance)
  {
    LoopDance();
  }
  else if (mode == kModeCompass)
  {
    LoopCompass();
  }
  
  pixels.show();
  delay(50);
}

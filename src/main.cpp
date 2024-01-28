#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// put function declarations here:
void turnOn();
void turnOff();
void sendPulse();

uint8_t getStatus(float distance);

void turnOnPixels();

#define LED_PIN 2

#define TRIG_PIN 12
#define ECHO_PIN 14

#define PIXELS_PIN 13
#define NUM_PIXELS 30

// define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034

const uint8_t ASSIST_DIST_CM = 135;
const uint8_t ALERT_DIST_CM = 5;
const int STOP_TIME = 250;

const uint8_t ZONE_DIST_CM = (ASSIST_DIST_CM - ALERT_DIST_CM) / 3;
const uint8_t LOW_SPEED = 50;
const uint8_t OFF = 0;
const uint8_t SAFE = 1;
const uint8_t WARN = 2;
const uint8_t DANGER = 3;
const uint8_t STOP = 4;

long duration;
float prevDistanceCm;
float distanceCm;

uint8_t status = OFF;
uint8_t red = 0;
uint8_t green = 0;
uint8_t speed = 0;
int stop_count = 0;

Adafruit_NeoPixel pixels(NUM_PIXELS, PIXELS_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output
  pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an Input

  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  pixels.begin();
}

void loop()
{

  // put your main code here, to run repeatedly:
  turnOn();

  sendPulse();
  // Reads the ECHO_PIN Pin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance
  distanceCm = duration * SOUND_VELOCITY / 2;

  uint8_t new_status = getStatus(distanceCm);

  if (new_status == STOP)
  {
    if (int(prevDistanceCm) == int(distanceCm))
    {
      if (stop_count < STOP_TIME) {
        stop_count++;
      } else {
        new_status = OFF;
        speed = 0;
      }
    } else {
      stop_count = 0;
    }
  }
  
  prevDistanceCm = distanceCm;
  status = new_status;

  turnOnPixels();

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.print(" cm.\tStatus: ");
  Serial.println(status);

  turnOff();

  delay(10);
}

// put function definitions here:
void turnOn()
{
  digitalWrite(LED_PIN, HIGH);
}

void turnOff()
{
  digitalWrite(LED_PIN, LOW);
}

void sendPulse()
{
  // Clears the TRIG_PIN pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Sets the TRIG_PIN pin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

uint8_t getStatus(float distance)
{
  if (distance > ASSIST_DIST_CM)
  {
    speed = 10;
    return OFF;
  }

  uint8_t warning = ASSIST_DIST_CM - ZONE_DIST_CM;
  uint8_t alert = ASSIST_DIST_CM - ZONE_DIST_CM * 2;
  speed = (distance * LOW_SPEED) / ASSIST_DIST_CM;

  if (distance >= warning)
  {
    return SAFE;
  }
  if (distance >= alert)
  {
    return WARN;
  }
  if (distance >= ALERT_DIST_CM)
  {
    return DANGER;
  }
  speed = 0;
  return STOP;
}

void turnOnPixels()
{
  red = 0;
  green = 0;

  if (status == SAFE) {
    green = 255;
  } else if (status == WARN) {
    red = 155;
    green = 155;
  } else if (status == DANGER || status == STOP) {
    red = 255;
  }

  if (status != OFF && status != STOP) {
    pixels.clear();
  }

  for (int i = 0; i < NUM_PIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(red, green, 0));
    pixels.show();
    delay(speed);
  }
}
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// put function declarations here:
void turnOn();
void turnOff();
void sendPulse(uint8_t trigPin);

uint8_t getStatus(float distance);

void turnOnPixels();

#define LED_PIN 2

#define TRIG_1_PIN 5
#define ECHO_1_PIN 4

#define DOUBLE_SENSOR 

#ifdef DOUBLE_SENSOR
  #define TRIG_2_PIN 12
  #define ECHO_2_PIN 14
#endif

#define PIXELS_PIN 13
#define NUM_PIXELS 30

// define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034

const uint8_t ASSIST_DIST_CM = 135;
const uint8_t ALERT_DIST_CM = 5;
const long STOP_TIME = 1 * 60000;

const uint8_t NO_MOVE_TOLERANCE = 3;
const uint8_t PIXELS_MIDDLE = NUM_PIXELS / 2;

const uint8_t ZONE_DIST_CM = (ASSIST_DIST_CM - ALERT_DIST_CM) / 3;
const uint8_t LOW_SPEED = 50;
const uint8_t OFF = 0;
const uint8_t SAFE = 1;
const uint8_t WARN = 2;
const uint8_t DANGER = 3;
const uint8_t STOP = 4;

long duration_1;
float prev_distance;
float distance_1;
#ifdef DOUBLE_SENSOR
  long duration_2;
  float distance_2;
#endif

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
  Serial.println();
  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIG_1_PIN, OUTPUT); // Sets the TRIG_1_PIN as an Output
  pinMode(ECHO_1_PIN, INPUT);  // Sets the ECHO_1_PIN as an Input

  #ifdef DOUBLE_SENSOR
    Serial.println("Using double sensor");
    pinMode(TRIG_2_PIN, OUTPUT); // Sets the TRIG_2_PIN as an Output
    pinMode(ECHO_2_PIN, INPUT);  // Sets the ECHO_2_PIN as an Input
  #endif

  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.
  Serial.print("Pixels: ");
  Serial.println(NUM_PIXELS);
  pixels.begin();
}

void loop()
{

  // put your main code here, to run repeatedly:
  turnOn();

  sendPulse(TRIG_1_PIN);

  // Reads the ECHO_1_PIN Pin, returns the sound wave travel time in microseconds
  duration_1 = pulseIn(ECHO_1_PIN, HIGH);
  // Calculate the distance
  distance_1 = duration_1 * SOUND_VELOCITY / 2;

  float distance = distance_1;

  delay(10);

  #ifdef DOUBLE_SENSOR
    sendPulse(TRIG_2_PIN);
    // Reads the ECHO_2_PIN Pin, returns the sound wave travel time in microseconds
    duration_2 = pulseIn(ECHO_2_PIN, HIGH);
    // Calculate the distance
    distance_2 = duration_2 * SOUND_VELOCITY / 2;
    if (distance_2 < distance_1) {
      distance = distance_2;
    }
  #endif

  uint8_t new_status = getStatus(distance);

  if (abs((prev_distance) - int(distance)) < NO_MOVE_TOLERANCE)
  {
    if (stop_count * (25 + speed * PIXELS_MIDDLE) < STOP_TIME)
    {
      stop_count++;
    }
    else
    {
      new_status = OFF;
      speed = 0;
    }
  }
  else
  {
    stop_count = 0;
  }

  prev_distance = distance;
  status = new_status;

  turnOnPixels();

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_1);
  #ifdef DOUBLE_SENSOR
    Serial.print(", ");
    Serial.print(distance_2);
  #endif
  Serial.print(" cm.\tStatus: ");
  switch (status)
  {
  case 0:
    Serial.println("OFF");
    break;
  case 1:
    Serial.println("SAFE");
    break;
  case 2:
    Serial.println("WARN");
    break;
  case 3:
    Serial.println("DANGER");
    break;
  case 4:
    Serial.println("STOP");
    break;
  default:
    break;
  }

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

void sendPulse(uint8_t trigPin)
{
  // Clears the TRIG pin
  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  // Sets the TRIG pin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);
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

  if (status == SAFE)
  {
    green = 255;
  }
  else if (status == WARN)
  {
    red = 155;
    green = 155;
  }
  else if (status == DANGER || status == STOP)
  {
    red = 255;
  }

  if (status != OFF && status != STOP)
  {
    pixels.clear();
  }

  for (int i = 0; i < PIXELS_MIDDLE; i++)
  {
    pixels.setPixelColor(i, pixels.Color(red, green, 0));
    pixels.setPixelColor(NUM_PIXELS - i - 1, pixels.Color(red, green, 0));
    pixels.show();
    delay(speed);
  }
}
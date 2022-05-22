#include "face.h"
#include "utils.h"
#include <Arduino.h>
#include <ArduinoLog.h>
#include <FastLED.h>
#include <DHT.h>
// #include <ros.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Header.h>
// #include <clare_head/FaceMessage.h>
#include "clareevo.h"
#include "clarempu.h"

const long BAUD = 115200;

const int SOUND_ANALOG_PIN = A6;
const int SOUND_DIGITAL_PIN = 3;

#define DHTTYPE DHT11
const int DHT11_PIN = 5;
DHT nose(DHT11_PIN, DHTTYPE);

const int NUM_EAR_LEDS = 4;
const int LED_PIN = 7;
CRGB ear_leds[NUM_EAR_LEDS];

// Fwd declarations
// void face_callback(const clare_head::FaceMessage& face_msg);

// ros::NodeHandle nh;
// ros::Subscriber<clare_head::FaceMessage> face_sub("clare/head/face",
// face_callback);

// clare_head::FaceMessage face_msg;
Face face = Face(0, 2, 3, 1);

ClareMpu mpu;
ClareEvo evo;

void setup() {
  Serial.begin(BAUD);

  // Available levels are:
  // * 0 - LOG_LEVEL_SILENT     no output
  // * 1 - LOG_LEVEL_FATAL      fatal errors
  // * 2 - LOG_LEVEL_ERROR      all errors
  // * 3 - LOG_LEVEL_WARNING    errors, and warnings
  // * 4 - LOG_LEVEL_NOTICE     errors, warnings and notices
  // * 5 - LOG_LEVEL_TRACE      errors, warnings, notices & traces
  // * 6 - LOG_LEVEL_VERBOSE    all
  // LOG_LEVEL_SILENT, LOG_LEVEL_FATAL, LOG_LEVEL_ERROR, LOG_LEVEL_WARNING,
  // LOG_LEVEL_NOTICE, LOG_LEVEL_TRACE, LOG_LEVEL_VERBOSE
  Log.begin(LOG_LEVEL_NOTICE, &Serial);
  // Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  face.reset();

  mpu.setupMpu(Log);

  Serial1.begin(115200);
  evo.setupEvo(Log, Serial1);

  FastLED.addLeds<WS2811, LED_PIN>(ear_leds, NUM_EAR_LEDS);

  nose.begin();

  // nh.initNode();
  // nh.subscribe(face_sub);
}

bool read_sound() {
  const float analog = analogRead(SOUND_ANALOG_PIN) * (3.3 / 1023.0);
  const int digital = digitalRead(SOUND_DIGITAL_PIN);
  return bool(digital);
}

void set_ears(CRGB::HTMLColorCode c) {
  for (int i = 0; i < NUM_EAR_LEDS; ++i) {
    // Turn the first led red for 1 second
    ear_leds[i] = c;
    FastLED.show();
  }
}

void smell(float &hum, float &temp) {
  hum = nose.readHumidity();
  temp = nose.readTemperature();

  if (isnan(hum)) hum = -1;
  if (isnan(temp)) temp = -1;
}

// void face_callback(const clare_head::FaceMessage& face_msg){
//   const char * expression  = face_msg.expression;
//   face.setExpression(expression);
// }

void loopEmotions(const int wait) {
  face.happy();
  delay(wait);
  face.happyBlink();
  delay(wait);
  face.bigHappy();
  delay(wait);
  face.sad();
  delay(wait);
  face.silly();
  delay(wait);
  face.angry();
  delay(wait);
  face.confused();
  delay(wait);
  face.ugh();
  delay(wait);
  face.surprised();
  delay(wait);
  face.kiss();
  delay(wait);
  face.mmm();
  delay(wait);
  face.sceptical();
  delay(wait);
  face.ohDear();
  delay(wait);
  face.noExpression();
  delay(wait);
  face.vampire();
  delay(wait);
}

float w;
float x;
float y;
float z;
float ax;
float ay;
float az;
float x1;
float x2;
float x3;
float x4;
int ctr = 0;
bool snd = false;
float temp;
float hum;

void loop() {
  ++ctr;

  // loopEmotions(500);
  mpu.readState(w, x, y, z, ax, ay, az);
  evo.readState(x1, x2, x3, x4);
  smell(hum, temp);
  snd = read_sound();

  Log.info("w=%D x=%D y=%D z=%D ax=%D ay=%D az=%D x1=%D x2=%D x3=%D x4=%D "
           "hum=%D temp=%D snd=%d",
           w, x, y, z, ax, ay, az, x1, x2, x3, x4, hum, temp, snd);

  if (ctr % 2) {
    set_ears(CRGB::Blue);
  } else {
    set_ears(CRGB::Green);
  }

  delay(1000);
}

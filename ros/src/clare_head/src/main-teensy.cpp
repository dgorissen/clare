#include "face.h"
#include "utils.h"
#include <Arduino.h>
#include <ArduinoLog.h>
#include <FastLED.h>
#include <DHT.h>
#include <IRremote.h>
// #include <ros.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Header.h>
// #include <clare_head/FaceMessage.h>
#include "clareevo.h"
#include "clarempu.h"

const long BAUD = 115200;

const int SOUND_DIGITAL_PIN = 22;
const int SOUND_ANALOG_PIN = 21;
const int LDR_ANALOG_PIN = 23;
const int DHT11_PIN = 5;
const int IRT_PIN = 6;
const int LED_PIN = 9;

DHT nose(DHT11_PIN, DHT11, 27);
const int NUM_EAR_LEDS = 4;
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

int snd = -1;
unsigned long last_snd = millis();
unsigned int debounce_snd_time = 1000;

void sound_int_handler(){
  unsigned long cur_snd = millis();

  if((cur_snd - last_snd) > debounce_snd_time){
    last_snd = millis();
    snd = 1;
  }
}

void setup() {
  // For logging
  Serial.begin(BAUD);
  
  // For readin from the dht attached to the other microcontroller
  Serial4.begin(BAUD);

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
  Log.begin(LOG_LEVEL_INFO, &Serial);
  // Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  pinMode(SOUND_DIGITAL_PIN, INPUT);
  pinMode(SOUND_ANALOG_PIN, INPUT);
  pinMode(LDR_ANALOG_PIN, INPUT);
  pinMode(IRT_PIN, OUTPUT);

  face.reset();
  mpu.setupMpu();
  nose.begin();

  Serial2.begin(115200);
  evo.setupEvo(&Serial2);

  FastLED.addLeds<WS2811, LED_PIN>(ear_leds, NUM_EAR_LEDS);
  attachInterrupt(digitalPinToInterrupt(SOUND_DIGITAL_PIN), sound_int_handler, RISING);

  IrSender.begin(IRT_PIN, ENABLE_LED_FEEDBACK);

  // nh.initNode();
  // nh.subscribe(face_sub);
}

int read_sound() {
  const float analog = analogRead(SOUND_ANALOG_PIN) * (3.3 / 1023.0);
  const int digital = digitalRead(SOUND_DIGITAL_PIN);
  return digital;
}

float read_ldr(){
  const float analog = analogRead(LDR_ANALOG_PIN) * (3.3 / 1023.0);
  return analog;
}

void set_ears(CRGB::HTMLColorCode c) {
  for (int i = 0; i < NUM_EAR_LEDS; ++i) {
    ear_leds[i] = c;
    FastLED.show();
  }
}

void smell_serial(float &hum, float &temp) {
  String shum, stemp;

  if (Serial4.available() > 0) {
    shum = Serial4.readStringUntil(',');
    stemp = Serial4.readStringUntil('\n');
  }
  //Log.info("DHT serial read: %s - %s\n", shum.c_str(), stemp.c_str());

  hum = atof(shum.c_str());
  temp = atof(stemp.c_str());
  
  if (hum < 1 || isnan(hum)) hum = -1;
  if (temp < 1 || isnan(temp)) temp = -1;
}

void smell_dhtlib(float &hum, float &temp) {
  hum = nose.readHumidity();
  temp = nose.readTemperature();

  if (isnan(hum)) hum = -1;
  if (isnan(temp)) temp = -1;
}

void send_ir(){
  uint16_t sAddress = 0x0102;
  uint8_t sCommand = 0x34;
  uint8_t sRepeats = 5;
  IrSender.sendNEC(sAddress, sCommand, sRepeats);
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

float w, x, y, z, ax, ay, az, x1, x2, x3, x4;
float temp;
float hum;
bool snd_heard;
float light;
int ctr = 0;


void loop2() {
  delay(1000);
  Serial.println("foo");
  String shum, stemp;

//  while (Serial4.available() > 0) {
//    char c = Serial4.read();
//    Serial.print("char:");
//    Serial.print(c);
//    Serial.println();
//  }

  //if (Serial4.available() > 0) {
   // String s = Serial4.readString();
   // Serial.println(s);
  //}


///  if (Serial4.available() > 0) {
  //  shum = Serial4.readStringUntil(',');
   // stemp = Serial4.readStringUntil('\n');
   // Serial.println("---");
    //Serial.println(atof(shum.c_str()));
    //Serial.println(atof(stemp.c_str()));
  //}

  float h,t;
  smell_serial(h,t);
  Log.error("Read: %D, %D\n", h, t);
}

void loop() {
  ++ctr;
  snd_heard = false;

  // loopEmotions(500);
  if(ctr % 5) {
    face.bigHappy();
  } else {
    face.sad();
  }

  mpu.readState(w, x, y, z, ax, ay, az);
  evo.readState(x1, x2, x3, x4);
  smell_serial(hum, temp);
  light = read_ldr();

  if(snd > 0) {
    snd_heard = 1;
    snd = 0;
  }

  Log.info("w=%D x=%D y=%D z=%D ax=%D ay=%D az=%D x1=%D x2=%D x3=%D x4=%D "
           "hum=%D temp=%D light=%D, snd=%d\n",
           w, x, y, z, ax, ay, az, x1, x2, x3, x4, hum, temp, light, snd_heard);

  if (ctr % 2) {
    set_ears(CRGB::Blue);
  } else {
    set_ears(CRGB::Green);
  }

  send_ir();

  delay(100);
}

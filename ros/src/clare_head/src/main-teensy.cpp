#include "face.h"
#include "utils.h"
#include <Arduino.h>
#include <ArduinoLog.h>
#include <FastLED.h>
#include <DHT.h>
#include <IRremote.h>
#include <Chrono.h>
// So we can use a different serial port
// needs to happen before ros.h include
#define USE_TEENSY_HW_SERIAL
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <clare_head/FaceMessage.h>
#include <clare_head/EarsMessage.h>
#include <clare_head/NoseMessage.h>
#include <clare_head/IRMessage.h>
#include <clare_head/EvoMessage.h>
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
void face_callback(const clare_head::FaceMessage& face_msg);

// ROS variables
// Wrapper class so we can use a custom serial port
class TeensyHardware1 : public ArduinoHardware {
  public:
  TeensyHardware1():ArduinoHardware(&Serial1, BAUD){};
};

// Fwd declarations
void face_callback(const clare_head::FaceMessage& face_msg);
void ears_callback(const clare_head::EarsMessage& ears_msg);
void ir_callback(const clare_head::IRMessage& ir_msg);

ros::NodeHandle_<TeensyHardware1> nh;
ros::Subscriber<clare_head::FaceMessage> face_sub("clare/head/face", face_callback);
ros::Subscriber<clare_head::EarsMessage> ears_sub("clare/head/ears", ears_callback);
ros::Subscriber<clare_head::IRMessage> ir_sub("clare/head/ir", ir_callback);

std_msgs::Bool noise_msg;
clare_head::FaceMessage face_msg;
clare_head::NoseMessage nose_msg;
clare_head::EvoMessage evo_msg;
sensor_msgs::Imu imu_msg;
std_msgs::Float32 light_msg;
std_msgs::UInt8 ir_msg;

ros::Publisher noise_pub("clare/head/noise", &noise_msg);
ros::Publisher nose_pub("clare/head/nose", &nose_msg);
ros::Publisher evo_pub("clare/head/evo", &evo_msg);
ros::Publisher imu_pub("clare/head/imu", &imu_msg);
ros::Publisher light_pub("clare/head/light", &light_msg);

Chrono lightChrono = Chrono();
Chrono noseChrono = Chrono();
Chrono evoChrono = Chrono();
Chrono imuChrono = Chrono();

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
    noise_msg.data = true;
    noise_pub.publish(&noise_msg);
  }
}

void setup_ros(){
  nh.initNode();
  nh.subscribe(face_sub);
  nh.subscribe(ears_sub);
  nh.subscribe(ir_sub);
  nh.advertise(noise_pub);
  nh.advertise(nose_pub);
  nh.advertise(light_pub);
  nh.advertise(evo_pub);
  nh.advertise(imu_pub);
}

void setup_test(){
  // Other serial output
  Serial1.begin(BAUD);
}

void setup() {
  // For logging
  Serial.begin(BAUD);
  
  // For reading from the dht attached to the other microcontroller
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

  setup_ros();
  //setup_test();
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

void set_ears(const CRGB c1, const CRGB c2, const CRGB c3, const CRGB c4) {
  ear_leds[0] = c1;
  ear_leds[1] = c2;
  ear_leds[2] = c3;
  ear_leds[3] = c4;
  FastLED.show();
}

void set_all_ears(const CRGB c) {
  set_ears(c,c,c,c);
}

void set_ears_and_antennae(const CRGB earCol, const CRGB antCol) {
  set_ears(earCol,antCol,earCol,antCol);
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

void face_callback(const clare_head::FaceMessage& face_msg){
  const char * expression  = face_msg.expression;
  face.setExpression(expression);
}

void ears_callback(const clare_head::EarsMessage& ears_msg){
  CRGB le(ears_msg.left_ear_col);
  CRGB la(ears_msg.left_antenna_col);
  CRGB re(ears_msg.right_ear_col);
  CRGB ra(ears_msg.right_antenna_col);
  set_ears(le, la, re, ra);
}

void ir_callback(const clare_head::IRMessage& ir_msg) {
  // TODO
  uint8_t cmd = ir_msg.cmd;
  send_ir();
}

float w, x, y, z, ax, ay, az, x1, x2, x3, x4;
float temp;
float hum;
bool snd_heard;
float light;
int ctr = 0;

void loop_test() {
  ++ctr;
  snd_heard = false;

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
    set_all_ears(CRGB::Blue);
  } else {
    set_all_ears(CRGB::Green);
  }

  send_ir();

  Serial1.print("Iteration ");
  Serial1.print(ctr);
  Serial1.println();
  
  delay(100);
}

void loop_ros() {
  float w, x, y, z, ax, ay, az, x1, x2, x3, x4;
  float temp;
  float hum;
  bool snd_heard;
  float light;
  int ctr = 0;

  snd_heard = false;
  if(snd > 0) {
    snd_heard = 1;
    snd = 0;
  }

  if(noseChrono.hasPassed(2000)){
    noseChrono.restart();
    smell_serial(hum, temp);
    nose_msg.temp = temp;
    nose_msg.humidity = hum;
    nose_pub.publish(&nose_msg);
  }

  if(evoChrono.hasPassed(500)){
    evoChrono.restart();
    evo.readState(x1, x2, x3, x4);
    evo_msg.x1 = x1;
    evo_msg.x2 = x2;
    evo_msg.x3 = x3;
    evo_msg.x4 = x4; 
    evo_pub.publish(&evo_msg);
  }

  if(lightChrono.hasPassed(5000)){
    lightChrono.restart();
    light = read_ldr();
    light_msg.data = light;
    light_pub.publish(&light_msg);
  }

  if(imuChrono.hasPassed(250)){
    imuChrono.restart();
    mpu.readState(w, x, y, z, ax, ay, az);
    imu_msg.header.stamp = nh.now();
    imu_msg.header.seq = ctr;
    imu_msg.orientation.x = x;
    imu_msg.orientation.y = y;
    imu_msg.orientation.z = z;
    imu_msg.orientation.w = w;
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_pub.publish(&imu_msg);
  }

  nh.spinOnce();
  delay(10);
  ++ctr;
}

void loop(){
  loop_ros();
  //loop_test();
}
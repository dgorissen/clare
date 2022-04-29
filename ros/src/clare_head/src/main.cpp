#include <Arduino.h>
#include <ArduinoLog.h>
#include "face.h"
#include "utils.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <clare_head/FaceMessage.h>

const long BAUD = 115200;

// Fwd declarations
void face_callback(const clare_head::FaceMessage& face_msg);

ros::NodeHandle nh;
ros::Subscriber<clare_head::FaceMessage> face_sub("clare/head/face", face_callback);

clare_head::FaceMessage face_msg;
Face face = Face(0, 1, 2, 3);

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
  //Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  
  face.reset();

  nh.initNode();
  nh.subscribe(face_sub);
}

void face_callback(const clare_head::FaceMessage& face_msg){
  const char * expression  = face_msg.expression;
  face.setExpression(expression);
}

void loopEmotions(){
    face.happy();
    delaySeconds(2);
    face.sad();
    delaySeconds(2);
    face.silly();
    delaySeconds(2);
    face.angry();
    delaySeconds(2);
    face.confused();
    delaySeconds(2);
    face.ugh();
    delaySeconds(2);
    face.surprised();
    delaySeconds(2);
    face.kiss();
    delaySeconds(2);
}

void loop() {
  while (true) {
    loopEmotions();
  }
}

#include <Arduino.h>

// Ref https://www.maxbotix.com/Arduino-Ultrasonic-Sensors-085/

// Baud rate
const long BAUD = 115200;

//--- Pin aliases for convenience
const int D2 = 2;
const int D3 = 3;
const int D4 = 4;
const int D5 = 5;
const int D6 = 6;

//--- Ultrasonics
// If this pin is low then the sensors are turned off
const int rangingEnablePin = D2;
// Set to low/high depending on the state of the rangingEnablePin
const int triggerPin = D3;
// Number of sensors
const int numUltraSensors = 3;
// Pin IDs
const int ultraPins[numUltraSensors] = {A7, A6, A5};
// Range values
int ranges[numUltraSensors];

//--- PIR Sensor
const int pirPin = D4;
int pirVal = -1;
int prevPirVal = -1;

//--- Voltage Sensor
const int voltPin = A2;
float voltVal = -1.0;

//-- Gas sensors
const int mqPins[2] = {A4, A3};
int mqVals[2] = {-1, -1};

void setup () {
  Serial.begin(BAUD);

  for(int i=0; i < numUltraSensors; ++i){
    pinMode(ultraPins[i], INPUT);
  }

  for(int i=0; i < 2; ++i){
    pinMode(mqPins[i], INPUT);
  }

  pinMode(pirPin, INPUT_PULLUP);
  pinMode(voltPin, INPUT);

  // Setup timers to facilitate publishing at a regular interval
  // http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  // TIMER 1 for interrupt frequency 0.5 Hz:
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 0.5 Hz increments
  OCR1A = 31249; // = 16000000 / (1024 * 0.5) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts
}

void read_ranges(){
  for(int i=0; i < numUltraSensors; ++i){
    ranges[i] = analogRead(ultraPins[i]) * 5;
  }
}

void read_gasses(){
  for(int i=0; i < 2; ++i){
    mqVals[i] = analogRead(mqPins[i]);
  }
}

void read_voltage(){
  int val = analogRead(voltPin);
  const float R1 = 30000.0;
  const float R2 = 7500.0;
  float vOUT = (val * 5.0) / 1023.0;
  voltVal = vOUT / (R2/(R1+R2));
}

void read_pir(){
  pirVal = digitalRead(pirPin);
}

ISR(TIMER1_COMPA_vect){
  //interrupt commands for TIMER 1
  read_voltage();
  read_gasses();

  Serial.print("V=");
  Serial.print(voltVal);

  Serial.print(" G=");
  Serial.print(mqVals[0]);
  Serial.print(",");
  Serial.print(mqVals[1]);
  Serial.println();
}

void loop() {
  read_pir();

  if(pirVal != prevPirVal){
    if(pirVal) {
      Serial.print("P=1");
    } else {
      Serial.print("P=0");
    }
    Serial.println();
    prevPirVal = pirVal;
  }

  //bool enableUltras = digitalRead(rangingEnablePin) == HIGH;
  // TODO: for testing
  bool enableUltras = true;

  if(enableUltras){
    digitalWrite(triggerPin,HIGH);
    delay(1);
    // digitalWrite(triggerPin,LOW);

    read_ranges();

    Serial.print("U=");
    Serial.print(ranges[0]);
    Serial.print(",");
    Serial.print(ranges[1]);
    Serial.print(",");
    Serial.print(ranges[2]);
    Serial.println();

    delay(150);
  } else {
    digitalWrite(triggerPin,LOW);
  }
}


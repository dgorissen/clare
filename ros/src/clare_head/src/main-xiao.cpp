#include <Arduino.h>
#include <DHT.h>

const long BAUD = 115200;
const int DHT11_PIN = 5;

DHT nose(DHT11_PIN, DHT11);

void setup() {
  Serial.begin(BAUD);
  Serial1.begin(BAUD);
  nose.begin();
}

void loop() {
  float hum = nose.readHumidity();
  float temp = nose.readTemperature();

  Serial.print("Read from DHT: ");
  Serial.print(hum);
  Serial.print(",");
  Serial.println(temp);

  if (isnan(hum)) hum = -1;
  if (isnan(temp)) temp = -1;
 
  Serial1.print(hum);
  Serial1.print(",");
  Serial1.print(temp);
  Serial1.print("\n");
  
  delay(2000);
}

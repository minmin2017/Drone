#include <Wire.h>
#include "bpm.h"

#define SDA_PIN 21
#define SCL_PIN 22

BMP280Reader bmp;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!bmp.begin(Wire, SDA_PIN, SCL_PIN)) {
    Serial.println("BMP280 not found!");
    while (1);
  }

  Serial.print("BMP280 OK @0x");
  Serial.println(bmp.address(), HEX);

  bmp.configure();
}

void loop() {
  Serial.print("Temp: ");
  Serial.print(bmp.temperatureC());
  Serial.print(" °C  |  Pressure: ");
  Serial.print(bmp.pressurehPa());
  Serial.print(" hPa  |  Alt: ");
  Serial.print(bmp.altitudeM());
  Serial.println(" m");

  delay(500);
}

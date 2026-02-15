/*
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "clocksystem.h"
#include <ESP32Servo.h>

MPU6050 mpu6050(Wire);
Servo esc1; 
Servo esc2;
Servo esc3;
Servo esc4;

#define buzzerPin 25
#define escPin1 13
#define escPin2 14
#define escPin3 27
#define escPin4 33
const int Aware = 30;
const int Danger = 45;

int throttle = 1000;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  
  esc1.attach(escPin1, 1000, 2000); 
  
  esc1.writeMicroseconds(1000); 

  esc2.attach(escPin2, 1000, 2000); 
  
  esc2.writeMicroseconds(1000); 
  
  esc3.attach(escPin3, 1000, 2000); 
  
  esc3.writeMicroseconds(1000); 

  esc4.attach(escPin4, 1000, 2000); 
  
  esc4.writeMicroseconds(1000); 
  
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  
  Serial.println("Calibrating... ห้ามขยับบอร์ด!");
  
  digitalWrite(buzzerPin, LOW); delay(100); digitalWrite(buzzerPin, HIGH);
  
  mpu6050.calcGyroOffsets(true);
  
  for(int i=0; i<2; i++){
    digitalWrite(buzzerPin, LOW); delay(200); digitalWrite(buzzerPin, HIGH); delay(50);
  }
  Serial.println("Calibrate เสร็จแล้ว!");
  Serial.println("พิมพ์ค่า 1000 - 1600 ในช่องข้างบนเพื่อเริ่มเทสมอเตอร์");
}

void loop() {
  timesys();     
  mpu6050.update();
  float p = mpu6050.getAngleX();
  float r = mpu6050.getAngleY();

  if (Serial.available() > 0) {
    int inputVal = Serial.parseInt();
    if (inputVal >= 1000 && inputVal <= 1500) { 
      throttle = inputVal;
      esc1.writeMicroseconds(throttle);
      esc2.writeMicroseconds(throttle);
      esc3.writeMicroseconds(throttle);
      esc4.writeMicroseconds(throttle);
      Serial.print("Motor Output: "); Serial.println(throttle);
      Serial.print("Pitch Output: "); Serial.println(p);
      Serial.print("Roll Output: "); Serial.println(r);
    }
  }



  if (abs(p) >= Danger || abs(r) >= Danger) {
    digitalWrite(buzzerPin, CLK_50ms ? LOW : HIGH);
  } 
  else if (abs(p) >= Aware || abs(r) >= Aware) {
    digitalWrite(buzzerPin, CLK_100ms ? LOW : HIGH);
  } 
  else {
    digitalWrite(buzzerPin, HIGH);

  }
}
*/
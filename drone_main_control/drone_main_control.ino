/**/
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "clocksystem.h"
#include <ESP32Servo.h>
#include "Communication_order.h"
#include "notification.h"

// ---------------- WiFi ----------------
const char* ssid     = "MinMin2017";
const char* password = "minmin2017i";
const bool wifi = true;
LanCommand net;
MonitorRegistry mon;

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
bool stop = false;
int throttle = 1000;
void doReset() {
  // ตัวอย่าง: reset state (จะเพิ่ม reset yaw/motor ก็ได้)
  stop = true;
  Serial.print("reset ");
}
bool handleMonitorCmd(WiFiClient& c, const String& line) {
  // จับเฉพาะคำสั่งที่ขึ้นต้นด้วย "MON"
  if (line.startsWith("MON")) {
    mon.handleCommand(c, line);
    return true;
  }
  return false;
}

void setup() {
  if (wifi) {
    bool ok = net.begin(ssid, password);
    if (!ok) {
      Serial.println("WiFi failed (net.begin timeout)");
      while (1) { delay(1000); }
    }
    Serial.print("WiFi connected, IP = ");
    Serial.println(WiFi.localIP());
  }
  

  net.onReset(doReset);
  net.setLineHook(handleMonitorCmd);
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  pinMode(escPin1,OUTPUT);
  pinMode(escPin2,OUTPUT);
  pinMode(escPin3,OUTPUT);
  pinMode(escPin4,OUTPUT);
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
  if (wifi) net.update();
  if(stop==false){

    
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

      }
    }


    Serial.print("Pitch Output: "); Serial.println(p);
    if (abs(p) >= Danger || abs(r) >= Danger) {
      digitalWrite(buzzerPin, CLK_50ms ? LOW : HIGH);
    } 
    else if (abs(p) >= Aware || abs(r) >= Aware) {
      digitalWrite(buzzerPin, CLK_100ms ? LOW : HIGH);
    } 
    else {
      digitalWrite(buzzerPin, HIGH);

    }
  }else{
    Serial.print("reset");
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
  }
}
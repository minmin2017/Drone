/*
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

// --- 1. การตั้งค่าตัวแปร PID (ค่าเหล่านี้ต้องผ่านการ Tuning) ---
float kp = 1.5, ki = 0.02, kd = 0.7;    // สำหรับ Roll และ Pitch
float kp_y = 2.0, ki_y = 0.01, kd_y = 0.0; // สำหรับ Yaw

// ตัวแปรเก็บสถานะของแต่ละแกน
float err_r, last_err_r, int_r, out_r;
float err_p, last_err_p, int_p, out_p;
float err_y, last_err_y, int_y, out_y;

float set_r = 0, set_p = 0, set_y = 0; // เป้าหมายคือรักษาระดับนิ่ง
unsigned long prevTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  
  // สำคัญ: ต้องวางโดรนนิ่งๆ เพื่อแก้ปัญหาค่า Drift ที่คุณเจอ
  mpu6050.calcGyroOffsets(true); 
  prevTime = micros();
}

void loop() {
  mpu6050.update();
  
  // คำนวณช่วงเวลา (dt) เพื่อความแม่นยำของ Integral และ Derivative
  float dt = (micros() - prevTime) / 1000000.0;
  prevTime = micros();

  // อ่านค่ามุมตามแกนที่คุณกำหนด
  float r = mpu6050.getAngleX();
  float p = mpu6050.getAngleY();
  float y = mpu6050.getAn
*/

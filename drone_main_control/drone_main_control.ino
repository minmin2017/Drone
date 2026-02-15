/**/
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP280.h>

#include "clocksystem.h"
#include "Communication_order.h"
#include "notification.h"

// ---------------- WiFi ----------------
const char* ssid     = "MinMin2017";
const char* password = "minmin2017i";
const bool wifi = true;
LanCommand net;
MonitorRegistry mon;

// ---------------- Sensors / ESC ----------------
MPU6050 mpu6050(Wire);
Adafruit_BMP280 bmp;

Servo esc1, esc2, esc3, esc4;

#define buzzerPin 25
#define escPin1 13
#define escPin2 14
#define escPin3 27
#define escPin4 33

const int Aware  = 30;
const int Danger = 45;

bool stop = false;

// ---------------- Throttle ----------------
int throttleBase = 1000;           // ผู้ใช้ตั้งจาก Serial (ฐานแรงยก)
int throttleCmd  = 1000;           // หลังรวม AltHold

// ===================== Attitude PID (Roll/Pitch/Yaw) =====================
float kp = 30.0f,  ki = 50.0f, kd = 0.0f;     // Roll/Pitch
float kp_y = 0.0f, ki_y = 0.0f, kd_y = 0.0f; // แนะนำเริ่ม 0 (yaw drift)

// anti-windup limits
float I_LIMIT_RP = 200.0f;
float OUT_LIMIT_RP = 400.0f; // จำกัด correction (us)

float err_r=0, last_err_r=0, int_r=0, out_r=0;
float err_p=0, last_err_p=0, int_p=0, out_p=0;
float err_y=0, last_err_y=0, int_y=0, out_y=0;

float set_r = 0, set_p = 0, set_y = 0;  // ล็อกให้ระดับ

unsigned long prevUs = 0;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline void resetAttPID() {
  int_r = int_p = int_y = 0;
  last_err_r = last_err_p = last_err_y = 0;
}

// ===================== BMP280 Altitude Hold PID =====================
bool altHold = false;
float seaLevel_hPa = 1013.25f; // ปรับภายหลังให้ตรงพื้นที่จริง
float altAlpha = 0.85f;        // low-pass filter
float altFilt = 0.0f;
float altSP = 0.0f;

float Kp_alt = 120.0f;         // us per meter
float Ki_alt = 20.0f;          // us per (m*s)  (รวมเข้ากับ integral เป็น us)
float Kd_alt = 40.0f;          // us per (m/s)

float I_LIMIT_alt = 200.0f;    // clamp integral (us)
float OUT_LIMIT_alt = 250.0f;  // จำกัด offset สูงสุด (us)

float i_alt = 0.0f;
float prevErr_alt = 0.0f;
bool first_alt = true;
unsigned long altPrevMs = 0;

static inline void altPidReset() {
  i_alt = 0.0f;
  prevErr_alt = 0.0f;
  first_alt = true;
}

float readAltMeters() {
  float altRaw = bmp.readAltitude(seaLevel_hPa);
  altFilt = altAlpha * altFilt + (1.0f - altAlpha) * altRaw;
  return altFilt;
}

float altPidUpdate(float sp, float pv, float dt) {
  if (dt <= 0) dt = 0.01f;
  float err = sp - pv;

  // I term (เก็บเป็น us)
  i_alt += (Ki_alt * err * dt);
  i_alt = clampf(i_alt, -I_LIMIT_alt, I_LIMIT_alt);

  // D term
  float derr = 0.0f;
  if (!first_alt) derr = (err - prevErr_alt) / dt;
  first_alt = false;
  prevErr_alt = err;

  float out = (Kp_alt * err) + i_alt + (Kd_alt * derr);
  out = clampf(out, -OUT_LIMIT_alt, OUT_LIMIT_alt);
  return out; // us offset
}

// ===================== Network hook =====================
void doReset() {
  stop = true;
  Serial.print("reset ");
}

bool handleMonitorCmd(WiFiClient& c, const String& line) {
  if (line.startsWith("MON")) {
    mon.handleCommand(c, line);
    return true;
  }
  return false;
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // WiFi
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

  // IMU
  mpu6050.begin();

  // ESC attach
  esc1.attach(escPin1, 1000, 2000);
  esc2.attach(escPin2, 1000, 2000);
  esc3.attach(escPin3, 1000, 2000);
  esc4.attach(escPin4, 1000, 2000);

  // ส่งต่ำสุดเพื่อ arm
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(2000);

  // buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);

  // BMP280 begin (ลอง 0x76/0x77)
  bool bmpOk = bmp.begin(0x76);
  if (!bmpOk) bmpOk = bmp.begin(0x77);

  if (bmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_125);
    delay(300);
    altFilt = bmp.readAltitude(seaLevel_hPa);
    altSP = altFilt;
    altPrevMs = millis();
    altPidReset();
    Serial.print("✅ BMP280 ready. Alt init = ");
    Serial.println(altFilt, 3);
  } else {
    Serial.println("❌ BMP280 not found (0x76/0x77). AltHold disabled.");
    altHold = false;
  }

  // IMU calibration
  Serial.println("Calibrating... ห้ามขยับบอร์ด!");
  digitalWrite(buzzerPin, LOW); delay(100); digitalWrite(buzzerPin, HIGH);
  mpu6050.calcGyroOffsets(true);

  for (int i = 0; i < 2; i++) {
    digitalWrite(buzzerPin, LOW); delay(200); digitalWrite(buzzerPin, HIGH); delay(50);
  }
  Serial.println("Calibrate เสร็จแล้ว!");

  prevUs = micros();
  resetAttPID();

  Serial.println("Commands:");
  Serial.println("  1000..1600  -> set throttle BASE");
  Serial.println("  H           -> toggle AltHold");
  Serial.println("  S           -> set current altitude as SP");
  Serial.println("  R           -> reset PID integrals");
}

// ===================== Loop =====================
void loop() {
  if (wifi) net.update();

  if (!stop) {
    timesys();

    // ---------- Read IMU ----------
    mpu6050.update();

    // dt for attitude PID
    unsigned long nowUs = micros();
    float dt = (nowUs - prevUs) / 1000000.0f;
    if (dt <= 0) dt = 0.001f;
    prevUs = nowUs;

    // คุณใช้ mapping นี้อยู่แล้ว: X=roll? Y=pitch? (อาจสลับตามการวางบอร์ด)
    float r = mpu6050.getAngleX();
    float p = mpu6050.getAngleY();
    float y = mpu6050.getAngleZ(); // ระวัง drift

    // ---------- Serial commands ----------
    if (Serial.available() > 0) {
      char ch = Serial.peek();

      if (ch == 'H' || ch == 'h') {
        Serial.read();
        if (bmp.sensorID() != 0) {
          altHold = !altHold;
          altPidReset();
          altPrevMs = millis();
          Serial.print("AltHold = "); Serial.println(altHold ? "ON" : "OFF");
        } else {
          Serial.println("BMP not ready; cannot enable AltHold.");
        }
      }
      else if (ch == 'S' || ch == 's') {
        Serial.read();
        if (bmp.sensorID() != 0) {
          altSP = altFilt; // ใช้ค่าที่กรองแล้ว
          altPidReset();
          Serial.print("New Alt SP = "); Serial.println(altSP, 3);
        } else {
          Serial.println("BMP not ready; cannot set SP.");
        }
      }
      else if (ch == 'R' || ch == 'r') {
        Serial.read();
        resetAttPID();
        altPidReset();
        Serial.println("PID integrals reset.");
      }
      else {
        int inputVal = Serial.parseInt();
        if (inputVal >= 1000 && inputVal <= 1600) {
          throttleBase = inputVal;
          Serial.print("Throttle BASE = "); Serial.println(throttleBase);
        }
      }
    }

    // ---------- Altitude Hold ----------
    float altNow = altFilt;
    float altOffset = 0.0f;

    if (altHold && bmp.sensorID() != 0) {
      unsigned long nowMs = millis();
      float dtAlt = (nowMs - altPrevMs) / 1000.0f;
      if (dtAlt <= 0) dtAlt = 0.01f;
      altPrevMs = nowMs;

      altNow = readAltMeters();
      altOffset = altPidUpdate(altSP, altNow, dtAlt);
    }

    throttleCmd = (int)(throttleBase + altOffset);
    throttleCmd = constrain(throttleCmd, 1000, 2000);

    // ---------- Attitude PID ----------
    // Roll
    err_r = set_r - r;
    int_r += err_r * dt;
    int_r = clampf(int_r, -I_LIMIT_RP, I_LIMIT_RP);
    out_r = (kp * err_r) + (ki * int_r) + (kd * (err_r - last_err_r) / dt);
    last_err_r = err_r;
    out_r = clampf(out_r, -OUT_LIMIT_RP, OUT_LIMIT_RP);

    // Pitch
    err_p = set_p - p;
    int_p += err_p * dt;
    int_p = clampf(int_p, -I_LIMIT_RP, I_LIMIT_RP);
    out_p = (kp * err_p) + (ki * int_p) + (kd * (err_p - last_err_p) / dt);
    last_err_p = err_p;
    out_p = clampf(out_p, -OUT_LIMIT_RP, OUT_LIMIT_RP);

    // Yaw (แนะนำเริ่ม 0,0,0)
    err_y = set_y - y;
    int_y += err_y * dt;
    int_y = clampf(int_y, -I_LIMIT_RP, I_LIMIT_RP);
    out_y = (kp_y * err_y) + (ki_y * int_y) + (kd_y * (err_y - last_err_y) / dt);
    last_err_y = err_y;
    out_y = clampf(out_y, -OUT_LIMIT_RP, OUT_LIMIT_RP);

    // ---------- Motor Mixing (X frame) ----------
    // ต้องตรงกับ “ตำแหน่งจริง” และ “ทิศหมุน” ของคุณ
    int m1 = (int)(throttleCmd - out_p - out_r + out_y); // หน้า-ขวา
    int m2 = (int)(throttleCmd - out_p + out_r - out_y); // หน้า-ซ้าย
    int m3 = (int)(throttleCmd + out_p + out_r + out_y); // หลัง-ซ้าย
    int m4 = (int)(throttleCmd + out_p - out_r - out_y); // หลัง-ขวา

    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);

    // ส่งไป ESC
    esc1.writeMicroseconds(m1);
    esc2.writeMicroseconds(m2);
    esc3.writeMicroseconds(m3);
    esc4.writeMicroseconds(m4);

    // ---------- Debug ----------
    Serial.print("R:"); Serial.print(r, 2);
    Serial.print(" P:"); Serial.print(p, 2);
    Serial.print(" Y:"); Serial.print(y, 2);
    Serial.print(" | Alt:"); Serial.print(altNow, 3);
    Serial.print(" SP:"); Serial.print(altSP, 3);
    Serial.print(" | Base:"); Serial.print(throttleBase);
    Serial.print(" Cmd:"); Serial.print(throttleCmd);
    Serial.print(" | m1:"); Serial.print(m1);
    Serial.print(" m2:"); Serial.print(m2);
    Serial.print(" m3:"); Serial.print(m3);
    Serial.print(" m4:"); Serial.print(m4);
    Serial.print(" | AltHold:"); Serial.println(altHold ? "ON" : "OFF");

    // ---------- Buzzer warning ----------
    if (abs(p) >= Danger || abs(r) >= Danger) {
      digitalWrite(buzzerPin, CLK_50ms ? LOW : HIGH);
    } else if (abs(p) >= Aware || abs(r) >= Aware) {
      digitalWrite(buzzerPin, CLK_100ms ? LOW : HIGH);
    } else {
      digitalWrite(buzzerPin, HIGH);
    }
  }
  else {
    // reset state
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
  }
}

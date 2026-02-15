#ifndef BMP_PID_H
#define BMP_PID_H

#include <Wire.h>
#include <Adafruit_BMP280.h>

// ================= PID CLASS =================
class PIDController {
public:
  PIDController(float kp, float ki, float kd,
                float i_limit, float out_limit)
  : Kp(kp), Ki(ki), Kd(kd),
    I_LIMIT(i_limit), OUT_LIMIT(out_limit) {}

  void reset() {
    integral = 0.0f;
    prevError = 0.0f;
    first = true;
  }

  float update(float setpoint, float measured, float dt) {
    if (dt <= 0) dt = 0.001f;

    float error = setpoint - measured;

    // I
    integral += error * dt;
    if (integral > I_LIMIT) integral = I_LIMIT;
    if (integral < -I_LIMIT) integral = -I_LIMIT;

    // D
    float derivative = 0.0f;
    if (!first) derivative = (error - prevError) / dt;
    first = false;

    prevError = error;

    float out = Kp * error + Ki * integral + Kd * derivative;

    if (out > OUT_LIMIT) out = OUT_LIMIT;
    if (out < -OUT_LIMIT) out = -OUT_LIMIT;

    lastError = error;
    lastDerivative = derivative;
    lastOut = out;

    return out;
  }

  float getError() const { return lastError; }
  float getIntegral() const { return integral; }
  float getDerivative() const { return lastDerivative; }
  float getOut() const { return lastOut; }

private:
  float Kp, Ki, Kd;
  float I_LIMIT, OUT_LIMIT;

  float integral = 0.0f;
  float prevError = 0.0f;
  bool first = true;

  float lastError = 0.0f;
  float lastDerivative = 0.0f;
  float lastOut = 0.0f;
};

// ================= BMP + PID SYSTEM =================
class BMP_PID_System {
public:
  BMP_PID_System(float kp, float ki, float kd,
                 float i_limit, float out_limit,
                 float delta_up = 0.05f)
  : pid(kp, ki, kd, i_limit, out_limit),
    deltaUp(delta_up) {}

  bool begin(uint8_t addr = 0x76) {
    if (!bmp.begin(addr)) return false;

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_125);

    delay(300);

    setNewSetpoint();   // ตั้ง SP ตอนเริ่ม
    prevMs = millis();
    return true;
  }

  void setNewSetpoint() {
    altRefAbs = bmp.readAltitude(1013.0);
    spAbs = altRefAbs + deltaUp;
    pid.reset();
  }

  float update() {
    unsigned long now = millis();
    float dt = (now - prevMs) / 1000.0f;
    if (dt <= 0) dt = 0.01f;
    prevMs = now;

    pvAbs = bmp.readAltitude(1011.9);
    return pid.update(spAbs, pvAbs, dt);
  }

  void printDebug() {
    Serial.print("AltAbs = ");
    Serial.print(pvAbs, 4);
    Serial.print(" m | altRefAbs = ");
    Serial.print(altRefAbs, 4);
    Serial.print(" m | SP = ");
    Serial.print(spAbs, 4);
    Serial.print(" m | Error = ");
    Serial.print(pid.getError(), 4);
    Serial.print(" | PID = ");
    Serial.println(pid.getOut(), 4);
  }

private:
  Adafruit_BMP280 bmp;
  PIDController pid;

  float altRefAbs = 0.0f;
  float spAbs = 0.0f;
  float pvAbs = 0.0f;
  float deltaUp;

  unsigned long prevMs = 0;
};

#endif

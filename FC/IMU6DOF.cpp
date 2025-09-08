#include "IMU6DOF.h"
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#define TIMEOUT_US 50000  // was 10000; give sensor time after power-up

IMU6DOF::IMU6DOF()
  : AccErrorX(-0.03f), AccErrorY(-0.01f), AccErrorZ(0.01f),
    GyroErrorX(0.11f), GyroErrorY(0.27f), GyroErrorZ(-0.06f),
    AccX(0), AccY(0), AccZ(0), AccX_prev(0), AccY_prev(0), AccZ_prev(0),
    GyroX(0), GyroY(0), GyroZ(0), GyroX_prev(0), GyroY_prev(0), GyroZ_prev(0),
    q0(1), q1(0), q2(0), q3(0),
    roll(0), pitch(0), yaw(0),
    now_us(0), last_us(0), dt_fc(0), target_freq(500) {}

bool IMU6DOF::init() {
  if (!IMU.begin()) {
    return false;
  }
  calibrateAttitude();
  return true;
}


void IMU6DOF::calibrateAttitude() {
  // Warm up with *valid* samples only
  const int warmup = 1000;
  int got = 0;
  uint32_t t0 = micros();
  while (got < warmup && (micros() - t0) < 1000000UL) { // up to 1s
    if (getIMUdata()) {
      Madgwick6DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, 0.002f);
      ++got;
    } else {
      delayMicroseconds(200);
    }
  }
}

void IMU6DOF::setLoopFrequency(int freq) {
  target_freq = freq;
}

void IMU6DOF::update() {
  last_us = now_us;
  now_us = micros();
  dt_fc = (now_us > last_us) ? (now_us - last_us) / 1e6f : 0.002f;
  // Clamp dt to sane range to protect filter after stalls
  if (dt_fc < 0.0005f) dt_fc = 0.0005f;
  if (dt_fc > 0.02f)   dt_fc = 0.02f;

  // Only update filter when we actually read new data
  if (getIMUdata()) {
    Madgwick6DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt_fc);
  }

  if (target_freq > 0) {
    const uint32_t target_us = (uint32_t)(1000000.0f / target_freq);
    while ((micros() - now_us) < target_us) {
      tight_loop_contents(); // don’t hard spin
    }
  }
}

bool IMU6DOF::getIMUdata() {
  const uint32_t start = micros();
  while ((!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) &&
         (micros() - start) < TIMEOUT_US) {
    delayMicroseconds(100);
  }

  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    return false; // no fresh sample this tick
  }

  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  // Simple IIR smoothing + bias remove
  GyroX = (1 - B_gyro) * GyroX_prev + B_gyro * (gx - GyroErrorX);
  GyroY = (1 - B_gyro) * GyroY_prev + B_gyro * (gy - GyroErrorY);
  GyroZ = (1 - B_gyro) * GyroZ_prev + B_gyro * (gz - GyroErrorZ);
  GyroX_prev = GyroX; GyroY_prev = GyroY; GyroZ_prev = GyroZ;

  AccX = (1 - B_accel) * AccX_prev + B_accel * (ax - AccErrorX);
  AccY = (1 - B_accel) * AccY_prev + B_accel * (ay - AccErrorY);
  AccZ = (1 - B_accel) * AccZ_prev + B_accel * (az - AccErrorZ);
  AccX_prev = AccX; AccY_prev = AccY; AccZ_prev = AccZ;

  return true;
}

void IMU6DOF::Madgwick6DOF(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt) {
  gx *= DEG_TO_RAD; gy *= DEG_TO_RAD; gz *= DEG_TO_RAD;

  // Protect against zero accel vector (e.g., no data)
  float anorm = ax*ax + ay*ay + az*az;
  if (anorm < 1e-12f) return;
  float recipNorm = invSqrt(anorm);
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

  float qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
  float qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
  float qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
  float qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

  float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
  float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
  float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
  float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

  float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
  float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
  float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
  float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;
  float sn = s0*s0 + s1*s1 + s2*s2 + s3*s3;
  if (sn > 1e-20f) {
    float rs = invSqrt(sn);
    s0 *= rs; s1 *= rs; s2 *= rs; s3 *= rs;
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  float qn = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  if (qn > 0.0f) {
    float rqn = invSqrt(qn);
    q0 *= rqn; q1 *= rqn; q2 *= rqn; q3 *= rqn;
  }

  roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1q1 - q2q2) * RAD_TO_DEG;
  pitch = -asinf(constrain(-2.0f * (q1*q3 - q0*q2), -0.999999f, 0.999999f)) * RAD_TO_DEG;
  yaw   = -atan2f(q1*q2 + q0*q3, 0.5f - q2q2 - q3q3) * RAD_TO_DEG;
}

inline float IMU6DOF::invSqrt(float x) { return 1.0f / sqrtf(x); }

float IMU6DOF::getRoll()  { return roll; }
float IMU6DOF::getPitch() { return pitch; }
float IMU6DOF::getYaw()   { return yaw; }

float IMU6DOF::getRollRate() {
  return GyroX;
}

float IMU6DOF::getPitchRate() {
  return GyroY;
}

float IMU6DOF::getYawRate() {
  return GyroZ;
}

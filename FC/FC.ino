// Nano RP2040 Connect (Arduino Mbed core)
// Core 0 (USB): iBUS → PID → Servos
// Core 1 (IMU): runs IMU updates and writes to shared state

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "IMU6DOF.h"
#include "IBus.h"
#include "PID.h"

// ==== UART pin mapping per core ====
#if defined(ARDUINO_ARCH_RP2040)        // Earle Philhower core
  #define USE_PHILHOWER 1
#else
  #define USE_PHILHOWER 0               // Arduino Mbed core
#endif

#if USE_PHILHOWER
  static constexpr int UART_TX_PIN = D4;   // not used
  static constexpr int UART_RX_PIN = D5;   // iBUS signal wire
#else
  static constexpr int UART_TX_PIN = -1;
  static constexpr int UART_RX_PIN = -1;
#endif

#define NUM_CH     6
#define IBUS_BAUD  115200
#define PRINT_PERIOD_MS 100

// Servo Pins
#define SERVO_1_PIN  D6
#define SERVO_2_PIN  D7
#define SERVO_3_PIN  D8
#define SERVO_4_PIN  D9

// Servo endpoint
#define SERVO_MIN 1000
#define SERVO_MAX 2000

// Normalization Scales
#define ROLL_NORM  45  // deg
#define PITCH_NORM 45  // deg
#define YAW_NORM   10  // deg/s

// Failsafe values
const uint16_t FAILSAFE_DEFAULTS[NUM_CH] = {
  1500, // Roll → center
  1500, // Pitch → center
  SERVO_MIN, // Throttle → idle
  1500, // Yaw → center
  SERVO_MIN, // Aux1
  SERVO_MIN  // Aux2
};


// ======== Shared State & Seqlock ========
struct SharedState {
  volatile uint32_t seq;
  volatile uint32_t last_update_ms;

  float imu_roll, imu_pitch, imu_yaw;
  float imu_roll_rate, imu_pitch_rate, imu_yaw_rate;
};

struct DesiredState {
  float roll_angle;
  float pitch_angle;
  float yaw_rate;
};

struct ControlOutputs {
  float roll;
  float pitch;
  float yaw;
};

SharedState g_state;
static uint32_t last_iBus_ms = 0;

// seqlock helpers
static inline void seqlock_write_begin(volatile uint32_t &seq) {
  seq++;
  __atomic_thread_fence(__ATOMIC_ACQ_REL);
}
static inline void seqlock_write_end(volatile uint32_t &seq) {
  __atomic_thread_fence(__ATOMIC_ACQ_REL);
  seq++;
}

template<typename T>
bool seqlock_read_consistent(const T &src, T &dst) {
  const volatile uint32_t *pseq = &src.seq;
  while (true) {
    uint32_t before = *pseq;
    if (before & 1u) continue;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    memcpy(&dst, &src, sizeof(T));
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    uint32_t after = src.seq;
    if (before == after && !(after & 1u)) return true;
  }
}

// ======== Core 1 (IMU) ========
IMU6DOF imu;

void setup1() {
  Wire.begin();
  Wire.setClock(400000);
  while (!imu.init()) { delay(50); }
  imu.setLoopFrequency(500);
  memset((void*)&g_state, 0, sizeof(g_state));
}

// ======== Core 1 (IMU) ========
void loop1() {
  static uint32_t last_loop_us = micros();

  imu.update();

  seqlock_write_begin(g_state.seq);
  g_state.imu_roll       = imu.getRoll();
  g_state.imu_pitch      = imu.getPitch();
  g_state.imu_yaw        = imu.getYaw();
  g_state.imu_roll_rate  = imu.getRollRate();
  g_state.imu_pitch_rate = imu.getPitchRate();
  g_state.imu_yaw_rate   = imu.getYawRate();
  g_state.last_update_ms = millis();
  seqlock_write_end(g_state.seq);

  // maintain ~500 Hz loop timing
  const uint32_t target_period_us = 2000; // 2 ms = 500 Hz
  uint32_t now_us = micros();
  uint32_t elapsed = now_us - last_loop_us;
  if (elapsed < target_period_us) {
    delayMicroseconds(target_period_us - elapsed);
  }
  last_loop_us = micros();
}



// ======== Core 0 (USB + iBUS + PID + Servos) ========
IBusDecoder ibus(Serial1);
DesiredState desired;
ControlOutputs control;

// Servos
Servo servo1, servo2, servo3, servo4;

// PID controllers
PID pidRoll(0.8f, 0.4f, 0.02f);
PID pidPitch(0.8f, 0.4f, 0.02f);
PID pidYaw(0.8f, 0.4f, 0.02f);

template <typename T1, typename T2>
float fmap(T1 x, T2 in_min, T2 in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void normalizeDesiredState(uint16_t *iBusData, DesiredState &st) {
  st.roll_angle  = fmap(iBusData[0], 1000, 2000, -ROLL_NORM, ROLL_NORM);
  st.pitch_angle = fmap(iBusData[1], 1000, 2000, -PITCH_NORM, PITCH_NORM);
  st.yaw_rate    = fmap(iBusData[3], 1000, 2000, -YAW_NORM, YAW_NORM);
}

void calculateOutputs(const DesiredState &desired, ControlOutputs &out,
                      float roll_angle, float pitch_angle,
                      float yaw_rate, float dt) {
  out.roll  = pidRoll.compute(desired.roll_angle,  roll_angle,  dt);
  out.pitch = pidPitch.compute(desired.pitch_angle, pitch_angle, dt);
  out.yaw   = pidYaw.compute(desired.yaw_rate, yaw_rate, dt);
}

void writeCommands(const ControlOutputs &out, uint16_t throttleRaw) {
  // Simple fixed-wing mixer
  int ail = constrain(map(out.roll,  -1.0f, 1.0f, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
  int ele = constrain(map(out.pitch, -1.0f, 1.0f, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
  int rud = constrain(map(out.yaw,   -1.0f, 1.0f, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
  int thr = constrain(throttleRaw, SERVO_MIN, SERVO_MAX);

  servo1.writeMicroseconds(ail); // aileron
  servo2.writeMicroseconds(ele); // elevator
  servo3.writeMicroseconds(thr); // throttle
  servo4.writeMicroseconds(rud); // rudder
}

void applyFailsafe(uint16_t *channels, bool got_frame) {
  if (got_frame) {
    last_iBus_ms = millis();  // refresh on valid frame
    return;
  }

  // If no valid frame for >100 ms → apply failsafe defaults
  if (millis() - last_iBus_ms > 100) {
    for (int i = 0; i < NUM_CH; i++) {
      channels[i] = FAILSAFE_DEFAULTS[i];
    }
  }
}


void sendTelemetryFrame(const uint16_t *channels,
                        const SharedState &snap,
                        const ControlOutputs &control,
                        uint32_t bad, uint32_t lost) {
  const uint8_t START_BYTE = 0xAA;

  // Prepare buffer
  uint8_t buf[128];
  int idx = 0;

  buf[idx++] = START_BYTE;  // start
  buf[idx++] = 0;           // placeholder for length

  auto put16 = [&](uint16_t v) {
    buf[idx++] = v & 0xFF;
    buf[idx++] = (v >> 8) & 0xFF;
  };

  auto put32f = [&](float f) {
    uint8_t *p = (uint8_t*)&f;
    for (int i = 0; i < 4; i++) buf[idx++] = p[i];
  };

  // Channels
  for (int i = 0; i < NUM_CH; i++) put16(channels[i]);

  // IMU
  put32f(snap.imu_roll);
  put32f(snap.imu_pitch);
  put32f(snap.imu_yaw);

  // Rates
  put32f(snap.imu_roll_rate);
  put32f(snap.imu_pitch_rate);
  put32f(snap.imu_yaw_rate);

  // PID outputs
  put32f(control.roll);
  put32f(control.pitch);
  put32f(control.yaw);

  // Bad/lost
  put16((uint16_t)bad);
  put16((uint16_t)lost);

  // Fill length
  buf[1] = idx - 2; // exclude start + length field

  // Compute checksum (XOR of payload only)
  uint8_t checksum = 0;
  for (int i = 2; i < idx; i++) {
    checksum ^= buf[i];
  }
  buf[idx++] = checksum;

  // Send frame
  Serial.write(buf, idx);
}

void printTelemetry(const uint16_t *channels,
                    const SharedState &snap,
                    const ControlOutputs &control,
                    uint32_t bad,
                    uint32_t lost) {
  Serial.print("CH: ");
  for (int i = 0; i < NUM_CH; i++) {
    Serial.print(channels[i]);
    if (i < NUM_CH - 1) Serial.print(' ');
  }

  Serial.print(" | R: ");
  Serial.print(snap.imu_roll, 2);
  Serial.print(" P: ");
  Serial.print(snap.imu_pitch, 2);
  Serial.print(" Y: ");
  Serial.print(snap.imu_yaw, 2);

  Serial.print(" | Rates R: ");
  Serial.print(snap.imu_roll_rate, 2);
  Serial.print(" P: ");
  Serial.print(snap.imu_pitch_rate, 2);
  Serial.print(" Y: ");
  Serial.print(snap.imu_yaw_rate, 2);

  Serial.print(" | RollOut: ");
  Serial.print(control.roll, 2);
  Serial.print(" PitchOut: ");
  Serial.print(control.pitch, 2);
  Serial.print(" YawOut: ");
  Serial.print(control.yaw, 2);

  Serial.print(" | bad: ");
  Serial.print(bad);
  Serial.print(" | lost: ");
  Serial.println(lost);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Core0: iBUS + IMU + PID");

#if USE_PHILHOWER
  Serial1.setRX(UART_RX_PIN);
  Serial1.setTX(UART_TX_PIN);
#endif
  Serial1.setTimeout(0);
  ibus.begin(IBUS_BAUD);

  servo1.attach(SERVO_1_PIN, SERVO_MIN, SERVO_MAX);
  servo2.attach(SERVO_2_PIN, SERVO_MIN, SERVO_MAX);
  servo3.attach(SERVO_3_PIN, SERVO_MIN, SERVO_MAX);
  servo4.attach(SERVO_4_PIN, SERVO_MIN, SERVO_MAX);
}

void loop() {
  static uint32_t last_print = 0;
  static uint32_t bad = 0, lost = 0;
  static uint32_t last_time = millis();

  uint16_t channels[NUM_CH];
  SharedState snap;

  bool got_frame = ibus.poll(channels, bad, lost);
  applyFailsafe(channels, got_frame);

  if (!seqlock_read_consistent(g_state, snap)) return;

  uint32_t now = millis();
  float dt = (now - last_time) / 1000.0f;
  last_time = now;

  // Normalize desired state
  normalizeDesiredState(channels, desired);

  // PID outputs
  calculateOutputs(desired, control,
                   snap.imu_roll,
                   snap.imu_pitch,
                   snap.imu_yaw_rate,   // 🔹 use gyro Z
                   dt);

  // Mix + write to servos
  writeCommands(control, channels[2]);

  // Telementry print
  if (now - last_print >= PRINT_PERIOD_MS || got_frame) {
    last_print = now;
    sendTelemetryFrame(channels, snap, control, bad, lost);
    //printTelemetry(channels, snap, control, bad, lost); // Used for serial based debug
  }
}

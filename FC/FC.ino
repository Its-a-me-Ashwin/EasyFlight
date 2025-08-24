// Nano RP2040 Connect (Arduino Mbed core)
// Core 0 (USB): prints at 10 Hz
// Core 1 (UART): reads iBUS frames, publishes to shared state

#include <Arduino.h>
#include <Wire.h>
#include "IMU6DOF.h"
#include "PID.h"


// ==== UART pin mapping per core ====
#if defined(ARDUINO_ARCH_RP2040)        // Earle Philhower core
  #define USE_PHILHOWER 1
#else
  #define USE_PHILHOWER 0               // Arduino Mbed core
#endif

// Choose pins + note for wiring
#if USE_PHILHOWER
  // iBUS RX on D5, TX on D4 (move your wire to D5)
  static constexpr int UART_TX_PIN = D4;
  static constexpr int UART_RX_PIN = D5;
  #define WIRING_NOTE "Philhower: iBUS -> D5 (RX)"
#else
  // Mbed core: Serial1 fixed to D0/D1 (move your wire to D0)
  // setRX/setTX are not used; pins are fixed by the core
  static constexpr int UART_TX_PIN = -1;
  static constexpr int UART_RX_PIN = -1;
  #define WIRING_NOTE "Mbed: iBUS -> D0 (RX)"
#endif

// ======== Config ========
#define NUM_CH            6
#define PRINT_PERIOD_MS   100   // 10 Hz
#define IBUS_BAUD         115200
// ========================

// ======== Shared State & Seqlock ========
struct SharedState {
  // Seqlock (must be first two members)
  volatile uint32_t seq;           // increments before/after write (odd=in progress)
  volatile uint32_t last_update_ms;

  // App data (extend as needed)
  uint16_t channels[NUM_CH];       // CH1..CH6
  uint32_t frame_count;            // valid frames seen
  uint32_t bad_checksum;           // checksum failures
  uint32_t lost_sync;              // header sync issues

  float imu_roll, imu_pitch, imu_yaw;
};

// Global shared data
SharedState g_state;

// Seqlock helpers
static inline void seqlock_write_begin(volatile uint32_t &seq) {
  seq++; // make odd
  __atomic_thread_fence(__ATOMIC_ACQ_REL);
}
static inline void seqlock_write_end(volatile uint32_t &seq) {
  __atomic_thread_fence(__ATOMIC_ACQ_REL);
  seq++; // make even
}

// Snapshot read (returns true if consistent copy obtained)
template<typename T>
bool seqlock_read_consistent(const T &src, T &dst) {
  // We assume seq and last_update_ms are first/volatile in struct
  const volatile uint32_t *pseq = &src.seq;
  while (true) {
    uint32_t before = *pseq;
    if (before & 1u) { continue; } // writer in progress
    __atomic_thread_fence(__ATOMIC_ACQUIRE);

    // Copy the struct byte-wise to avoid tearing on non-volatile fields
    memcpy(&dst, &src, sizeof(T));

    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    uint32_t after = src.seq;
    if (before == after && !(after & 1u)) return true;
  }
}

// ---------- Non-blocking iBUS parser (state machine) ----------
class IBusDecoder {
public:
  explicit IBusDecoder(HardwareSerial &uart, uint8_t num_ch = NUM_CH)
  : _uart(uart), _num_ch(num_ch > NUM_CH ? NUM_CH : num_ch) { reset(); }

  void begin(uint32_t baud = IBUS_BAUD) { _uart.begin(baud); }

  // Returns true when a full, valid frame decoded into out_ch
  bool poll(uint16_t *out_ch, uint32_t &bad_checksum, uint32_t &lost_sync) {
    while (_uart.available()) {
      uint8_t b = _uart.read();
      switch (_state) {
        case 0: // expect 0x20 (length)
          if (b == 0x20) { _sum = b; _idx = 0; _state = 1; }
          else { lost_sync++; }
          break;
        case 1: // expect 0x40 (type)
          if (b == 0x40) { _sum += b; _state = 2; }
          else { lost_sync++; _state = 0; }
          break;
        case 2: // read 28 payload bytes
          _buf[_idx++] = b; _sum += b;
          if (_idx >= 28) { _state = 3; _idx = 0; }
          break;
        case 3: // checksum LSB
          _chkL = b; _state = 4; break;
        case 4: { // checksum MSB
          uint16_t chk_calc = (uint16_t)(0xFFFF - (_sum & 0xFFFF)); // ← fixed
          uint16_t chk_recv = (uint16_t)_chkL | ((uint16_t)b << 8);
          if (chk_calc == chk_recv) {
            for (int i = 0; i < _num_ch; ++i) {
              int o = i * 2;
              out_ch[i] = (uint16_t)_buf[o] | ((uint16_t)_buf[o + 1] << 8);
            }
            reset();
            return true;
          } else {
            bad_checksum++;
            reset();
          }
        } break;
      }
    }
    return false;
  }

  void reset() { _state = 0; _idx = 0; _sum = 0; _chkL = 0; }

private:
  HardwareSerial &_uart;
  uint8_t  _num_ch;

  uint8_t  _state = 0;
  uint8_t  _buf[28];
  uint8_t  _idx = 0;
  uint32_t _sum = 0;
  uint8_t  _chkL = 0;
};


// ======== Core 1 (iBUS + IMU) ========
IBusDecoder ibus(Serial1);
IMU6DOF imu;

void setup1() {
  Wire.begin();
  Wire.setClock(400000);
  
  // Serial1.setRX(D5);   // iBUS signal wire goes to D5
  // Serial1.setTX(D4);   // TX not used by iBUS, but must be set
  // Serial1.setTimeout(2);
  // Serial1.begin(115200);   // iBUS is 115200 8N1

#if USE_PHILHOWER
  Serial1.setRX(UART_RX_PIN);
  Serial1.setTX(UART_TX_PIN);
#endif
  Serial1.setTimeout(10);
  Serial1.begin(115200);   // iBUS 115200 8N1

  ibus.begin(IBUS_BAUD);
  while (!imu.init())
  {
    delay(50);
  }
  imu.setLoopFrequency(500);

  // Initialize shared state
  memset((void*)&g_state, 0, sizeof(g_state));
  g_state.last_update_ms = millis();
  for (int i = 0; i < NUM_CH; ++i) g_state.channels[i] = 1500; // neutral
  g_state.imu_pitch = 0;
  g_state.imu_roll = 0;
  g_state.imu_yaw = 0;
  
}

void loop1() {
  uint16_t ch_local[NUM_CH];
  uint32_t bad = g_state.bad_checksum;
  uint32_t lost = g_state.lost_sync;

  if (ibus.poll(ch_local, bad, lost)) {
    seqlock_write_begin(g_state.seq);
    for (int i = 0; i < NUM_CH; ++i) g_state.channels[i] = ch_local[i];
    g_state.last_update_ms = millis();
    g_state.frame_count++;
    g_state.bad_checksum = bad;
    g_state.lost_sync = lost;
    seqlock_write_end(g_state.seq);
  }

  imu.update();
  float r = imu.getRoll(), p = imu.getPitch(), y = imu.getYaw();
  
  seqlock_write_begin(g_state.seq);
  g_state.imu_roll = r;
  g_state.imu_pitch = p;
  g_state.imu_yaw = y;
  seqlock_write_end(g_state.seq);

}


//// PID declarations
PID pidRoll(0.8f, 0.4f, 0.02f); 
PID pidPitch(0.8f, 0.4f, 0.02f);
PID pidYaw(0.8f, 0.4f, 0.02f);


// ======== Core 0 (USB @ 10 Hz and PID LOOPS for RPY) ========
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Dual-core iBUS (6ch) with seqlock + extensible SharedState @10Hz");
}

void loop() {
  static uint32_t last_print = 0;
  uint32_t now = millis();
  if (now - last_print >= PRINT_PERIOD_MS) {
    last_print = now;

    // Get a consistent snapshot for printing
    SharedState snap;
    if (seqlock_read_consistent(g_state, snap)) {
      uint32_t age = now - snap.last_update_ms;

      Serial.print("CH: ");
      for (int i = 0; i < NUM_CH; ++i) {
        Serial.print(snap.channels[i]);
        if (i < NUM_CH - 1) Serial.print(' ');
      }
      Serial.print("R: ");
      Serial.print(snap.imu_roll);
      Serial.print("P: ");
      Serial.print(snap.imu_pitch);
      Serial.print("Y: ");
      Serial.print(snap.imu_yaw);
      Serial.print(" | age: ");
      Serial.print(age);
      Serial.print(" ms");
      Serial.print(" | frames: ");
      Serial.print(snap.frame_count);
      Serial.print(" | bad: ");
      Serial.print(snap.bad_checksum);
      Serial.print(" | lost: ");
      Serial.println(snap.lost_sync);
    }
  }
}

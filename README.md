# EasyFlight ✈️  
A compact and configurable **flight controller** for RC planes and drones, featuring dual-core processing, real-time telemetry, and a GUI for debugging and tuning.  

---

## Features
- Runs on **RP2040 dual-core MCU**  
- **IMU processing** with Madgwick filter  
- **iBUS decoding** for RC input  
- **PID loops** for roll, pitch, and yaw stabilization  
- **PWM outputs** for servos and ESCs  
- **Telemetry streaming** over USB serial  
- **Black box logging** to onboard flash (16 MB)  
- Python-based GUI for **real-time visualization** and playback from logs  

---

## Hardware
- **MCU**: Raspberry Pi RP2040 (dual-core Cortex-M0+)  
- **IMU**: 6-DoF sensor (accelerometer + gyroscope)  
- **Microphone** *(experimental)*: testing for barometric/altitude estimation  
- **Flash**: 16 MB onboard for black box logging  
- **RC Input**: iBUS (FlySky)  
- **Outputs**: 4 PWM channels (ESCs / servos)  

---

## System Design
- **Core 1** → IMU sensor fusion (Madgwick filter) + additional sensor processing  
- **Core 0** → iBUS decoding, PID control loops, PWM signal generation, telemetry output  
- **Python GUI** → plots telemetry in real-time, or replays black box logs for post-flight analysis  

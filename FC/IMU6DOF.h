// IMU6DOF.h
#ifndef IMU6DOF_H
#define IMU6DOF_H

#include <Arduino_LSM6DSOX.h>

class IMU6DOF {
  public:
    // Constructor
    IMU6DOF();

    // Initialization and calibration
    bool init();
    void calculateIMUError();
    void calibrateAttitude();

    // Main interface
    void update(); // fetch data and update quaternion/angles
    void setLoopFrequency(int freq);

    // Attitude access
    float getRoll();
    float getPitch();
    float getYaw();

  private:
    // Filter constants
    float B_madgwick = 0.04f;
    float B_accel    = 0.14f;
    float B_gyro     = 0.10f;

    // Calibration errors
    volatile float AccErrorX = -0.03;
    volatile float AccErrorY = -0.01;
    volatile float AccErrorZ = 0.01;
    volatile float GyroErrorX = 0.11;
    volatile float GyroErrorY = 0.27;
    volatile float GyroErrorZ = -0.06;

    // Sensor values
    volatile float AccX, AccY, AccZ, AccX_prev, AccY_prev, AccZ_prev;
    volatile float GyroX, GyroY, GyroZ, GyroX_prev, GyroY_prev, GyroZ_prev;

    // Quaternion and Euler
    volatile float q0, q1, q2, q3;
    volatile float roll, pitch, yaw;

    // Timing
    unsigned long now_us, last_us;
    volatile float dt_fc;
    int target_freq;

    bool getIMUdata();
    void Madgwick6DOF(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt);
    inline float invSqrt(float x);
};

#endif

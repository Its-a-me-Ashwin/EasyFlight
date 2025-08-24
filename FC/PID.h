#ifndef PID_H
#define PID_H

#include <stdint.h>

class PID {
public:
  // Construct with gains; default output limits = [-1, 1], derivative filter disabled (tau=0)
  PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

  // Set (or update) PID tunings
  void setTunings(float kp, float ki, float kd);

  // Output limits (min < max). Defaults to [-1, 1].
  void setOutputLimits(float minOut, float maxOut);

  // Typical tau is small, e.g. 0.01–0.05 s for fast IMU loops.
  void setDerivativeFilterTau(float tau_seconds);

  // Reset internal state (integrator, derivative filter, previous measurement/error)
  void reset(float currentMeasurement = 0.0f);

  // Compute controller output for given target, current, and dt (seconds).
  // Returns value clamped within output limits (default [-1, 1]).
  float compute(float target, float current, float dt_seconds);

  // Accessors (optional)
  inline float getKp() const { return _kp; }
  inline float getKi() const { return _ki; }
  inline float getKd() const { return _kd; }

  inline float getLastOutput() const { return _lastOutput; }
  inline float getIntegrator() const { return _integrator; }

private:
  // Gains
  float _kp, _ki, _kd;

  // Output clamp
  float _outMin, _outMax;

  // Derivative filter (exponential smoothing of dInput)
  float _tau;          // time constant (s); <=0 => disabled
  float _dFilt;        // filtered dInput state
  bool  _dFiltInit;

  // Integrator and history
  float _integrator;
  float _prevMeas;
  bool  _hasPrev;

  // Last output
  float _lastOutput;

  // Helpers
  static inline float clamp(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }
};

#endif // PID_H

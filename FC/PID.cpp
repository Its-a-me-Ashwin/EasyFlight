#include "PID.h"

PID::PID(float kp, float ki, float kd)
: _kp(kp), _ki(ki), _kd(kd),
  _outMin(-1.0f), _outMax(1.0f),
  _tau(0.0f), _dFilt(0.0f), _dFiltInit(false),
  _integrator(0.0f), _prevMeas(0.0f), _hasPrev(false),
  _lastOutput(0.0f)
{}

void PID::setTunings(float kp, float ki, float kd) {
  _kp = kp; _ki = ki; _kd = kd;
}

void PID::setOutputLimits(float minOut, float maxOut) {
  if (minOut >= maxOut) return; // ignore invalid
  _outMin = minOut;
  _outMax = maxOut;
  // Re-clamp current state/output to new limits
  _integrator = clamp(_integrator, _outMin, _outMax);
  _lastOutput = clamp(_lastOutput, _outMin, _outMax);
}

void PID::setDerivativeFilterTau(float tau_seconds) {
  _tau = tau_seconds;
  _dFiltInit = false; // reinitialize on next compute
}

void PID::reset(float currentMeasurement) {
  _integrator = 0.0f;
  _prevMeas   = currentMeasurement;
  _hasPrev    = false;
  _dFilt      = 0.0f;
  _dFiltInit  = false;
  _lastOutput = 0.0f;
}

float PID::compute(float target, float current, float dt) {
  if (dt <= 0.0f) {
    // Return last output if dt is invalid
    return _lastOutput;
  }

  // Error
  const float error = target - current;

  // ----- Derivative (on measurement) -----
  // Raw derivative of measurement
  float dInput = 0.0f;
  if (_hasPrev) {
    dInput = (current - _prevMeas) / dt; // note: derivative on measurement
  } else {
    // First call: no derivative
    dInput = 0.0f;
    _hasPrev = true;
  }
  _prevMeas = current;

  // Optional low-pass filter on derivative
  float dInputFilt = dInput;
  if (_tau > 0.0f) {
    // alpha = dt / (tau + dt)
    const float alpha = dt / (_tau + dt);
    if (!_dFiltInit) {
      _dFilt = dInput;   // initialize to first value to avoid a spike
      _dFiltInit = true;
    } else {
      _dFilt = _dFilt + alpha * (dInput - _dFilt);
    }
    dInputFilt = _dFilt;
  }

  // PID terms (note: derivative on measurement => negative contribution)
  const float P = _kp * error;
  const float D = -_kd * dInputFilt;

  // ----- Conditional Integration (anti-windup) -----
  // Propose a new integrator value, then check saturation.
  float I_candidate = _integrator + _ki * error * dt;

  // Tentative unsaturated output
  float u_unsat = P + I_candidate + D;

  // Final (clamped) output
  float u = clamp(u_unsat, _outMin, _outMax);

  // Only accept integrator update if:
  //  - we didn't saturate, OR
  //  - we saturated high and error would drive output back down (error < 0), OR
  //  - we saturated low  and error would drive output back up   (error > 0)
  bool saturatedHigh = (u >= _outMax - 1e-6f);
  bool saturatedLow  = (u <= _outMin + 1e-6f);
  bool acceptI =
      (!saturatedHigh && !saturatedLow) ||
      (saturatedHigh && (error < 0.0f)) ||
      (saturatedLow  && (error > 0.0f));

  if (acceptI) {
    _integrator = I_candidate;
    // Optional: clamp integrator to output bounds to avoid extreme stored energy
    _integrator = clamp(_integrator, _outMin, _outMax);
    // Recompute with accepted integrator (keeps u consistent if we want)
    u_unsat = P + _integrator + D;
    u = clamp(u_unsat, _outMin, _outMax);
  }

  _lastOutput = u;
  return u;
}

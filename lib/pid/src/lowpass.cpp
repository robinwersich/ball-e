#include "lowpass.h"

LowPassFilter::LowPassFilter(const LowPassCoefficients& coefficients) : _coeff(coefficients) {}

float LowPassFilter::filter(float signal) {
  float output = _coeff.a1 * _prev_output + _coeff.b0 * signal + _coeff.b1 * _prev_signal;
  _prev_signal = signal;
  _prev_output = output;
  return output;
}

float LowPassFilter::filtered_derivative(float signal, float dt) {
  const auto prev_output = _prev_output;
  return (filter(signal) - prev_output) / dt;
}

void LowPassFilter::reset() {
  _prev_signal = 0;
  _prev_output = 0;
}

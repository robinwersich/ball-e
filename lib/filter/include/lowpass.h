#pragma once

/**
 * The coefficients for a first order low pass filter.
 * These can be calculated from the desired cutoff and sampling frequency
 * using the `LowPassFilter` python script in this repository.
 */
struct LowPassCoefficients {
  /** Factor for the previous output. */
  float a1 = 0;
  /** Factor for the current signal. */
  float b0 = 1;
  /** Factor for the previous signal. */
  float b1 = 0;
};

/** This implements a simple first order low pass filter. */
template <typename T = float> class LowPassFilter {
 public:
  /** Creates a lowpass filter with */
  LowPassFilter(const LowPassCoefficients& coefficients) : _coeff{coefficients} {}

  /**
   * Returns the filtered output of the signal.
   * @note This must be called at the sampling frequency used to calculate the coefficients.
   */
  T filter(T signal) {
    T output = _coeff.a1 * _prev_output + _coeff.b0 * signal + _coeff.b1 * _prev_signal;
    _prev_signal = signal;
    _prev_output = output;
    return output;
  }

  /** Returns the derivative of the filtered input signal. */
  template <typename TTime> T filtered_derivative(T signal, TTime dt) {
    const auto prev_output = _prev_output;
    return (filter(signal) - prev_output) / dt;
  }

  /** Resets the internal state of the filter. */
  void reset() {
    _prev_signal = {};
    _prev_output = {};
  }

 private:
  LowPassCoefficients _coeff;
  T _prev_signal = {};
  T _prev_output = {};
};

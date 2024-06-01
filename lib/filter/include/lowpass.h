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
class LowPassFilter {
 public:
  /** Creates a lowpass filter with */
  LowPassFilter(const LowPassCoefficients& coefficients);

  /**
   * Returns the filtered output of the signal.
   * @note This must be called at the sampling frequency used to calculate the coefficients.
   */
  float filter(float signal);

  /** Returns the derivative of the filtered input signal. */
  float filtered_derivative(float signal, float dt);

  /** Resets the internal state of the filter. */
  void reset();

 private:
  LowPassCoefficients _coeff;
  float _prev_signal = 0;
  float _prev_output = 0;
};

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sqrtm2by2.cpp
//
// Code generation for function 'sqrtm2by2'
//

// Include files
#include "sqrtm2by2.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
void sqrtm2by2(double T[4])
{
  if (T[1] != 0.0) {
    double mu;
    double r11;
    mu = std::sqrt(-T[1] * T[2]);
    if (T[0] > 0.0) {
      if (T[0] < mu) {
        r11 = T[0] / mu;
        r11 = mu * std::sqrt(r11 * r11 + 1.0);
      } else if (T[0] > mu) {
        double r22;
        r22 = mu / T[0];
        r11 = T[0] * std::sqrt(r22 * r22 + 1.0);
      } else if (std::isnan(mu)) {
        r11 = rtNaN;
      } else {
        r11 = T[0] * 1.4142135623730951;
      }
      r11 = std::sqrt((T[0] + r11) / 2.0);
    } else {
      r11 = std::abs(T[0]);
      if (r11 < mu) {
        r11 /= mu;
        r11 = mu * std::sqrt(r11 * r11 + 1.0);
      } else if (r11 > mu) {
        double r22;
        r22 = mu / r11;
        r11 *= std::sqrt(r22 * r22 + 1.0);
      } else if (std::isnan(mu)) {
        r11 = rtNaN;
      } else {
        r11 *= 1.4142135623730951;
      }
      r11 = mu / std::sqrt(2.0 * (-T[0] + r11));
    }
    T[0] = r11;
    T[3] = r11;
    T[2] /= 2.0 * r11;
    T[1] /= 2.0 * r11;
  } else {
    double r11;
    double r22;
    r11 = std::sqrt(T[0]);
    r22 = std::sqrt(T[3]);
    T[3] = r22;
    T[0] = r11;
    T[2] /= r11 + r22;
  }
}

} // namespace coder

// End of code generation (sqrtm2by2.cpp)

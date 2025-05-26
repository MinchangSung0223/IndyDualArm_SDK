//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// checkCondition.cpp
//
// Code generation for function 'checkCondition'
//

// Include files
#include "checkCondition.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
bool checkCondition(const creal_T z)
{
  double b;
  double b_tmp;
  double y;
  bool c;
  y = std::abs(z.re);
  b_tmp = std::abs(z.im);
  if (y < b_tmp) {
    y /= b_tmp;
    y = b_tmp * std::sqrt(y * y + 1.0);
  } else if (y > b_tmp) {
    b = b_tmp / y;
    y *= std::sqrt(b * b + 1.0);
  } else if (std::isnan(b_tmp)) {
    y = rtNaN;
  } else {
    y *= 1.4142135623730951;
  }
  if (y > 90.509667991878089) {
    c = true;
  } else {
    y = std::abs(z.re - 1.0);
    if (y < b_tmp) {
      y /= b_tmp;
      y = b_tmp * std::sqrt(y * y + 1.0);
    } else if (y > b_tmp) {
      b = b_tmp / y;
      y *= std::sqrt(b * b + 1.0);
    } else if (std::isnan(b_tmp)) {
      y = rtNaN;
    } else {
      y *= 1.4142135623730951;
    }
    if (y < 0.011048543456039804) {
      c = true;
    } else {
      y = std::abs(z.re + 1.0);
      if (y < b_tmp) {
        y /= b_tmp;
        y = b_tmp * std::sqrt(y * y + 1.0);
      } else if (y > b_tmp) {
        b = b_tmp / y;
        y *= std::sqrt(b * b + 1.0);
      } else if (std::isnan(b_tmp)) {
        y = rtNaN;
      } else {
        y *= 1.4142135623730951;
      }
      if (y < 0.011048543456039804) {
        c = true;
      } else {
        c = false;
      }
    }
  }
  return c;
}

} // namespace coder

// End of code generation (checkCondition.cpp)

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// norm.cpp
//
// Code generation for function 'norm'
//

// Include files
#include "norm.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
double b_norm(const creal_T x[4])
{
  double y;
  y = 0.0;
  for (int k{0}; k < 4; k++) {
    double absx;
    double b;
    absx = std::abs(x[k].re);
    b = std::abs(x[k].im);
    if (absx < b) {
      absx /= b;
      absx = b * std::sqrt(absx * absx + 1.0);
    } else if (absx > b) {
      b /= absx;
      absx *= std::sqrt(b * b + 1.0);
    } else if (std::isnan(b)) {
      absx = rtNaN;
    } else {
      absx *= 1.4142135623730951;
    }
    if (std::isnan(absx) || (absx > y)) {
      y = absx;
    }
  }
  return y;
}

double c_norm(const creal_T x[16])
{
  double y;
  int j;
  bool exitg1;
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    double b;
    double b_y;
    double s;
    int a_tmp;
    a_tmp = j << 2;
    b_y = std::abs(x[a_tmp].re);
    b = std::abs(x[a_tmp].im);
    if (b_y < b) {
      b_y /= b;
      b_y = b * std::sqrt(b_y * b_y + 1.0);
    } else if (b_y > b) {
      b /= b_y;
      b_y *= std::sqrt(b * b + 1.0);
    } else if (std::isnan(b)) {
      b_y = rtNaN;
    } else {
      b_y *= 1.4142135623730951;
    }
    s = b_y;
    b_y = std::abs(x[a_tmp + 1].re);
    b = std::abs(x[a_tmp + 1].im);
    if (b_y < b) {
      b_y /= b;
      b_y = b * std::sqrt(b_y * b_y + 1.0);
    } else if (b_y > b) {
      b /= b_y;
      b_y *= std::sqrt(b * b + 1.0);
    } else if (std::isnan(b)) {
      b_y = rtNaN;
    } else {
      b_y *= 1.4142135623730951;
    }
    s += b_y;
    b_y = std::abs(x[a_tmp + 2].re);
    b = std::abs(x[a_tmp + 2].im);
    if (b_y < b) {
      b_y /= b;
      b_y = b * std::sqrt(b_y * b_y + 1.0);
    } else if (b_y > b) {
      b /= b_y;
      b_y *= std::sqrt(b * b + 1.0);
    } else if (std::isnan(b)) {
      b_y = rtNaN;
    } else {
      b_y *= 1.4142135623730951;
    }
    s += b_y;
    b_y = std::abs(x[a_tmp + 3].re);
    b = std::abs(x[a_tmp + 3].im);
    if (b_y < b) {
      b_y /= b;
      b_y = b * std::sqrt(b_y * b_y + 1.0);
    } else if (b_y > b) {
      b /= b_y;
      b_y *= std::sqrt(b * b + 1.0);
    } else if (std::isnan(b)) {
      b_y = rtNaN;
    } else {
      b_y *= 1.4142135623730951;
    }
    s += b_y;
    if (std::isnan(s)) {
      y = rtNaN;
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }
      j++;
    }
  }
  return y;
}

} // namespace coder

// End of code generation (norm.cpp)

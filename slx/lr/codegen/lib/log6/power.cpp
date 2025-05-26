//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// power.cpp
//
// Code generation for function 'power'
//

// Include files
#include "power.h"
#include "log.h"
#include "log6_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
creal_T power(const creal_T a, double b)
{
  creal_T y;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, b);
    y.im = 0.0;
  } else if ((a.re == 0.0) && (std::floor(b) == b)) {
    y.re = 0.0;
    y.im = -rt_powd_snf(a.im, -1.0);
  } else if ((a.im == 0.0) && std::isinf(b) && (std::abs(a.re) == 1.0)) {
    y.re = 1.0;
    y.im = 0.0;
  } else {
    double r;
    double y_tmp;
    y = a;
    b_log(y);
    r = b * y.re;
    y_tmp = b * y.im;
    if (r == 0.0) {
      y.re = std::cos(y_tmp);
      y.im = std::sin(y_tmp);
    } else if (y_tmp == 0.0) {
      y.re = std::exp(r);
      y.im = 0.0;
    } else if (std::isinf(y_tmp) && std::isinf(r) && (r < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y.re = r * (r * std::cos(y_tmp));
      y.im = r * (r * std::sin(y_tmp));
    }
  }
  return y;
}

} // namespace coder

// End of code generation (power.cpp)

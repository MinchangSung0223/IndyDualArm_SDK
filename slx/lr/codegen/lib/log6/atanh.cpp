//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// atanh.cpp
//
// Code generation for function 'atanh'
//

// Include files
#include "atanh.h"
#include "log6_data.h"
#include "log6_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
void b_atanh(creal_T &x)
{
  double xi;
  double xr;
  xr = std::abs(x.re);
  xi = std::abs(x.im);
  if ((xr > 3.3519519824856489E+153) || (xi > 3.3519519824856489E+153)) {
    if (xi == 0.0) {
      xr = 1.0 / xr;
    } else if (xr == 0.0) {
      xr = 0.0;
    } else if (xr > xi) {
      double absz;
      absz = xi / xr;
      xr = (absz * 0.0 + 1.0) / (xr + absz * xi);
    } else if (xi == xr) {
      xr = 0.5 / xr;
    } else {
      double absz;
      absz = xr / xi;
      xr = absz / (xi + absz * xr);
    }
    xi = 1.5707963267948966;
  } else if ((xr == 1.0) && (xi == 0.0)) {
    xr = rtInf;
  } else if (xr == 1.0) {
    xr = std::log(std::sqrt(std::sqrt(xi * xi + 4.0)) /
                  std::sqrt(xi + 2.9833362924800834E-154));
    xi =
        (std::atan((xi + 2.9833362924800834E-154) / 2.0) + 1.5707963267948966) /
        2.0;
  } else if ((xi == 0.0) && (!(xr > 1.0))) {
    if (xr < 0.5) {
      double t;
      t = xr + xr;
      t += t * (xr / (1.0 - xr));
      if (!(t < 2.2204460492503131E-16)) {
        t = std::log(t + 1.0) * (t / ((t + 1.0) - 1.0));
      }
      xr = t / 2.0;
    } else if (xr == 1.0) {
      xr = rtInf;
    } else {
      double t;
      t = (xr + xr) / (1.0 - xr);
      if ((t > 4.503599627370496E+15) || std::isnan(t)) {
        t++;
        t = std::log(t);
      } else {
        t = std::log(t + 1.0) * (t / ((t + 1.0) - 1.0));
      }
      xr = t / 2.0;
    }
  } else {
    double absz;
    double t;
    t = (xi + 2.9833362924800834E-154) * (xi + 2.9833362924800834E-154);
    xi = rt_atan2d_snf(2.0 * xi, (1.0 - xr) * (xr + 1.0) - t) / 2.0;
    t = 4.0 * (xr / ((1.0 - xr) * (1.0 - xr) + t));
    absz = std::abs(t);
    if ((absz > 4.503599627370496E+15) || (std::isinf(t) || std::isnan(t))) {
      t++;
      t = std::log(t);
    } else if (!(absz < 2.2204460492503131E-16)) {
      t = std::log(t + 1.0) * (t / ((t + 1.0) - 1.0));
    }
    xr = t / 4.0;
  }
  if (x.re < 0.0) {
    xr = -xr;
  }
  if ((x.im < 0.0) || ((x.im == 0.0) && (x.re < -1.0))) {
    xi = -xi;
  }
  x.re = xr;
  x.im = xi;
}

void b_atanh(double &x)
{
  bool negx;
  if (x < 0.0) {
    negx = true;
    x = -x;
  } else {
    negx = false;
  }
  if (x > 1.0) {
    x = rtNaN;
  } else if (x < 0.5) {
    double t;
    t = x + x;
    t += t * (x / (1.0 - x));
    if (!(t < 2.2204460492503131E-16)) {
      t = std::log(t + 1.0) * (t / ((t + 1.0) - 1.0));
    }
    x = t / 2.0;
  } else if (x == 1.0) {
    x = rtInf;
  } else {
    double t;
    t = (x + x) / (1.0 - x);
    if ((t > 4.503599627370496E+15) || std::isnan(t)) {
      t++;
      t = std::log(t);
    } else {
      t = std::log(t + 1.0) * (t / ((t + 1.0) - 1.0));
    }
    x = t / 2.0;
  }
  if (negx) {
    x = -x;
  }
}

} // namespace coder

// End of code generation (atanh.cpp)

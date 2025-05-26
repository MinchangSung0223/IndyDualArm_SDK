//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdlanv2.cpp
//
// Code generation for function 'xdlanv2'
//

// Include files
#include "xdlanv2.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
double xdlanv2(double &a, double &b, double &c, double &d, double &rt1i,
               double &rt2r, double &rt2i, double &cs, double &sn)
{
  double rt1r;
  if (c == 0.0) {
    cs = 1.0;
    sn = 0.0;
  } else if (b == 0.0) {
    cs = 0.0;
    sn = 1.0;
    rt1r = d;
    d = a;
    a = rt1r;
    b = -c;
    c = 0.0;
  } else {
    rt1r = a - d;
    if ((rt1r == 0.0) && ((b < 0.0) != (c < 0.0))) {
      cs = 1.0;
      sn = 0.0;
    } else {
      double bcmax;
      double bcmis;
      double p;
      double scale;
      double sigma;
      double z;
      int count;
      int i;
      p = 0.5 * rt1r;
      bcmis = std::abs(b);
      sigma = std::abs(c);
      bcmax = std::fmax(bcmis, sigma);
      if (!(b < 0.0)) {
        count = 1;
      } else {
        count = -1;
      }
      if (!(c < 0.0)) {
        i = 1;
      } else {
        i = -1;
      }
      bcmis = std::fmin(bcmis, sigma) * static_cast<double>(count) *
              static_cast<double>(i);
      scale = std::fmax(std::abs(p), bcmax);
      z = p / scale * p + bcmax / scale * bcmis;
      if (z >= 8.8817841970012523E-16) {
        a = std::sqrt(scale) * std::sqrt(z);
        if (!(p < 0.0)) {
          rt1r = a;
        } else {
          rt1r = -a;
        }
        z = p + rt1r;
        a = d + z;
        d -= bcmax / z * bcmis;
        scale = std::abs(z);
        if (sigma < scale) {
          bcmax = sigma / scale;
          scale *= std::sqrt(bcmax * bcmax + 1.0);
        } else if (sigma > scale) {
          scale /= sigma;
          scale = sigma * std::sqrt(scale * scale + 1.0);
        } else if (std::isnan(scale)) {
          scale = rtNaN;
        } else {
          scale = sigma * 1.4142135623730951;
        }
        cs = z / scale;
        sn = c / scale;
        b -= c;
        c = 0.0;
      } else {
        sigma = b + c;
        scale = std::fmax(std::abs(rt1r), std::abs(sigma));
        count = 0;
        while ((scale >= 7.4428285367870146E+137) && (count <= 20)) {
          sigma *= 1.3435752215134178E-138;
          rt1r *= 1.3435752215134178E-138;
          scale = std::fmax(std::abs(rt1r), std::abs(sigma));
          count++;
        }
        while ((scale <= 1.3435752215134178E-138) && (count <= 20)) {
          sigma *= 7.4428285367870146E+137;
          rt1r *= 7.4428285367870146E+137;
          scale = std::fmax(std::abs(rt1r), std::abs(sigma));
          count++;
        }
        bcmis = std::abs(sigma);
        scale = std::abs(rt1r);
        if (bcmis < scale) {
          bcmax = bcmis / scale;
          scale *= std::sqrt(bcmax * bcmax + 1.0);
        } else if (bcmis > scale) {
          scale /= bcmis;
          scale = bcmis * std::sqrt(scale * scale + 1.0);
        } else if (std::isnan(scale)) {
          scale = rtNaN;
        } else {
          scale = bcmis * 1.4142135623730951;
        }
        cs = std::sqrt(0.5 * (bcmis / scale + 1.0));
        if (!(sigma < 0.0)) {
          count = 1;
        } else {
          count = -1;
        }
        sn = -(0.5 * rt1r / (scale * cs)) * static_cast<double>(count);
        sigma = a * cs + b * sn;
        bcmax = -a * sn + b * cs;
        scale = c * cs + d * sn;
        bcmis = -c * sn + d * cs;
        b = bcmax * cs + bcmis * sn;
        c = -sigma * sn + scale * cs;
        rt1r = 0.5 * ((sigma * cs + scale * sn) + (-bcmax * sn + bcmis * cs));
        a = rt1r;
        d = rt1r;
        if (c != 0.0) {
          if (b != 0.0) {
            if ((b < 0.0) == (c < 0.0)) {
              bcmax = std::sqrt(std::abs(b));
              bcmis = std::sqrt(std::abs(c));
              a = bcmax * bcmis;
              if (!(c < 0.0)) {
                p = a;
              } else {
                p = -a;
              }
              scale = 1.0 / std::sqrt(std::abs(b + c));
              a = rt1r + p;
              d = rt1r - p;
              b -= c;
              c = 0.0;
              bcmax *= scale;
              bcmis *= scale;
              rt1r = cs * bcmax - sn * bcmis;
              sn = cs * bcmis + sn * bcmax;
              cs = rt1r;
            }
          } else {
            b = -c;
            c = 0.0;
            rt1r = cs;
            cs = -sn;
            sn = rt1r;
          }
        }
      }
    }
  }
  rt1r = a;
  rt2r = d;
  if (c == 0.0) {
    rt1i = 0.0;
    rt2i = 0.0;
  } else {
    rt1i = std::sqrt(std::abs(b)) * std::sqrt(std::abs(c));
    rt2i = -rt1i;
  }
  return rt1r;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xdlanv2.cpp)

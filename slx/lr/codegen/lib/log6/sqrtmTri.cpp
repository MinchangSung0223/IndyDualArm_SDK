//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sqrtmTri.cpp
//
// Code generation for function 'sqrtmTri'
//

// Include files
#include "sqrtmTri.h"
#include "log6_data.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "sylvesterRecursive.h"
#include <cmath>

// Function Definitions
namespace coder {
void sqrtmTriRecursive(creal_T T[16], int i, int j, int m)
{
  if (m == 1) {
    internal::scalar::b_sqrt(T[(i + ((j - 1) << 2)) - 1]);
  } else if (m == 2) {
    creal_T r11;
    creal_T r22;
    double ai;
    double ar;
    double bi;
    double br;
    double im;
    double re;
    int m1;
    int m2;
    m1 = (i + ((j - 1) << 2)) - 1;
    r11 = T[m1];
    internal::scalar::b_sqrt(r11);
    m2 = i + (j << 2);
    r22 = T[m2];
    internal::scalar::b_sqrt(r22);
    T[m2] = r22;
    T[m1] = r11;
    ar = T[m2 - 1].re;
    ai = T[m2 - 1].im;
    br = r11.re + r22.re;
    bi = r11.im + r22.im;
    if (bi == 0.0) {
      if (ai == 0.0) {
        re = ar / br;
        im = 0.0;
      } else if (ar == 0.0) {
        re = 0.0;
        im = ai / br;
      } else {
        re = ar / br;
        im = ai / br;
      }
    } else if (br == 0.0) {
      if (ar == 0.0) {
        re = ai / bi;
        im = 0.0;
      } else if (ai == 0.0) {
        re = 0.0;
        im = -(ar / bi);
      } else {
        re = ai / bi;
        im = -(ar / bi);
      }
    } else {
      double brm;
      brm = std::abs(br);
      im = std::abs(bi);
      if (brm > im) {
        double s;
        s = bi / br;
        im = br + s * bi;
        re = (ar + s * ai) / im;
        im = (ai - s * ar) / im;
      } else if (im == brm) {
        double s;
        if (br > 0.0) {
          s = 0.5;
        } else {
          s = -0.5;
        }
        if (bi > 0.0) {
          im = 0.5;
        } else {
          im = -0.5;
        }
        re = (ar * s + ai * im) / brm;
        im = (ai * s - ar * im) / brm;
      } else {
        double s;
        s = br / bi;
        im = bi + s * br;
        re = (s * ar + ai) / im;
        im = (s * ai - ar) / im;
      }
    }
    T[m2 - 1].re = re;
    T[m2 - 1].im = im;
  } else {
    int b_i;
    int i1;
    int m1;
    int m2;
    m1 = m / 2;
    m2 = m - m1;
    sqrtmTriRecursive(T, i, j, m1);
    b_i = i + m1;
    i1 = j + m1;
    sqrtmTriRecursive(T, b_i, i1, m2);
    sylvesterRecursive(i, j, b_i, i1, T, i, i1, m1, m2);
  }
}

void sqrtmTriRecursive(double T[16], int i, int j, int m)
{
  if (m == 1) {
    int b_i;
    b_i = (i + ((j - 1) << 2)) - 1;
    T[b_i] = std::sqrt(T[b_i]);
  } else if (m == 2) {
    double d;
    int b_i;
    b_i = i + ((j - 1) << 2);
    d = T[b_i];
    if (d != 0.0) {
      double d1;
      double mu;
      double r11;
      double t12_tmp;
      int m1;
      m1 = i + (j << 2);
      t12_tmp = T[m1 - 1];
      mu = std::sqrt(-d * t12_tmp);
      d1 = T[b_i - 1];
      if (d1 > 0.0) {
        if (d1 < mu) {
          r11 = d1 / mu;
          r11 = mu * std::sqrt(r11 * r11 + 1.0);
        } else if (d1 > mu) {
          double r22;
          r22 = mu / d1;
          r11 = d1 * std::sqrt(r22 * r22 + 1.0);
        } else if (std::isnan(mu)) {
          r11 = rtNaN;
        } else {
          r11 = d1 * 1.4142135623730951;
        }
        r11 = std::sqrt((d1 + r11) / 2.0);
      } else {
        r11 = std::abs(T[b_i - 1]);
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
        r11 = mu / std::sqrt(2.0 * (-d1 + r11));
      }
      T[b_i - 1] = r11;
      T[m1] = r11;
      T[m1 - 1] = t12_tmp / (2.0 * r11);
      T[b_i] = d / (2.0 * r11);
    } else {
      double r11;
      double r22;
      int m1;
      r11 = std::sqrt(T[b_i - 1]);
      m1 = i + (j << 2);
      r22 = std::sqrt(T[m1]);
      T[m1] = r22;
      T[b_i - 1] = r11;
      T[m1 - 1] /= r11 + r22;
    }
  } else {
    int b_i;
    int i1;
    int m1;
    int m2;
    m1 = m / 2;
    if (T[((i + m1) + (((j + m1) - 2) << 2)) - 1] != 0.0) {
      m1++;
    }
    m2 = m - m1;
    sqrtmTriRecursive(T, i, j, m1);
    b_i = i + m1;
    i1 = j + m1;
    sqrtmTriRecursive(T, b_i, i1, m2);
    sylvesterRecursive(i, j, b_i, i1, T, i, i1, m1, m2);
  }
}

} // namespace coder

// End of code generation (sqrtmTri.cpp)
